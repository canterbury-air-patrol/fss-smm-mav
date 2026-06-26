#include <catch2/catch_test_macros.hpp>

#include "mav/internal.hpp"

#include <ardupilotmega/mavlink.h>

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

/* Integration harness for the real mav_connection over a loopback TCP socket.
 *
 * These tests drive the genuinely hard, threaded I/O boundary that the pure
 * Catch2 cases in fmu_test.cpp cannot reach: connect/recv/teardown, the
 * heartbeat-driven comms status, and (later) the fd lifecycle under reconnect
 * and the mission-upload sequence. They use real sockets on 127.0.0.1 and poll
 * with timeouts rather than fixed sleeps so they stay deterministic, and are run
 * under TSan in CI to exercise the threading. */

namespace
{

/* A minimal stand-in for the autopilot end of the MAVLink link: it listens on an
 * ephemeral loopback port, accepts a single connection, and lets the test inject
 * MAVLink frames or drop the connection mid-stream. */
class MavLoopbackServer
{
  public:
    MavLoopbackServer ()
    {
        this->listen_fd = socket (AF_INET, SOCK_STREAM, 0);
        REQUIRE (this->listen_fd >= 0);
        int one = 1;
        setsockopt (this->listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof (one));

        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl (INADDR_LOOPBACK);
        addr.sin_port = 0; /* let the kernel pick a free port */
        REQUIRE (bind (this->listen_fd, reinterpret_cast<struct sockaddr *> (&addr), sizeof (addr)) == 0);

        socklen_t len = sizeof (addr);
        REQUIRE (getsockname (this->listen_fd, reinterpret_cast<struct sockaddr *> (&addr), &len) == 0);
        this->listen_port = ntohs (addr.sin_port);

        REQUIRE (listen (this->listen_fd, 1) == 0);

        this->accept_thread = std::thread (
            [this] ()
            {
                int fd = accept (this->listen_fd, nullptr, nullptr);
                this->client_fd.store (fd);
            });
    }

    MavLoopbackServer (const MavLoopbackServer &) = delete;
    MavLoopbackServer (MavLoopbackServer &&) = delete;
    auto operator= (const MavLoopbackServer &) -> MavLoopbackServer & = delete;
    auto operator= (MavLoopbackServer &&) -> MavLoopbackServer & = delete;

    ~MavLoopbackServer ()
    {
        /* Closing the listen socket unblocks a still-pending accept() on Linux. */
        if (this->listen_fd >= 0)
        {
            shutdown (this->listen_fd, SHUT_RDWR);
            close (this->listen_fd);
        }
        if (this->accept_thread.joinable ())
        {
            this->accept_thread.join ();
        }
        this->dropClient ();
    }

    auto
    port () const -> uint16_t
    {
        return this->listen_port;
    }

    /* True once the FMU has connected (the accept() completed). */
    auto
    waitForClient (std::chrono::milliseconds timeout) -> bool
    {
        return waitFor ([this] () { return this->client_fd.load () >= 0; }, timeout);
    }

    /* Send a HEARTBEAT as if from the autopilot (sysid 1), so the FMU records a
     * fresh last_heartbeat_ts and the heartbeat loop reports the link up. */
    void
    sendHeartbeat (uint8_t type = MAV_TYPE_QUADROTOR)
    {
        int fd = this->client_fd.load ();
        if (fd < 0)
        {
            return;
        }
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack (1, 1, &msg, type, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        unsigned int len = mavlink_msg_to_send_buffer (buf, &msg);
        ssize_t sent = send (fd, buf, len, MSG_NOSIGNAL);
        (void)sent;
    }

    /* Drop the accepted connection to simulate a mid-stream link loss. */
    void
    dropClient ()
    {
        int fd = this->client_fd.exchange (-1);
        if (fd >= 0)
        {
            shutdown (fd, SHUT_RDWR);
            close (fd);
        }
    }

    template <typename Pred>
    static auto
    waitFor (Pred pred, std::chrono::milliseconds timeout) -> bool
    {
        const auto deadline = std::chrono::steady_clock::now () + timeout;
        while (std::chrono::steady_clock::now () < deadline)
        {
            if (pred ())
            {
                return true;
            }
            std::this_thread::sleep_for (std::chrono::milliseconds (10));
        }
        return pred ();
    }

  private:
    int listen_fd{ -1 };
    uint16_t listen_port{ 0 };
    std::atomic<int> client_fd{ -1 };
    std::thread accept_thread{};
};

/* Thread-safe record of the comms-status callbacks (invoked from the FMU's
 * heartbeat thread). */
class CommsRecorder
{
  public:
    void
    record (MavCommsStatus status)
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        this->events.push_back (status);
    }

    auto
    sawStatus (MavCommsStatus status) -> bool
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        for (auto event : this->events)
        {
            if (event == status)
            {
                return true;
            }
        }
        return false;
    }

  private:
    std::mutex mtx{};
    std::vector<MavCommsStatus> events{};
};

constexpr uint16_t test_goto_altitude_m = 50;
constexpr uint16_t test_altitude_floor_m = 10;
constexpr uint16_t test_altitude_cap_m = 120;
constexpr auto io_timeout = std::chrono::seconds (8);

} // namespace

TEST_CASE ("mav_connection reports the link up after a heartbeat is received", "[mav_io]")
{
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_goto_altitude_m, test_altitude_floor_m, test_altitude_cap_m);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();

    REQUIRE (server.waitForClient (io_timeout));

    /* The autopilot starts heartbeating: the FMU must report the link up. The
     * heartbeat loop runs ~1 Hz, so keep emitting until it observes one. */
    const bool up = MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat ();
            return recorder.sawStatus (MavCommsStatus::ok);
        },
        io_timeout);
    REQUIRE (up);
}
