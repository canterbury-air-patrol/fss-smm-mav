#include <catch2/catch_test_macros.hpp>

#include "mav/internal.hpp"

#include <ardupilotmega/mavlink.h>

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <netinet/in.h>
#include <optional>
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

/* The test packs its autopilot-side MAVLink on a dedicated channel, distinct from
 * the FMU's recv (COMM_0) and send (COMM_1) channels, so the test thread never
 * contends with the FMU threads on a shared per-channel status (the wire bytes are
 * identical regardless of channel). */
constexpr mavlink_channel_t autopilot_tx_channel = MAVLINK_COMM_2;

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

        /* Accept continuously so a reconnect after a drop is picked up. Each new
         * connection replaces (and closes) any previous client fd. The loop ends
         * when the listen socket is closed in the destructor. */
        this->accept_thread = std::thread (
            [this] ()
            {
                while (true)
                {
                    int fd = accept (this->listen_fd, nullptr, nullptr);
                    if (fd < 0)
                    {
                        /* A signal can interrupt accept(); keep accepting rather
                         * than stopping the harness. Any other error means the
                         * listen socket was closed for teardown, so exit. */
                        if (errno == EINTR)
                        {
                            continue;
                        }
                        return;
                    }
                    int old = this->client_fd.exchange (fd);
                    if (old >= 0)
                    {
                        shutdown (old, SHUT_RDWR);
                        close (old);
                    }
                    this->accept_count.fetch_add (1);
                }
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

    /* Number of connections accepted so far; used to confirm a reconnect produced
     * a genuinely new socket rather than reusing stale state. */
    auto
    acceptCount () const -> int
    {
        return this->accept_count.load ();
    }

    /* Send a HEARTBEAT as if from the autopilot (sysid 1), so the FMU records a
     * fresh last_heartbeat_ts and the heartbeat loop reports the link up. */
    void
    sendHeartbeat (uint8_t type = MAV_TYPE_QUADROTOR)
    {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack_chan (1, 1, autopilot_tx_channel, &msg, type, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0,
                                         MAV_STATE_ACTIVE);
        sendMsg (msg);
    }

    /* Send a GLOBAL_POSITION_INT so the FMU's recv path fires its position
     * callback — a deterministic observable that the link carries data, used to
     * prove a reconnect re-established a working recv path. */
    void
    sendPosition ()
    {
        mavlink_message_t msg;
        mavlink_msg_global_position_int_pack_chan (1, 1, autopilot_tx_channel, &msg, 0, /*lat*/ -435000000,
                                                   /*lon*/ 1726000000,
                                                   /*alt mm*/ 100000, /*rel alt mm*/ 100000, 0, 0, 0, /*hdg*/ 0);
        sendMsg (msg);
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
    void
    sendMsg (const mavlink_message_t &msg)
    {
        int fd = this->client_fd.load ();
        if (fd < 0)
        {
            return;
        }
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        unsigned int len = mavlink_msg_to_send_buffer (buf, &msg);
        ssize_t sent = send (fd, buf, len, MSG_NOSIGNAL);
        (void)sent;
    }

    int listen_fd{ -1 };
    uint16_t listen_port{ 0 };
    std::atomic<int> client_fd{ -1 };
    std::atomic<int> accept_count{ 0 };
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
    lastStatus () -> std::optional<MavCommsStatus>
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        if (this->events.empty ())
        {
            return std::nullopt;
        }
        return this->events.back ();
    }

  private:
    std::mutex mtx{};
    std::vector<MavCommsStatus> events{};
};

constexpr uint16_t test_goto_altitude_m = 50;
constexpr uint16_t test_altitude_floor_m = 10;
constexpr uint16_t test_altitude_cap_m = 120;
constexpr auto io_timeout = std::chrono::seconds (8);

/* mav_connection parses on the single global channel MAVLINK_COMM_0. In the real
 * app there is only ever one connection, but here each test spins up its own, so
 * a previous test (or a previous connection within a test) can leave partial-
 * frame state on that channel and desync the next test's parser. Reset it at the
 * start of each test so the cases are independent. */
void
reset_mav_parser ()
{
    mavlink_reset_channel_status (MAVLINK_COMM_0);
}

/* Thread-safe counter of position callbacks (invoked from the FMU recv thread),
 * a direct observable that the recv path is processing inbound MAVLink. */
class PositionRecorder
{
  public:
    void
    record ()
    {
        this->count.fetch_add (1);
    }
    auto
    count_at_least (int n) -> bool
    {
        return this->count.load () >= n;
    }

  private:
    std::atomic<int> count{ 0 };
};

} // namespace

TEST_CASE ("mav_connection reports the link down at cold start, then up once a heartbeat arrives", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_goto_altitude_m, test_altitude_floor_m, test_altitude_cap_m);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();

    REQUIRE (server.waitForClient (io_timeout));

    /* The link is connected but no heartbeat has arrived: the heartbeat loop must
     * edge-trigger a failure rather than leave the optimistic default unreported.
     * Wait for that down BEFORE sending heartbeats, so the subsequent up is a real
     * down->up edge (the status callback only fires on a change). */
    REQUIRE (
        MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout));

    /* Now the autopilot heartbeats: the FMU must report the link up. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat ();
            return recorder.lastStatus () == MavCommsStatus::ok;
        },
        io_timeout));
}

TEST_CASE ("mav_connection recovers the link after a mid-stream drop and reconnect", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_goto_altitude_m, test_altitude_floor_m, test_altitude_cap_m);
    conn.registerPositionCB ([&positions] (const PositionData &) { positions.record (); });
    conn.start ();

    REQUIRE (server.waitForClient (io_timeout));

    /* Confirm the initial recv path works: a position message reaches the FMU. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendPosition ();
            return positions.count_at_least (1);
        },
        io_timeout));
    const int before_drop = 1;

    /* Drop the link mid-stream. The recv thread sees EOF and flags the connection
     * broken; the reconnector (driven here, as the main loop would) tears the old
     * socket down and dials a fresh one. */
    server.dropClient ();
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            conn.attemptReconnect ();
            return server.acceptCount () >= 2;
        },
        io_timeout));

    /* On the new connection, a position message must again reach the FMU —
     * proving the reconnect re-established a working recv path, not just a
     * socket. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            conn.attemptReconnect ();
            server.sendPosition ();
            return positions.count_at_least (before_drop + 1);
        },
        io_timeout));
}
