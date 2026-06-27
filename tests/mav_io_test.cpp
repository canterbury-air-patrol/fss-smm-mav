#include <catch2/catch_test_macros.hpp>

#include "mav/internal.hpp"
#include "mav/mav.hpp"
#include "mav/mission-plan.hpp"
#include "smm/smm.hpp"

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
#include <sys/time.h>
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
/* The test parses the FMU's traffic on its own channel too, distinct from the TX
 * channel above and the FMU's COMM_0/COMM_1, so no channel status is shared. */
constexpr mavlink_channel_t autopilot_rx_channel = MAVLINK_COMM_3;

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
                    /* Bound recv() so the server-side parse loop can re-check its
                     * deadline rather than block forever waiting for the FMU. */
                    struct timeval tv = { 0, 200000 };
                    setsockopt (fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof (tv));
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

    /* Send a MISSION_REQUEST_INT for `seq` as the autopilot would during a
     * mission upload (addressed to the FMU at SYS_ID/COMP_ID). */
    void
    sendMissionRequestInt (uint16_t seq, uint8_t mission_type)
    {
        mavlink_message_t msg;
        mavlink_msg_mission_request_int_pack_chan (1, 1, autopilot_tx_channel, &msg, fmu_sys_id, fmu_comp_id, seq,
                                                   mission_type);
        sendMsg (msg);
    }

    /* Acknowledge a completed mission upload as accepted. */
    void
    sendMissionAck (uint8_t mission_type)
    {
        mavlink_message_t msg;
        mavlink_msg_mission_ack_pack_chan (1, 1, autopilot_tx_channel, &msg, fmu_sys_id, fmu_comp_id,
                                           MAV_MISSION_ACCEPTED, mission_type, 0);
        sendMsg (msg);
    }

    /* Read and parse inbound MAVLink from the FMU until a message of `want`
     * arrives (skipping heartbeats, mode sets, etc.) or the timeout elapses. */
    auto
    recvMessage (uint32_t want, mavlink_message_t &out, std::chrono::milliseconds timeout) -> bool
    {
        const auto deadline = std::chrono::steady_clock::now () + timeout;
        mavlink_status_t status;
        while (std::chrono::steady_clock::now () < deadline)
        {
            int fd = this->client_fd.load ();
            if (fd < 0)
            {
                std::this_thread::sleep_for (std::chrono::milliseconds (10));
                continue;
            }
            uint8_t buf[1024];
            ssize_t n = recv (fd, buf, sizeof (buf), 0);
            if (n <= 0)
            {
                continue; /* timeout or EOF: the while-condition re-checks the deadline */
            }
            for (ssize_t i = 0; i < n; i++)
            {
                if (mavlink_parse_char (autopilot_rx_channel, buf[i], &out, &status) && out.msgid == want)
                {
                    return true;
                }
            }
        }
        return false;
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
    /* The FMU's MAVLink identity (SYS_ID/COMP_ID in mavlink.cpp): mission
     * requests/acks must be addressed here for the FMU to act on them. */
    static constexpr uint8_t fmu_sys_id = 200;
    static constexpr uint8_t fmu_comp_id = 1;

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
    mavlink_reset_channel_status (MAVLINK_COMM_0);       /* FMU parse channel */
    mavlink_reset_channel_status (autopilot_rx_channel); /* test-side parse channel */
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
    auto
    count_now () const -> int
    {
        return this->count.load ();
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

TEST_CASE ("mav_connection survives repeated drop/reconnect cycles", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_goto_altitude_m, test_altitude_floor_m, test_altitude_cap_m);
    conn.registerPositionCB ([&positions] (const PositionData &) { positions.record (); });
    conn.start ();

    REQUIRE (server.waitForClient (io_timeout));

    /* Confirm a position reaches the FMU on the current connection. */
    auto prove_delivery = [&] ()
    {
        const int before = positions.count_now ();
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPosition ();
                return positions.count_now () > before;
            },
            io_timeout));
    };

    prove_delivery ();

    /* Hammer the fd lifecycle: each cycle drops the link and drives a reconnect,
     * so the recv thread is repeatedly torn down (close while it may be in recv)
     * and a fresh socket dialled — the close-during-recv and fd-reuse paths. Run
     * under TSan in CI, this is where those races would surface. After each
     * reconnect, a position must again reach the FMU on the new socket. */
    constexpr int cycles = 5;
    for (int c = 0; c < cycles; c++)
    {
        const int target_accepts = server.acceptCount () + 1;
        server.dropClient ();
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                conn.attemptReconnect ();
                return server.acceptCount () >= target_accepts;
            },
            io_timeout));
        prove_delivery ();
    }
}

/* Outcome of a mission-upload handshake: the sequence numbers the FMU emitted an
 * item for, and the MISSION_SET_CURRENT sequence it selected after the ack. */
namespace
{
struct MissionUploadResult
{
    std::vector<uint16_t> item_seqs{};
    uint16_t set_current{ 0 };
};

/* Drive a full mission-upload handshake (count -> request -> item -> ack) and
 * return what the FMU emitted, which the pure mission_item_for tests cannot
 * reach. `expected_count` items are exchanged. */
auto
run_mission_upload (MavLoopbackServer &server, uint16_t expected_count) -> MissionUploadResult
{
    MissionUploadResult result;
    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));
    REQUIRE (mavlink_msg_mission_count_get_count (&msg) == expected_count);
    const uint8_t mission_type = mavlink_msg_mission_count_get_mission_type (&msg);

    for (uint16_t seq = 0; seq < expected_count; seq++)
    {
        server.sendMissionRequestInt (seq, mission_type);
        REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_ITEM_INT, msg, io_timeout));
        result.item_seqs.push_back (mavlink_msg_mission_item_int_get_seq (&msg));
    }

    server.sendMissionAck (mission_type);
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_SET_CURRENT, msg, io_timeout));
    result.set_current = mavlink_msg_mission_set_current_get_seq (&msg);
    return result;
}
} // namespace

TEST_CASE ("a goto mission uploads three items and sets current to sequence 0", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;

    mav_connection conn ("127.0.0.1", server.port (), test_goto_altitude_m, test_altitude_floor_m, test_altitude_cap_m);
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    /* A goto lays out seq 0/1 = waypoint, seq 2 = RTL terminator (count 3). */
    conn.commandGoto (Point (-43.5, 172.6));

    const MissionUploadResult result = run_mission_upload (server, 3);

    REQUIRE (result.item_seqs == std::vector<uint16_t>{ 0, 1, 2 });
    /* A goto resumes at mission sequence 0 (no search offset). */
    REQUIRE (result.set_current == 0);
}

TEST_CASE ("a search mission resume sets current past the setup items (todo/48)", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;

    mav_connection conn ("127.0.0.1", server.port (), test_goto_altitude_m, test_altitude_floor_m, test_altitude_cap_m);
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    /* An empty search: seq 0/1 = setup/takeoff, seq 2 = RTL terminator (count 3).
     * Its current point index is 0, which must resume at mission sequence 2 — the
     * two-item offset, not the bare point index. */
    conn.loadSearch (std::make_shared<SMMSearch> ());

    const MissionUploadResult result = run_mission_upload (server, 3);

    REQUIRE (result.item_seqs == std::vector<uint16_t>{ 0, 1, 2 });
    REQUIRE (result.set_current == search_point_mission_seq (0));
    REQUIRE (result.set_current == 2);
}

/* Test-only accessor for the friend seam in SMM: inject a held (paused) search,
 * since the only production path to current_search is an HTTP acquire. */
struct SMMTestAccess
{
    static void
    setSearch (SMM &smm, std::shared_ptr<SMMSearch> search)
    {
        smm.current_search = std::move (search);
    }
};

TEST_CASE ("SMM resumes a held search by re-loading it on continue (todo/50)", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;

    MAV mav ("127.0.0.1", server.port (), terminate_action::none, test_goto_altitude_m, test_altitude_floor_m,
             test_altitude_cap_m);
    mav.start ();
    REQUIRE (server.waitForClient (io_timeout));

    SMM smm (mav, test_altitude_cap_m, test_altitude_floor_m, 90.0);
    /* Simulate a search acquired earlier and then paused by an interrupting hold/
     * rtl: still held locally, but no longer loaded on the autopilot. */
    SMMTestAccess::setSearch (smm, std::make_shared<SMMSearch> ());

    /* `continue` re-enters searching, which calls SMM::search. Before todo/50 this
     * early-returned and never re-commanded the autopilot; it must now re-issue
     * the mission upload so the search resumes. */
    smm.search (Point (-43.5, 172.6));

    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));
    REQUIRE (mavlink_msg_mission_count_get_count (&msg) == 3);
}
