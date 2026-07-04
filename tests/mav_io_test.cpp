#include <catch2/catch_test_macros.hpp>

#include "ilogger.hpp"
#include "mav/internal.hpp"
#include "mav/mav.hpp"
#include "mav/mission-plan.hpp"
#include "smm/smm.hpp"

#include <ardupilotmega/mavlink.h>

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <random>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
/* main() (src/main.cpp) ignores SIGPIPE so a send() to a peer that has
 * closed its end reports EPIPE (which sendMavLinkMsg already handles)
 * instead of terminating the process. Catch2 supplies its own main() here,
 * so do the same at static-init time: a test that drops the server side
 * and leaves the connection "open" from the FMU's perspective for a while
 * (todo/61) can otherwise have a heartbeat send land on the dead socket and
 * kill the whole test binary. */
struct IgnoreSigpipe
{
    IgnoreSigpipe () { std::signal (SIGPIPE, SIG_IGN); }
};
const IgnoreSigpipe ignore_sigpipe_once{};
} // namespace

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

    /* Build a valid HEARTBEAT frame's raw wire bytes without sending it, so a
     * fuzz test can truncate or corrupt them before injecting via sendRaw()
     * (todo/67). */
    auto
    packHeartbeat (uint8_t type = MAV_TYPE_QUADROTOR) -> std::vector<uint8_t>
    {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack_chan (1, 1, autopilot_tx_channel, &msg, type, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0,
                                         MAV_STATE_ACTIVE);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        unsigned int len = mavlink_msg_to_send_buffer (buf, &msg);
        return std::vector<uint8_t> (buf, buf + len);
    }

    /* Send bytes directly to the FMU's connection, bypassing MAVLink framing
     * entirely — used to inject garbage/truncated/corrupted frames (todo/67).
     * Surfaces a short/failed send instead of silently swallowing it: an
     * intermittent socket issue that dropped bytes would otherwise look like
     * a parser-robustness failure and produce a misleading test result. */
    void
    sendRaw (const std::vector<uint8_t> &data)
    {
        int fd = this->client_fd.load ();
        if (fd < 0)
        {
            return;
        }
        ssize_t sent = send (fd, data.data (), data.size (), MSG_NOSIGNAL);
        if (sent < 0 || static_cast<std::size_t> (sent) != data.size ())
        {
            std::cerr << "sendRaw: short/failed send (" << sent << " of " << data.size ()
                      << " bytes): " << std::strerror (errno) << "\n";
        }
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

/* Discards everything: these tests exercise the I/O boundary, not logging, and
 * a real Logger would need a file-backed directory (todo/59). */
class NullLogger : public ILogger
{
  public:
    void
    log (LogLevel, std::string_view) override
    {
    }
};

NullLogger test_logger{};

/* Records every message logged, so a test can assert a specific warning was
 * emitted (todo/61's goto-upload-lost warning is only observable this way,
 * since nothing else about the FMU's state changes when the upload is
 * abandoned mid-handshake). Thread-safe: log() runs on mav_connection's
 * heartbeat thread. */
class CapturingLogger : public ILogger
{
  public:
    void
    log (LogLevel, std::string_view msg) override
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        this->messages.emplace_back (msg);
    }
    auto
    containsSubstring (const std::string &needle) -> bool
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        return std::any_of (this->messages.begin (), this->messages.end (),
                            [&] (const std::string &m) { return m.find (needle) != std::string::npos; });
    }

  private:
    std::mutex mtx{};
    std::vector<std::string> messages{};
};

constexpr uint16_t test_goto_altitude_m = 50;
constexpr uint16_t test_altitude_floor_m = 10;
constexpr uint16_t test_altitude_cap_m = 120;
constexpr uint32_t test_position_stream_interval_us = 200000;
constexpr uint32_t test_battery_stream_interval_us = 1000000;
constexpr uint64_t test_smm_report_interval_ms = 1000;
constexpr long test_smm_connect_timeout_s = 5;
constexpr long test_smm_transfer_timeout_s = 10;
constexpr MavParams test_mav_params{ test_goto_altitude_m, test_altitude_floor_m, test_altitude_cap_m,
                                     test_position_stream_interval_us, test_battery_stream_interval_us };
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

/* Establish a real down->up edge and wait for it: the comms-status callback
 * only fires on a change, and mav_comms_ok's optimistic default is "up" (see
 * heartbeat_loop's own comment), so a test that skipped straight to waiting
 * for `ok` could hang forever if its first heartbeat happened to be
 * processed before heartbeat_loop's first check ever observed "down". Shared
 * by every test that needs the link up before doing anything else, so this
 * race fix lives in one place. */
auto
waitForColdStartThenUp (MavLoopbackServer &server, CommsRecorder &recorder) -> bool
{
    if (!MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout))
    {
        return false;
    }
    return MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat ();
            return recorder.lastStatus () == MavCommsStatus::ok;
        },
        io_timeout);
}

} // namespace

TEST_CASE ("mav_connection reports the link down at cold start, then up once a heartbeat arrives", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
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

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
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

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
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

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    /* A goto lays out seq 0/1 = waypoint, seq 2 = RTL terminator (count 3). */
    conn.commandGoto (Point (-43.5, 172.6));

    const MissionUploadResult result = run_mission_upload (server, 3);

    REQUIRE (result.item_seqs == std::vector<uint16_t>{ 0, 1, 2 });
    /* A goto resumes at mission sequence 0 (no search offset). */
    REQUIRE (result.set_current == 0);
}

/* todo/61: unlike RTL/failsafe/low-battery/terminate, a goto's "sent" only
 * reflects the opening MISSION_COUNT — the rest of the upload is request-
 * driven and has no replay-on-recovery guarantee. A link drop before the
 * MISSION_ACK must not be a silent no-op reported as a successful goto; it
 * must surface a clear, operator-visible warning instead. */
TEST_CASE ("a link drop mid goto-upload logs a warning instead of silently succeeding (todo/61)", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture);
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    server.sendHeartbeat ();

    /* MISSION_COUNT goes out (goto_ack_pending is now armed), but the server
     * never issues the MISSION_REQUEST/MISSION_ACK that would complete the
     * upload. */
    conn.commandGoto (Point (-43.5, 172.6));
    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));

    /* Sever the link before the handshake completes. The recv thread sees EOF
     * and marks the connection down, so the heartbeat loop's next tick
     * edge-triggers the down report (and, with it, the goto-upload-lost
     * warning) without waiting out the full heartbeat timeout. */
    server.dropClient ();

    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            return capture.containsSubstring ("goto mission upload in flight")
                   && capture.containsSubstring ("the goto did not take effect");
        },
        io_timeout));
}

TEST_CASE ("a search mission resume sets current past the setup items (todo/48)", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
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
    setSearch (SMM &smm, const std::shared_ptr<SMMSearch> &search)
    {
        std::lock_guard<std::mutex> lk (smm.state_lock);
        smm.current_search = search;
        smm.current_search_points.store (search != nullptr ? search->getPointsCount () : 0);
    }
    /* Mark a search active without a held search, as if one had been requested
     * but not yet acquired (no SMM connection in these tests, so the production
     * HTTP acquire path is unavailable). */
    static void
    setSearchActive (SMM &smm, bool active)
    {
        smm.search_active.store (active);
    }
    /* Inject a sentinel non-null asset so the worker takes the "connected"
     * acquire/report path without a real SMM connection. The sentinel is never
     * dereferenced: the tests that use it override the smm_asset_* seams. */
    static void
    setAsset (SMM &smm, smm_asset asset)
    {
        std::lock_guard<std::mutex> lk (smm.state_lock);
        smm.asset = asset;
    }
};

/* SMM test double exposing the I/O seams so a test can make a blocking SMM call
 * controllable (block_fetch + releaseFetch) and observable (the call counters)
 * without a real SMM server. */
class TestSMM : public SMM
{
  public:
    using SMM::SMM;

    std::atomic<int> fetch_calls{ 0 };
    std::atomic<int> commit_calls{ 0 };
    std::atomic<int> report_calls{ 0 };
    std::atomic<bool> fetch_entered{ false };
    std::atomic<bool> block_fetch{ false };

    void
    releaseFetch ()
    {
        {
            std::lock_guard<std::mutex> lk (this->fetch_mtx);
            this->fetch_released = true;
        }
        this->fetch_cv.notify_all ();
    }

  protected:
    auto
    fetchSearch (double /*lat*/, double /*lon*/) -> smm_search override
    {
        this->fetch_calls++;
        this->fetch_entered = true;
        if (this->block_fetch.load ())
        {
            std::unique_lock<std::mutex> lk (this->fetch_mtx);
            this->fetch_cv.wait (lk, [this] { return this->fetch_released; });
        }
        return nullptr;
    }
    auto
    commitSearch (SMMSearch & /*candidate*/) -> bool override
    {
        this->commit_calls++;
        return false;
    }
    void
    reportPositionToSmm (double /*lat*/, double /*lon*/, int32_t /*alt*/, int /*heading*/) override
    {
        this->report_calls++;
    }

  private:
    std::mutex fetch_mtx{};
    std::condition_variable fetch_cv{};
    bool fetch_released{ false };
};

TEST_CASE ("SMM resumes a held search by re-loading it on continue (todo/50)", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;

    MAV mav ("127.0.0.1", server.port (), terminate_action::none, test_mav_params, test_logger);
    mav.start ();
    REQUIRE (server.waitForClient (io_timeout));

    SMM smm (mav, test_logger, test_altitude_cap_m, test_altitude_floor_m, 90.0, test_smm_report_interval_ms,
             test_smm_connect_timeout_s, test_smm_transfer_timeout_s);
    /* SMM now feeds its flight actions back through callbacks (the App routes
     * these through the event queue); wire them straight to MAV here, as the
     * event-loop handlers do once the FMU is searching. */
    smm.registerLoadSearchCB ([&mav] (const std::shared_ptr<SMMSearch> &s) { mav.loadSearch (s); });
    smm.registerRtlCB ([&mav] { mav.setMode (flight_mode_rtl); });
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

TEST_CASE ("a pending search acquisition is retried off the timer, not just on position (todo/41)", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    MAV mav ("127.0.0.1", server.port (), terminate_action::none, test_mav_params, test_logger);
    mav.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    mav.start ();
    REQUIRE (server.waitForClient (io_timeout));

    /* Wait for the cold-start comms-down edge BEFORE sending heartbeats, so the
     * heartbeats below produce a real down->up transition the status callback
     * reports (it only fires on change; if the link came up before the loop
     * observed it down, no "ok" edge would ever fire). */
    REQUIRE (
        MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout));
    /* Now drive the up edge. Once comms reports ok the autopilot type is known
     * (the heartbeat handler registers it before recording receipt), so the
     * commanded RTL below is sent as SET_MODE rather than deferred. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat ();
            return recorder.lastStatus () == MavCommsStatus::ok;
        },
        io_timeout));

    SMM smm (mav, test_logger, test_altitude_cap_m, test_altitude_floor_m, 90.0, test_smm_report_interval_ms,
             test_smm_connect_timeout_s, test_smm_transfer_timeout_s);
    smm.registerLoadSearchCB ([&mav] (const std::shared_ptr<SMMSearch> &s) { mav.loadSearch (s); });
    smm.registerRtlCB ([&mav] { mav.setMode (flight_mode_rtl); });
    /* A search is active but not yet acquired, with no SMM asset (no connect
     * call). The acquire fallback in that state is a safe RTL; the resulting
     * SET_MODE is the observable that retryPendingSearch drove an acquire attempt
     * off the reconnect-timer path, with no position report involved. */
    SMMTestAccess::setSearchActive (smm, true);

    smm.retryPendingSearch ();

    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, io_timeout));
}

TEST_CASE ("SMM public methods stay responsive while the worker is in a slow SMM call (todo/33)", "[mav_io]")
{
    reset_mav_parser ();
    /* No loopback server / start(): the SMM worker never touches MAV on the
     * search path, so a bare MAV object is enough. */
    MAV mav ("127.0.0.1", 1, terminate_action::none, test_mav_params, test_logger);

    int dummy = 0;
    TestSMM smm (mav, test_logger, test_altitude_cap_m, test_altitude_floor_m, 90.0, test_smm_report_interval_ms,
                 test_smm_connect_timeout_s, test_smm_transfer_timeout_s);
    smm.registerLoadSearchCB ([] (const std::shared_ptr<SMMSearch> &) {});
    smm.registerRtlCB ([] {});
    smm.block_fetch = true;
    /* Sentinel asset so the worker takes the connected acquire path into the
     * (now blocking) fetchSearch seam. */
    SMMTestAccess::setAsset (smm, reinterpret_cast<smm_asset> (&dummy));

    smm.search (Point (-43.5, 172.6));
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.fetch_entered.load (); }, io_timeout));

    /* With the worker stuck in fetchSearch, every event-loop-facing SMM call must
     * still return promptly — the event loop is never blocked by SMM I/O. The
     * fetch stays blocked for far longer than this bound, so a call that waited on
     * the worker would blow it. */
    constexpr auto limit = std::chrono::seconds (2);
    auto bounded = [&limit] (const std::function<void ()> &fn) -> bool
    {
        auto start = std::chrono::steady_clock::now ();
        fn ();
        return (std::chrono::steady_clock::now () - start) < limit;
    };
    REQUIRE (bounded ([&] { (void)smm.currentSearchPoints (); }));
    REQUIRE (bounded ([&] { smm.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0)); }));
    REQUIRE (bounded ([&] { smm.reachedPoint (0); }));
    REQUIRE (bounded ([&] { smm.cancelSearch (); }));

    /* Unblock so the worker leaves fetchSearch, then wait until it has drained the
     * queued report. This must happen before the object is destroyed: the worker
     * (joined in ~SMM) calls TestSMM's seams, so it must not still be parked in
     * fetchSearch when ~TestSMM tears down the seam's sync primitives. */
    smm.releaseFetch ();
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.report_calls.load () >= 1; }, io_timeout));
}

TEST_CASE ("SMM keeps reporting position even when not searching (todo/33)", "[mav_io]")
{
    reset_mav_parser ();
    MAV mav ("127.0.0.1", 1, terminate_action::none, test_mav_params, test_logger);

    int dummy = 0;
    TestSMM smm (mav, test_logger, test_altitude_cap_m, test_altitude_floor_m, 90.0, test_smm_report_interval_ms,
                 test_smm_connect_timeout_s, test_smm_transfer_timeout_s);
    smm.registerLoadSearchCB ([] (const std::shared_ptr<SMMSearch> &) {});
    smm.registerRtlCB ([] {});
    SMMTestAccess::setAsset (smm, reinterpret_cast<smm_asset> (&dummy));
    /* Not searching (a command/latch is in control). */
    SMMTestAccess::setSearchActive (smm, false);

    smm.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0));

    /* Position is still reported to SMM, and no acquire/accept is attempted while
     * not searching. */
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.report_calls.load () >= 1; }, io_timeout));
    REQUIRE (smm.commit_calls.load () == 0);
}

TEST_CASE ("SMM does not accept a search if the searching role is revoked mid-fetch (todo/33)", "[mav_io]")
{
    reset_mav_parser ();
    MAV mav ("127.0.0.1", 1, terminate_action::none, test_mav_params, test_logger);

    int dummy = 0;
    TestSMM smm (mav, test_logger, test_altitude_cap_m, test_altitude_floor_m, 90.0, test_smm_report_interval_ms,
                 test_smm_connect_timeout_s, test_smm_transfer_timeout_s);
    std::atomic<int> rtl{ 0 };
    smm.registerLoadSearchCB ([] (const std::shared_ptr<SMMSearch> &) {});
    smm.registerRtlCB ([&rtl] { rtl++; });
    smm.block_fetch = true;
    SMMTestAccess::setAsset (smm, reinterpret_cast<smm_asset> (&dummy));

    /* Begin acquiring; the worker blocks in the fetch. */
    smm.search (Point (-43.5, 172.6));
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.fetch_entered.load (); }, io_timeout));

    /* A command takes over (revoking the searching role) while the fetch is in
     * flight, then the fetch completes. */
    smm.cancelSearch ();
    smm.releaseFetch ();

    /* Enqueue a follow-up report: the single FIFO worker processes it only after
     * the search task has fully resolved, so once report_calls ticks the accept
     * decision is final. The worker must have aborted without committing a search
     * on the server and without an RTL fallback (a clean revoke, not a failure). */
    SMMTestAccess::setAsset (smm, reinterpret_cast<smm_asset> (&dummy));
    smm.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0));
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.report_calls.load () >= 1; }, io_timeout));
    REQUIRE (smm.commit_calls.load () == 0);
    REQUIRE (rtl.load () == 0);
}

/* todo/67: malformed-byte-stream robustness, the single-repo share of Tier-3
 * Path K k01. These drive the real mav_connection recv path (not a mock)
 * over the loopback socket, so hostile bytes exercise the actual MAVLink
 * parser and its resync behaviour; the full-system flood (SITL + FSS + the
 * cap-fmu binary) stays in Tier-3 Path K as the cross-check. Run under the
 * same TSan `make check` as everything else, so a parser-state race would
 * surface here too. */

TEST_CASE ("mav_connection survives a stream of random garbage bytes (todo/67)", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &) { positions.record (); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* Seeded, deterministic corpus so a failure is reproducible. */
    std::mt19937 rng (0xC0FFEE);
    std::vector<uint8_t> garbage (4096);
    for (auto &b : garbage)
    {
        b = static_cast<uint8_t> (rng ());
    }
    server.sendRaw (garbage);

    /* The connection must survive (no crash/hang): a subsequent heartbeat
     * still reports the link up and a position report still parses, proving
     * the parser resynchronised rather than getting stuck on the garbage. */
    int before = positions.count_now ();
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat ();
            server.sendPosition ();
            return recorder.lastStatus () == MavCommsStatus::ok && positions.count_now () > before;
        },
        io_timeout));
}

TEST_CASE ("mav_connection resynchronises after a truncated frame (todo/67)", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &) { positions.record (); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    REQUIRE (waitForColdStartThenUp (server, recorder));

    auto full_frame = server.packHeartbeat ();
    /* Cut the frame at every possible offset short of complete, each time
     * followed by valid traffic: the parser must eventually resynchronise
     * rather than staying stuck forever. A truncated frame's declared length
     * is still whatever the original frame said, so the byte-level parser
     * treats however many of the next frame's bytes are needed to "complete"
     * it as fake payload before its checksum fails and it resumes scanning
     * for STX — which can consume the very next frame's own STX depending on
     * the cut point. So resync is not guaranteed within exactly one
     * subsequent frame; resend inside the predicate until one gets through
     * clean, the same defensive pattern used for the initial heartbeat-up
     * race above. */
    for (std::size_t cut = 1; cut < full_frame.size (); cut++)
    {
        int before = positions.count_now ();
        std::vector<uint8_t> truncated (full_frame.begin (), full_frame.begin () + static_cast<long> (cut));
        server.sendRaw (truncated);
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPosition ();
                return positions.count_now () > before;
            },
            io_timeout));
    }
}

TEST_CASE ("mav_connection drops a corrupted-CRC frame without flapping link state (todo/67)", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &) { positions.record (); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* Corrupt the checksum (the last two bytes of an unsigned MAVLink2
     * frame). Must be dropped silently, not misinterpreted as some other
     * message. */
    auto frame = server.packHeartbeat ();
    frame[frame.size () - 1] ^= 0xFF;
    frame[frame.size () - 2] ^= 0xFF;
    server.sendRaw (frame);

    /* Drive completion via a real response instead of a blind sleep: waiting
     * for a subsequent position report to arrive both proves the corrupted
     * frame was processed (and dropped) without wedging anything, and takes
     * exactly as long as that requires rather than a fixed guess that could
     * be too short (flaky) or too long (slow) depending on machine load. */
    int before = positions.count_now ();
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendPosition ();
            return positions.count_now () > before;
        },
        io_timeout));

    /* The link must never have flapped down while the corrupted frame was
     * being dropped. */
    REQUIRE (recorder.lastStatus () == MavCommsStatus::ok);
}

TEST_CASE ("mav_connection keeps routing valid messages between garbage bursts (todo/67)", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &) { positions.record (); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    REQUIRE (waitForColdStartThenUp (server, recorder));

    std::mt19937 rng (0xBADC0DE);
    for (int i = 0; i < 5; i++)
    {
        std::vector<uint8_t> garbage (256);
        for (auto &b : garbage)
        {
            b = static_cast<uint8_t> (rng ());
        }
        server.sendRaw (garbage);

        int before = positions.count_now ();
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendHeartbeat ();
                server.sendPosition ();
                return recorder.lastStatus () == MavCommsStatus::ok && positions.count_now () > before;
            },
            io_timeout));
    }
}
