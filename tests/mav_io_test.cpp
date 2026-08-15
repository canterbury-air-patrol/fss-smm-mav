#include "catch2-compat.hpp"

#include "ilogger.hpp"
#include "mav/internal.hpp"
#include "mav/mav-comms.hpp"
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
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <poll.h>
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
 * so do the same at static-init time: a test that drops the server side and
 * leaves the connection "open" from the FMU's perspective for a while can
 * otherwise have a heartbeat send land on the dead socket and kill the
 * whole test binary. */
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
    /* `listen_recv_buf_bytes`, if non-zero, shrinks the receive buffer on the
     * *listening* socket before bind()/listen(): every accepted connection
     * inherits it, so the TCP window advertised in that connection's very
     * first SYN-ACK is already small. Doing this on the per-connection fd
     * after accept() is too late — the initial window was already advertised
     * using whatever the default was at handshake time, and a receiver that
     * never reads never sends a window update to shrink it further. */
    explicit MavLoopbackServer (int listen_recv_buf_bytes = 0)
    {
        this->listen_fd = socket (AF_INET, SOCK_STREAM, 0);
        REQUIRE (this->listen_fd >= 0);
        int one = 1;
        setsockopt (this->listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof (one));
        if (listen_recv_buf_bytes > 0)
        {
            setsockopt (this->listen_fd, SOL_SOCKET, SO_RCVBUF, &listen_recv_buf_bytes, sizeof (listen_recv_buf_bytes));
        }

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

    /* Send a HEARTBEAT as if from `sysid` (the autopilot, sysid 1, by default),
     * so the FMU records a fresh last_heartbeat_ts and the heartbeat loop
     * reports the link up. A non-default sysid stands in for another vehicle or
     * GCS sharing the link. */
    void
    sendHeartbeat (uint8_t type = MAV_TYPE_QUADROTOR, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack_chan (sysid, 1, autopilot_tx_channel, &msg, type, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0,
                                         MAV_STATE_ACTIVE);
        sendMsg (msg);
    }

    /* Send a GLOBAL_POSITION_INT so the FMU's recv path fires its position
     * callback — a deterministic observable that the link carries data, used to
     * prove a reconnect re-established a working recv path. */
    void
    sendPosition ()
    {
        sendPosition (/*lat*/ -435000000, /*lon*/ 1726000000);
    }

    /* Same, with an explicit lat/lon and sysid: lets a test send the exact same
     * coordinates repeatedly, e.g. to simulate an EKF position estimate that
     * has frozen rather than genuinely updating, or attribute the report to a
     * system other than the configured autopilot. */
    void
    sendPosition (int32_t lat, int32_t lon, uint8_t sysid = 1)
    {
        sendPosition (lat, lon, /*alt mm*/ 100000, /*rel alt mm*/ 100000, sysid);
    }

    /* Same, with alt (MSL) and relative_alt (AGL) set independently: every
     * other overload above packs both to the same value, so no test using them
     * can distinguish which field a consumer actually read. */
    void
    sendPosition (int32_t lat, int32_t lon, int32_t alt_mm, int32_t relative_alt_mm, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_global_position_int_pack_chan (sysid, 1, autopilot_tx_channel, &msg, 0, lat, lon, alt_mm,
                                                   relative_alt_mm, 0, 0, 0, /*hdg*/ 0);
        sendMsg (msg);
    }

    /* Same, with the autopilot's uptime (time_boot_ms) set explicitly: every
     * overload above packs 0, which the restart detector treats as "nothing
     * observed yet", so no test using them can drive it. */
    void
    sendPositionAt (uint32_t time_boot_ms, int32_t lat = -435000000, int32_t lon = 1726000000, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_global_position_int_pack_chan (sysid, 1, autopilot_tx_channel, &msg, time_boot_ms, lat, lon,
                                                   /*alt mm*/ 100000, /*rel alt mm*/ 100000, 0, 0, 0, /*hdg*/ 0);
        sendMsg (msg);
    }

    /* Send a SYSTEM_TIME carrying the autopilot's uptime — the second source the
     * restart detector reads, so a reboot is still caught when the position
     * stream is not flowing. A non-default sysid stands in for another vehicle
     * or GCS sharing the link. */
    void
    sendSystemTime (uint32_t time_boot_ms, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_system_time_pack_chan (sysid, 1, autopilot_tx_channel, &msg, /*time_unix_usec*/ 0, time_boot_ms);
        sendMsg (msg);
    }

    /* Send a GPS_RAW_INT with the given fix_type — e.g. GPS_FIX_TYPE_NO_FIX —
     * as ArduPilot would report GPS health directly (distinct from
     * GLOBAL_POSITION_INT, which carries the EKF's position estimate and has
     * no fix-validity field of its own). A non-default sysid stands in for
     * another vehicle or GCS sharing the link. */
    void
    sendGpsRawInt (uint8_t fix_type, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_gps_raw_int_pack_chan (sysid, 1, autopilot_tx_channel, &msg, /*time_usec*/ 0, fix_type, /*lat*/ 0,
                                           /*lon*/ 0, /*alt*/ 0, /*eph*/ UINT16_MAX, /*epv*/ UINT16_MAX,
                                           /*vel*/ UINT16_MAX, /*cog*/ UINT16_MAX, /*satellites_visible*/ 0,
                                           /*alt_ellipsoid*/ 0, /*h_acc*/ 0, /*v_acc*/ 0, /*vel_acc*/ 0,
                                           /*hdg_acc*/ 0, /*yaw*/ 0);
        sendMsg (msg);
    }

    /* Send a BATTERY_STATUS as ArduPilot would, with the pack voltage in cell 0
     * (see battery-voltage.hpp). A non-default sysid stands in for another
     * vehicle or GCS sharing the link. */
    void
    sendBatteryStatus (int8_t remaining, int32_t consumed, uint16_t cell0_mv, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        uint16_t voltages[10] = { cell0_mv };
        uint16_t voltages_ext[4] = {};
        mavlink_msg_battery_status_pack_chan (sysid, 1, autopilot_tx_channel, &msg, /*id*/ 0, /*battery_function*/ 0,
                                              /*type*/ 0, /*temperature*/ 0, voltages, /*current_battery*/ 0, consumed,
                                              /*energy_consumed*/ -1, remaining, /*time_remaining*/ 0,
                                              /*charge_state*/ 0, voltages_ext, /*mode*/ 0, /*fault_bitmask*/ 0);
        sendMsg (msg);
    }

    /* Send a PARAM_VALUE reply for `name`, as ArduPilot would reply to a
     * PARAM_REQUEST_READ — always a float on the wire regardless of the
     * declared param_type. A non-default sysid stands in for another vehicle
     * or GCS sharing the link. */
    void
    sendParamValue (const char *name, float value, uint8_t sysid = 1)
    {
        /* param_id is a fixed 16-byte field, not a NUL-terminated C string;
         * mavlink_msg_param_value_pack_chan always reads 16 bytes from the
         * pointer it is given, so a `name` shorter than that must first be
         * copied into a buffer that size, zero-padded, or the call reads out
         * of bounds (pack_fixed_width_field(), mav-comms.hpp — shared with
         * mav_connection::checkFailsafeConfig()'s identical need). */
        char param_id_buf[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
        pack_fixed_width_field (param_id_buf, sizeof (param_id_buf), name);
        mavlink_message_t msg;
        mavlink_msg_param_value_pack_chan (sysid, 1, autopilot_tx_channel, &msg, param_id_buf, value,
                                           MAV_PARAM_TYPE_REAL32, /*param_count*/ 1, /*param_index*/ 0);
        sendMsg (msg);
    }

    /* Send a MISSION_ITEM_REACHED for `seq`. A non-default sysid stands in
     * for another vehicle or GCS sharing the link. */
    void
    sendMissionItemReached (uint16_t seq, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_mission_item_reached_pack_chan (sysid, 1, autopilot_tx_channel, &msg, seq);
        sendMsg (msg);
    }

    /* Send a MISSION_REQUEST_INT for `seq` as the autopilot would during a
     * mission upload (addressed to the FMU at SYS_ID/COMP_ID). A non-default
     * sysid stands in for another vehicle or GCS sharing the link. */
    void
    sendMissionRequestInt (uint16_t seq, uint8_t mission_type, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_mission_request_int_pack_chan (sysid, 1, autopilot_tx_channel, &msg, fmu_sys_id, fmu_comp_id, seq,
                                                   mission_type);
        sendMsg (msg);
    }

    /* Acknowledge a completed mission upload as accepted. A non-default sysid
     * stands in for another vehicle or GCS sharing the link. */
    void
    sendMissionAck (uint8_t mission_type, uint8_t sysid = 1)
    {
        mavlink_message_t msg;
        mavlink_msg_mission_ack_pack_chan (sysid, 1, autopilot_tx_channel, &msg, fmu_sys_id, fmu_comp_id,
                                           MAV_MISSION_ACCEPTED, mission_type, 0);
        sendMsg (msg);
    }

    /* Build a valid HEARTBEAT frame's raw wire bytes without sending it, so a
     * fuzz test can truncate or corrupt them before injecting via sendRaw().
     */
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
     * entirely — used to inject garbage/truncated/corrupted frames. Surfaces
     * a short/failed send instead of silently swallowing it: an intermittent
     * socket issue that dropped bytes would otherwise look like a
     * parser-robustness failure and produce a misleading test result. */
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
     * arrives (skipping heartbeats, mode sets, etc.) or the timeout elapses.
     * The FMU can emit two messages back-to-back (e.g. MISSION_SET_CURRENT
     * immediately followed by SET_MODE) that land in the same recv() chunk;
     * any bytes after the match are stashed in `leftover` for the next call
     * instead of being silently discarded, so a subsequent recvMessage() for
     * that trailing message cannot flake depending on kernel buffering. */
    auto
    recvMessage (uint32_t want, mavlink_message_t &out, std::chrono::milliseconds timeout) -> bool
    {
        mavlink_status_t status;
        while (!this->leftover.empty ())
        {
            uint8_t c = this->leftover.front ();
            this->leftover.erase (this->leftover.begin ());
            if (mavlink_parse_char (autopilot_rx_channel, c, &out, &status) && out.msgid == want)
            {
                return true;
            }
        }
        const auto deadline = std::chrono::steady_clock::now () + timeout;
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
                    this->leftover.insert (this->leftover.end (), buf + i + 1, buf + n);
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
    /* Bytes already read from the socket but not yet consumed by a
     * recvMessage() call, since a match returns as soon as it is found and
     * must not drop whatever followed it in the same recv() chunk. Only
     * ever touched from the (single) test thread, so it needs no lock. */
    std::vector<uint8_t> leftover{};
};

/* A bare TCP listener that intentionally never calls accept(), so its accept
 * queue can be deliberately filled and then a further connect() attempt
 * reliably times out (its SYN is dropped by the kernel, not merely delayed)
 * — a deterministic, root-free, firewall-free local stand-in for a
 * black-holed remote endpoint. Distinct from MavLoopbackServer, which always
 * accepts.
 *
 * The accept queue is filled by self-calibration rather than assuming
 * listen(fd, 1) leaves room for exactly one connection: each filler
 * connection is itself bounded by a short non-blocking connect+poll, so a
 * filler that lands on an already-full queue cannot hang the test; the loop
 * stops as soon as one filler fails to complete quickly, which is the signal
 * that the queue is now genuinely full regardless of the exact backlog
 * rounding a given kernel applies. */
class BlackholeListener
{
  public:
    BlackholeListener ()
    {
        this->listen_fd = socket (AF_INET, SOCK_STREAM, 0);
        REQUIRE (this->listen_fd >= 0);
        int one = 1;
        setsockopt (this->listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof (one));

        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl (INADDR_LOOPBACK);
        addr.sin_port = 0;
        REQUIRE (bind (this->listen_fd, reinterpret_cast<struct sockaddr *> (&addr), sizeof (addr)) == 0);

        socklen_t len = sizeof (addr);
        REQUIRE (getsockname (this->listen_fd, reinterpret_cast<struct sockaddr *> (&addr), &len) == 0);
        this->listen_port = ntohs (addr.sin_port);

        REQUIRE (listen (this->listen_fd, 1) == 0);

        bool filled = false;
        for (int i = 0; i < 8 && !filled; i++)
        {
            int filler = socket (AF_INET, SOCK_STREAM, 0);
            REQUIRE (filler >= 0);
            int flags = fcntl (filler, F_GETFL, 0);
            fcntl (filler, F_SETFL, flags | O_NONBLOCK);

            struct sockaddr_in target = {};
            target.sin_family = AF_INET;
            target.sin_addr.s_addr = htonl (INADDR_LOOPBACK);
            target.sin_port = htons (this->listen_port);
            int rc = connect (filler, reinterpret_cast<struct sockaddr *> (&target), sizeof (target));
            bool ok = false;
            if (rc == 0)
            {
                ok = true;
            }
            else if (errno == EINPROGRESS)
            {
                struct pollfd pfd = { .fd = filler, .events = POLLOUT, .revents = 0 };
                if (poll (&pfd, 1, 200) == 1)
                {
                    int so_error = 0;
                    socklen_t so_error_len = sizeof (so_error);
                    ok = getsockopt (filler, SOL_SOCKET, SO_ERROR, &so_error, &so_error_len) == 0 && so_error == 0;
                }
            }
            if (ok)
            {
                /* Keep it open and never accepted: it stays in the queue. */
                this->fillers.push_back (filler);
            }
            else
            {
                close (filler);
                filled = true;
            }
        }
        REQUIRE (filled);
    }

    ~BlackholeListener ()
    {
        for (int fd : this->fillers)
        {
            close (fd);
        }
        if (this->listen_fd >= 0)
        {
            close (this->listen_fd);
        }
    }
    BlackholeListener (const BlackholeListener &) = delete;
    BlackholeListener (BlackholeListener &&) = delete;
    auto operator= (const BlackholeListener &) -> BlackholeListener & = delete;
    auto operator= (BlackholeListener &&) -> BlackholeListener & = delete;

    auto
    port () const -> uint16_t
    {
        return this->listen_port;
    }

  private:
    int listen_fd{ -1 };
    uint16_t listen_port{ 0 };
    std::vector<int> fillers{};
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
 * a real Logger would need a file-backed directory. */
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
 * emitted (the goto-upload-lost warning is only observable this way, since
 * nothing else about the FMU's state changes when the upload is abandoned
 * mid-handshake). Thread-safe: log() runs on mav_connection's heartbeat
 * thread. */
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
constexpr uint32_t test_mav_connect_timeout_ms = 2000;
constexpr uint32_t test_mav_send_timeout_ms = 2000;
constexpr MavParams test_mav_params{ test_goto_altitude_m,
                                     test_altitude_floor_m,
                                     test_altitude_cap_m,
                                     test_position_stream_interval_us,
                                     test_battery_stream_interval_us,
                                     test_mav_connect_timeout_ms,
                                     test_mav_send_timeout_ms };
constexpr auto io_timeout = std::chrono::seconds (8);
/* Bound for asserting a message is *not* sent: unlike a positive wait, there is no early
 * exit (the loop must run out the clock), so this stays far shorter than io_timeout. Any
 * erroneous MISSION_SET_CURRENT/SET_MODE(AUTO) an ack-correlation regression would emit
 * happens synchronously while processing the incoming MISSION_ACK, well within this margin.
 */
constexpr auto no_message_timeout = std::chrono::milliseconds (300);

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
    record (const PositionData &pd)
    {
        {
            const std::lock_guard<std::mutex> lk (this->mtx);
            this->last = pd;
        }
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
    /* Latest PositionData received, for tests that need to inspect the exact
     * values that reached the callback, not just that one arrived. */
    auto
    lastPosition () -> PositionData
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        return this->last;
    }

  private:
    std::atomic<int> count{ 0 };
    std::mutex mtx{};
    PositionData last{};
};

/* Thread-safe counter of battery callbacks (proves a foreign sysid's
 * BATTERY_STATUS never reaches it). */
class BatteryRecorder
{
  public:
    void
    record (const BatteryData &bd)
    {
        {
            const std::lock_guard<std::mutex> lk (this->mtx);
            this->last = bd;
        }
        this->count.fetch_add (1);
    }
    auto
    count_now () const -> int
    {
        return this->count.load ();
    }
    auto
    lastBattery () -> BatteryData
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        return this->last;
    }

  private:
    std::atomic<int> count{ 0 };
    std::mutex mtx{};
    BatteryData last{};
};

/* Thread-safe counter of reached-point callbacks (proves a foreign
 * sysid's MISSION_ITEM_REACHED never reaches it). */
class ReachedRecorder
{
  public:
    void
    record (int point)
    {
        {
            const std::lock_guard<std::mutex> lk (this->mtx);
            this->last = point;
        }
        this->count.fetch_add (1);
    }
    auto
    count_now () const -> int
    {
        return this->count.load ();
    }
    auto
    lastPoint () -> int
    {
        const std::lock_guard<std::mutex> lk (this->mtx);
        return this->last;
    }

  private:
    std::atomic<int> count{ 0 };
    std::mutex mtx{};
    int last{ -1 };
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

/* Every param_id the failsafe-config check has requested and not yet been read
 * off the wire. A single check now issues several PARAM_REQUEST_READs, so a
 * test that pulled just the first message would be asserting on the order this
 * list happens to be built in rather than on its contents. Blocks for
 * io_timeout on the first, then drains whatever else has already arrived. */
auto
collectParamRequests (MavLoopbackServer &server) -> std::vector<std::string>
{
    std::vector<std::string> requested;
    mavlink_message_t msg;
    auto timeout = std::chrono::duration_cast<std::chrono::milliseconds> (io_timeout);
    while (server.recvMessage (MAVLINK_MSG_ID_PARAM_REQUEST_READ, msg, timeout))
    {
        char buf[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN + 1] = { 0 };
        mavlink_msg_param_request_read_get_param_id (&msg, buf);
        requested.emplace_back (buf);
        timeout = no_message_timeout;
    }
    return requested;
}

auto
requested (const std::vector<std::string> &params, const std::string &name) -> bool
{
    return std::find (params.begin (), params.end (), name) != params.end ();
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

TEST_CASE ("start() does not block beyond the configured connect deadline against a black-holed endpoint", "[mav_io]")
{
    reset_mav_parser ();
    BlackholeListener blackhole;
    CapturingLogger capture;

    MavParams short_connect_params = test_mav_params;
    short_connect_params.mav_connect_timeout_ms = 300;

    mav_connection conn ("127.0.0.1", blackhole.port (), short_connect_params, capture);

    auto t0 = std::chrono::steady_clock::now ();
    conn.start ();
    auto elapsed = std::chrono::steady_clock::now () - t0;

    /* Generous margin over the 300ms deadline covers scheduling noise and TSan
     * instrumentation overhead without masking a regression to an unbounded
     * (multi-second-to-minutes) OS-level connect timeout. */
    REQUIRE (elapsed < std::chrono::seconds (2));
    REQUIRE (capture.containsSubstring ("timed out"));
}

TEST_CASE ("a send to a peer that stops reading is bounded by the configured send timeout", "[mav_io]")
{
    reset_mav_parser ();
    /* Shrink the listening socket's receive buffer before any connection
     * exists, so the accepted connection's very first SYN-ACK already
     * advertises a small window — shrinking it afterwards is too late (see
     * MavLoopbackServer's constructor comment). The server never reads once
     * connected, so that small window is never replenished. */
    MavLoopbackServer server (1);
    CapturingLogger capture;

    MavParams send_bound_params = test_mav_params;
    send_bound_params.mav_send_timeout_ms = 500;

    mav_connection conn ("127.0.0.1", server.port (), send_bound_params, capture);
    conn.start ();

    REQUIRE (server.waitForClient (io_timeout));

    /* Empirically, ~21500 unread 46-byte MAVLink frames (~1MB) are enough to
     * exhaust the local send buffer plus the near-zero receive window on this
     * kernel before a blocking send() has nowhere left to put more data;
     * comfortably overshoot that so the bound is exercised regardless of
     * exact OS buffer sizing. Once one call blocks and times out,
     * TCP_USER_TIMEOUT aborts the connection, so the remaining calls in the
     * loop fail fast rather than each re-blocking for 500ms. */
    char callsign[9] = "TEST0000";
    auto t0 = std::chrono::steady_clock::now ();
    for (int i = 0; i < 60000; i++)
    {
        conn.sendADSB (0x123456, -35.0, 149.0, 100.0, 0, 0, 0, 0, callsign, 0, 0, 0, 0);
    }
    auto elapsed = std::chrono::steady_clock::now () - t0;

    /* The definitive proof the bound engaged: sendMavLinkMsgLocked logs on
     * any failed send, which only happens once SO_SNDTIMEO/TCP_USER_TIMEOUT
     * make a blocking send() fail instead of hanging. */
    REQUIRE (capture.containsSubstring ("MAV send() failed"));
    /* Backstop on wall-clock time too: only the one call that finds the
     * buffer genuinely full should block for up to ~500ms (observed up to
     * ~2x that in practice); a generous margin absorbs scheduling/TSan noise
     * without masking a regression to unbounded blocking. */
    REQUIRE (elapsed < std::chrono::seconds (10));
}

TEST_CASE ("a healthy link survives the reconnector after the cold-start comms-down report", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();

    /* The cold-start report is "link down" by design (no heartbeat has arrived
     * on a socket that is milliseconds old), and that report used to also flag
     * the connection broken — so the reconnector's next pass tore down a link
     * that was about to work, dragging a stream re-request, a failsafe-param
     * re-check and any in-flight mission upload with it, and leaving fd == -1
     * for the redial. Establish the link, then keep it healthy. */
    REQUIRE (waitForColdStartThenUp (server, recorder));
    const int accepts_before = server.acceptCount ();
    REQUIRE (accepts_before == 1);

    /* Drive the reconnector as App's thread would, well past the point where
     * the cold-start pass and several further heartbeat_loop() passes have
     * happened, while the autopilot keeps heartbeating. The interval stays far
     * inside heartbeat_loop()'s 5s timeout, so nothing here is a genuine
     * staleness retire. */
    for (int i = 0; i < 20; i++)
    {
        server.sendHeartbeat ();
        conn.attemptReconnect ();
        std::this_thread::sleep_for (std::chrono::milliseconds (75));
    }

    /* Not one redial: the socket accepted at start() is still the live one. */
    REQUIRE (server.acceptCount () == accepts_before);
    /* And the link was never reported down again — a teardown/redial cycle
     * blanks the fd for the join+connect, which the 1Hz heartbeat pass can land
     * in and turn into a comms failure (the FMU's own comms-loss failsafe) on a
     * link that was never broken. */
    REQUIRE (recorder.lastStatus () == MavCommsStatus::ok);
}

TEST_CASE ("a socket that connects but never heartbeats is eventually retired", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    /* The server accepts and then says nothing at all — an endpoint that
     * carries TCP but no autopilot traffic (mavproxy up, autopilot serial
     * dead). Deferring the retire decision until a socket has been open for a
     * heartbeat timeout must not turn into never retiring such a socket, so
     * this is the counterpart of the healthy-link case above. */
    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();

    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (
        MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout));
    const int accepts_before = server.acceptCount ();

    /* heartbeat_loop()'s timeout is a fixed 5s, so this wait has to outlast it
     * by a clear margin — hence a longer bound than io_timeout. It is only a
     * bound: the reconnect is observed as soon as it happens. */
    constexpr auto retire_timeout = std::chrono::seconds (20);
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            conn.attemptReconnect ();
            return server.acceptCount () > accepts_before;
        },
        retire_timeout));
}

TEST_CASE ("a stale heartbeat on an otherwise-open socket is retired and reconnected", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();

    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* Run the reconnector against the *healthy* link first, and only snapshot
     * the accept count afterwards. Without this the test could not tell a
     * reconnect caused by the heartbeats stopping from one caused by a `broken`
     * flag latched back at cold start — which is what it was actually observing,
     * so it passed even with half-open detection removed entirely. */
    for (int i = 0; i < 10; i++)
    {
        server.sendHeartbeat ();
        conn.attemptReconnect ();
        std::this_thread::sleep_for (std::chrono::milliseconds (50));
    }
    const int accepts_before = server.acceptCount ();
    REQUIRE (accepts_before == 1);

    /* Stop heartbeating without dropping the socket — the fd stays open at the
     * TCP layer throughout, the "half-open" scenario this test exists for.
     * Wait for the resulting heartbeat-staleness down edge. */
    REQUIRE (
        MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout));

    /* heartbeat_loop() must have flagged the connection broken without tearing
     * it down itself (docs/threading.md: only attemptReconnect() retires a
     * connection) — drive the reconnector as App's would. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            conn.attemptReconnect ();
            return server.acceptCount () > accepts_before;
        },
        io_timeout));

    /* The new connection is a genuinely working recv path, not just a fresh
     * socket. */
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
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
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
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
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

void
expect_auto_mode (MavLoopbackServer &server)
{
    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, io_timeout));
    REQUIRE (mavlink_msg_set_mode_get_target_system (&msg) == 1);
    REQUIRE (mavlink_msg_set_mode_get_custom_mode (&msg) == COPTER_MODE_AUTO);
}
} // namespace

TEST_CASE ("a goto mission uploads three items and sets current to sequence 0", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    /* Establish the link as genuinely up first: otherwise the cold-start "no
     * heartbeat yet" detection (heartbeat_loop) can invalidate the goto's own
     * MISSION_ACK correlation (goto_ack_pending) before this upload
     * completes, exactly as a real comms-loss failsafe would — which is not
     * what this test is exercising. */
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* A goto lays out seq 0/1 = waypoint, seq 2 = RTL terminator (count 3). */
    conn.commandGoto (Point (-43.5, 172.6));

    const MissionUploadResult result = run_mission_upload (server, 3);

    REQUIRE (result.item_seqs == std::vector<uint16_t>{ 0, 1, 2 });
    /* A goto resumes at mission sequence 0 (no search offset). */
    REQUIRE (result.set_current == 0);
}

TEST_CASE ("a goto mission selects current before engaging AUTO after upload", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    conn.commandGoto (Point (-43.5, 172.6));

    const MissionUploadResult result = run_mission_upload (server, 3);

    REQUIRE (result.set_current == 0);
    expect_auto_mode (server);
}

/* Unlike RTL/failsafe/low-battery/terminate, a goto's "sent" only reflects
 * the opening MISSION_COUNT — the rest of the upload is request- driven
 * and has no replay-on-recovery guarantee. A link drop before the
 * MISSION_ACK must not be a silent no-op reported as a successful goto; it
 * must surface a clear, operator-visible warning instead. */
TEST_CASE ("a link drop mid goto-upload logs a warning instead of silently succeeding", "[mav_io]")
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

TEST_CASE ("a search mission resume sets current past the setup items", "[mav_io]")
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

TEST_CASE ("a search mission selects current before engaging AUTO after upload", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    conn.loadSearch (std::make_shared<SMMSearch> ());

    const MissionUploadResult result = run_mission_upload (server, 3);

    REQUIRE (result.set_current == search_point_mission_seq (0));
    expect_auto_mode (server);
}

/* A late MISSION_ACK for a goto upload must not be able to select a mission
 * item or command AUTO once a newer safety-critical mode (here, RTL —
 * standing in for a real low-battery/comms-loss/operator RTL, all of which
 * reach the autopilot the same way via commandRTL()) has taken control while
 * the upload was still open. */
TEST_CASE ("a newer RTL invalidates an in-flight goto upload; its late ACCEPTED ack is ignored", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    conn.commandGoto (Point (-43.5, 172.6));

    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));
    const uint8_t mission_type = mavlink_msg_mission_count_get_mission_type (&msg);
    const uint16_t count = mavlink_msg_mission_count_get_count (&msg);

    /* Drive the item exchange (as run_mission_upload would) but withhold the
     * MISSION_ACK: the upload is still open when the newer safety state takes
     * control. */
    for (uint16_t seq = 0; seq < count; seq++)
    {
        server.sendMissionRequestInt (seq, mission_type);
        REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_ITEM_INT, msg, io_timeout));
    }

    /* The newer safety state takes control mid-upload: the same call the
     * state machine's RTL action makes. */
    conn.commandRTL ();
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, io_timeout));
    REQUIRE (mavlink_msg_set_mode_get_custom_mode (&msg) == COPTER_MODE_RTL);

    /* The autopilot finishes and accepts the now-superseded upload. */
    server.sendMissionAck (mission_type);

    /* Neither MISSION_SET_CURRENT nor a further SET_MODE (AUTO) may follow. */
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_MISSION_SET_CURRENT, msg, no_message_timeout));
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, no_message_timeout));
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("invalidated an in-flight"); },
                                         io_timeout));
}

/* The equivalent interrupted-search-upload case. loadSearch() itself issues
 * its own RTL before the upload (existing "enter RTL while loading"
 * behaviour); the second RTL here stands in for a newer safety state — e.g.
 * low battery — arriving before the search's own MISSION_ACK. */
TEST_CASE ("a newer RTL invalidates an in-flight search upload; its late ACCEPTED ack is ignored", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    conn.loadSearch (std::make_shared<SMMSearch> ());

    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));
    const uint8_t mission_type = mavlink_msg_mission_count_get_mission_type (&msg);
    const uint16_t count = mavlink_msg_mission_count_get_count (&msg);

    for (uint16_t seq = 0; seq < count; seq++)
    {
        server.sendMissionRequestInt (seq, mission_type);
        REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_ITEM_INT, msg, io_timeout));
    }

    conn.commandRTL ();
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, io_timeout));
    REQUIRE (mavlink_msg_set_mode_get_custom_mode (&msg) == COPTER_MODE_RTL);

    server.sendMissionAck (mission_type);

    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_MISSION_SET_CURRENT, msg, no_message_timeout));
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, no_message_timeout));
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("invalidated an in-flight"); },
                                         io_timeout));
}

/* Re-driving fmu_state_searching before the first search upload's MISSION_ACK
 * arrives used to segfault the process in flight. loadSearch()'s preamble RTL
 * invalidates whatever upload is still open, which drops a loading search's
 * `search` reference with it — and the count was then computed from that
 * now-null pointer. search_loaded stays false for the whole upload, so the
 * public overload re-enters on exactly this state and no timing window is
 * needed to reproduce it.
 *
 * The second load must supersede the first and complete, and must say so as a
 * supersede: the RTL here is the upload's own scaffolding, not an operator or
 * failsafe taking control. */
TEST_CASE ("a second search load while the first upload is unacked supersedes it", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    auto search = std::make_shared<SMMSearch> ();
    conn.loadSearch (search);

    /* Take the opening MISSION_COUNT but withhold the MISSION_ACK: the first
     * upload is still open when the second load begins. */
    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));

    conn.loadSearch (search);

    /* The superseding upload runs to completion, resuming at the same mission
     * sequence a first-time load would. */
    const MissionUploadResult result = run_mission_upload (server, 3);
    REQUIRE (result.item_seqs == std::vector<uint16_t>{ 0, 1, 2 });
    REQUIRE (result.set_current == search_point_mission_seq (0));
    expect_auto_mode (server);

    REQUIRE (MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("superseded an in-flight"); },
                                         io_timeout));
    REQUIRE_FALSE (capture.containsSubstring ("invalidated an in-flight"));
}

/* A duplicate/retransmitted accepted MISSION_ACK arriving after a goto
 * upload already completed must not repeat the MISSION_SET_CURRENT/AUTO
 * transition. goto_active alone used to be a long-lived signal — it never
 * cleared after a first ack — so a second ack would re-fire it. */
TEST_CASE ("a duplicate accepted goto ACK after completion does not re-enter AUTO", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    conn.commandGoto (Point (-43.5, 172.6));

    const MissionUploadResult result = run_mission_upload (server, 3);
    REQUIRE (result.set_current == 0);
    expect_auto_mode (server);

    /* A duplicate delivery of the same accepted ack, after the upload has
     * already completed and AUTO has already been commanded once. */
    mavlink_message_t msg;
    server.sendMissionAck (MAV_MISSION_TYPE_MISSION);

    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_MISSION_SET_CURRENT, msg, no_message_timeout));
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, no_message_timeout));
    REQUIRE (MavLoopbackServer::waitFor (
        [&] () { return capture.containsSubstring ("no goto/search upload was genuinely pending"); }, io_timeout));
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

/* Parse_waypoints() is the pure decision SMMSearch's constructor uses to
 * decide whether a fetched search is loadable -- tested directly here against
 * plain smm_waypoint_s structs, without a live smm_search handle (there is no
 * lightweight way to fabricate one; smm_search is opaque and only populated
 * via the real HTTP-backed library). */
TEST_CASE ("parse_waypoints reports all_valid only when every waypoint is a valid coordinate", "[smm]")
{
    smm_waypoint_s a{ -43.5, 172.6 };
    smm_waypoint_s b{ -41.0, 174.0 };
    smm_waypoint_s bad{ std::numeric_limits<double>::quiet_NaN (), 0.0 };

    SECTION ("every waypoint valid")
    {
        smm_waypoint wps[] = { &a, &b };
        ParsedWaypoints result = parse_waypoints (wps, 2);
        REQUIRE (result.all_valid);
        REQUIRE (result.points.size () == 2);
        REQUIRE (result.points[0].getLatitude () == Catch::Approx (-43.5));
        REQUIRE (result.points[1].getLatitude () == Catch::Approx (-41.0));
    }

    SECTION ("one invalid waypoint invalidates the whole batch")
    {
        smm_waypoint wps[] = { &a, &bad, &b };
        ParsedWaypoints result = parse_waypoints (wps, 3);
        REQUIRE_FALSE (result.all_valid);
        /* Every point is still decoded -- the caller (SMMSearch's constructor)
         * discards the whole candidate rather than the single bad point, so
         * there is no partial mission to accidentally load. */
        REQUIRE (result.points.size () == 3);
    }

    SECTION ("zero waypoints is vacuously all-valid")
    {
        ParsedWaypoints result = parse_waypoints (nullptr, 0);
        REQUIRE (result.all_valid);
        REQUIRE (result.points.empty ());
    }
}

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
    /* Test-controlled stand-in for smm_asset_last_command(): no live SMM
     * server to make the C library return a real operator command from, so
     * this seam lets a test set what checkOperatorCommand() sees on the next
     * reportPosition(). */
    std::atomic<smm_asset_command> next_operator_command{ SMM_COMMAND_UNKNOWN };

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
    auto
    lastOperatorCommand () -> smm_asset_command override
    {
        return this->next_operator_command.load ();
    }

  private:
    std::mutex fetch_mtx{};
    std::condition_variable fetch_cv{};
    bool fetch_released{ false };
};

TEST_CASE ("SMM resumes a held search by re-loading it on continue", "[mav_io]")
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

    /* `continue` re-enters searching, which calls SMM::search. This once
     * early-returned and never re-commanded the autopilot; it must now re-issue
     * the mission upload so the search resumes. */
    smm.search (Point (-43.5, 172.6));

    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));
    REQUIRE (mavlink_msg_mission_count_get_count (&msg) == 3);
}

TEST_CASE ("a pending search acquisition is retried off the timer, not just on position", "[mav_io]")
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

TEST_CASE ("SMM public methods stay responsive while the worker is in a slow SMM call", "[mav_io]")
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

TEST_CASE ("SMM keeps reporting position even when not searching", "[mav_io]")
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

TEST_CASE ("SMM does not accept a search if the searching role is revoked mid-fetch", "[mav_io]")
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

TEST_CASE ("SMM drops a held search and reattempts acquisition on an operator abandon-search command, once per "
           "distinct command",
           "[mav_io]")
{
    reset_mav_parser ();
    MAV mav ("127.0.0.1", 1, terminate_action::none, test_mav_params, test_logger);

    int dummy = 0;
    /* A short report interval (rather than the usual test_smm_report_interval_ms)
     * so two real position reports land far enough apart to each clear the
     * rate-limit gate checkOperatorCommand() sits behind, without a
     * second-scale sleep in this test. */
    TestSMM smm (mav, test_logger, test_altitude_cap_m, test_altitude_floor_m, 90.0, 10, test_smm_connect_timeout_s,
                 test_smm_transfer_timeout_s);
    smm.registerLoadSearchCB ([] (const std::shared_ptr<SMMSearch> &) {});
    smm.registerRtlCB ([] {});
    std::mutex cmds_mtx;
    std::vector<SMMCommand> operator_cmds;
    smm.registerOperatorCommandCB (
        [&] (SMMCommand cmd)
        {
            std::lock_guard<std::mutex> lk (cmds_mtx);
            operator_cmds.push_back (cmd);
        });
    SMMTestAccess::setAsset (smm, reinterpret_cast<smm_asset> (&dummy));
    SMMTestAccess::setSearchActive (smm, true);
    SMMTestAccess::setSearch (smm, std::make_shared<SMMSearch> ());

    smm.next_operator_command = SMM_COMMAND_ABANDON_SEARCH;
    smm.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0));

    /* The held search is dropped (without completing it -- SMMSearch's
     * destructor only ever calls smm_search_destroy, never smm_search_complete)
     * and maybeAcquire(), called right after in doReportPosition(), attempts a
     * fresh fetch since nothing is held any more. */
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.fetch_calls.load () >= 1; }, io_timeout));
    REQUIRE (MavLoopbackServer::waitFor (
        [&]
        {
            std::lock_guard<std::mutex> lk (cmds_mtx);
            return operator_cmds.size () == 1;
        },
        io_timeout));
    {
        std::lock_guard<std::mutex> lk (cmds_mtx);
        REQUIRE (operator_cmds[0] == smm_cmd_abandon_search);
    }

    /* A second report with the SAME operator command must not re-fire:
     * smm_asset_last_command() keeps reporting the same value until the
     * operator issues a different one, so without edge-triggering this would
     * re-drop (and endlessly re-churn) whatever gets reacquired in between. */
    std::this_thread::sleep_for (std::chrono::milliseconds (20));
    smm.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0));
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.report_calls.load () >= 2; }, io_timeout));
    {
        std::lock_guard<std::mutex> lk (cmds_mtx);
        REQUIRE (operator_cmds.size () == 1);
    }
}

TEST_CASE ("SMM reports an operator mission-complete command once, without touching the held search", "[mav_io]")
{
    reset_mav_parser ();
    MAV mav ("127.0.0.1", 1, terminate_action::none, test_mav_params, test_logger);

    int dummy = 0;
    TestSMM smm (mav, test_logger, test_altitude_cap_m, test_altitude_floor_m, 90.0, 10, test_smm_connect_timeout_s,
                 test_smm_transfer_timeout_s);
    smm.registerLoadSearchCB ([] (const std::shared_ptr<SMMSearch> &) {});
    smm.registerRtlCB ([] {});
    std::mutex cmds_mtx;
    std::vector<SMMCommand> operator_cmds;
    smm.registerOperatorCommandCB (
        [&] (SMMCommand cmd)
        {
            std::lock_guard<std::mutex> lk (cmds_mtx);
            operator_cmds.push_back (cmd);
        });
    SMMTestAccess::setAsset (smm, reinterpret_cast<smm_asset> (&dummy));
    /* No search active/held: a mission-complete report is only about the
     * operator-command callback, not search acquisition. */

    smm.next_operator_command = SMM_COMMAND_MISSION_COMPLETE;
    smm.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0));

    REQUIRE (MavLoopbackServer::waitFor (
        [&]
        {
            std::lock_guard<std::mutex> lk (cmds_mtx);
            return operator_cmds.size () == 1;
        },
        io_timeout));
    {
        std::lock_guard<std::mutex> lk (cmds_mtx);
        REQUIRE (operator_cmds[0] == smm_cmd_mission_complete);
    }
    REQUIRE (smm.commit_calls.load () == 0);

    /* Repeated reports with the same command must not re-fire. */
    std::this_thread::sleep_for (std::chrono::milliseconds (20));
    smm.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0));
    REQUIRE (MavLoopbackServer::waitFor ([&] { return smm.report_calls.load () >= 2; }, io_timeout));
    {
        std::lock_guard<std::mutex> lk (cmds_mtx);
        REQUIRE (operator_cmds.size () == 1);
    }
}

/* Malformed-byte-stream robustness, the single-repo share of Tier-3 Path K
 * k01. These drive the real mav_connection recv path (not a mock) over the
 * loopback socket, so hostile bytes exercise the actual MAVLink parser and
 * its resync behaviour; the full-system flood (SITL + FSS + the cap-fmu
 * binary) stays in Tier-3 Path K as the cross-check. Run under the same
 * TSan `make check` as everything else, so a parser-state race would
 * surface here too. */

TEST_CASE ("mav_connection survives a stream of random garbage bytes", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
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

TEST_CASE ("mav_connection resynchronises after a truncated frame", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
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

TEST_CASE ("mav_connection drops a corrupted-CRC frame without flapping link state", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
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

TEST_CASE ("mav_connection keeps routing valid messages between garbage bursts", "[mav_io][fuzz]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
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

/* Cap-fmu's GPS-denial/no-fix handling, the single-repo share of Tier-3 Path M
 * m03. Investigation found that mav_connection's MAVLINK_MSG_ID_GPS_RAW_INT case
 * (fix_type — the autopilot's own no-fix/2D/ 3D indicator) was a recognised
 * no-op: received and silently dropped, while GLOBAL_POSITION_INT (the EKF's
 * position estimate, which has no fix-validity field of its own) was always
 * forwarded as if fresh. That gap is now closed: fix_type is now tracked and the
 * next GLOBAL_POSITION_INT's report carries a POSITION_FLAG_VALID_COORDS bit
 * reflecting it — the position is still delivered (not suppressed), but
 * downstream (FSS-Web) can now tell a fix- backed report from a degraded/lost
 * one instead of treating both as current. */

TEST_CASE ("GPS_RAW_INT fix_type gates the position report's valid-coords flag", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* Re-send this phase's position until the callback echoes its (distinct)
     * coordinates back. Waiting on the coordinates rather than a count bump
     * makes each phase immune to a straggler re-send from the previous phase:
     * frames arrive in TCP-stream order, so once this phase's coordinates come
     * back, the GPS_RAW_INT sent before them has been processed. A count-based
     * wait could instead be satisfied by a previous phase's final re-send —
     * still carrying the old fix state — and read a stale flag. */
    auto waitForEchoedPosition = [&] (int32_t lat_e7, int32_t lon_e7) -> PositionData
    {
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPosition (lat_e7, lon_e7);
                PositionData last = positions.lastPosition ();
                return last.getP ().getLatitude () == Catch::Approx (lat_e7 * 1e-7)
                       && last.getP ().getLongitude () == Catch::Approx (lon_e7 * 1e-7);
            },
            io_timeout));
        return positions.lastPosition ();
    };

    /* No GPS_RAW_INT has arrived yet: the default (GPS_FIX_TYPE_NO_GPS) must
     * not be mistaken for a good fix, so a position reported before the first
     * fix_type is flagged invalid too. */
    PositionData pd_before_fix = waitForEchoedPosition (-435000000, 1726000000);
    REQUIRE (pd_before_fix.getP ().getLatitude () == Catch::Approx (-43.5));
    REQUIRE (pd_before_fix.getP ().getLongitude () == Catch::Approx (172.6));
    REQUIRE ((pd_before_fix.getFlags () & POSITION_FLAG_VALID_COORDS) == 0);

    /* An explicit no-fix report: the position is still delivered unchanged,
     * just flagged. */
    server.sendGpsRawInt (GPS_FIX_TYPE_NO_FIX);
    PositionData pd_no_fix = waitForEchoedPosition (-436000000, 1727000000);
    REQUIRE (pd_no_fix.getP ().getLatitude () == Catch::Approx (-43.6));
    REQUIRE (pd_no_fix.getP ().getLongitude () == Catch::Approx (172.7));
    REQUIRE ((pd_no_fix.getFlags () & POSITION_FLAG_VALID_COORDS) == 0);

    /* Once a real fix comes in, the next position report is flagged valid. */
    server.sendGpsRawInt (GPS_FIX_TYPE_3D_FIX);
    PositionData pd_good_fix = waitForEchoedPosition (-437000000, 1728000000);
    REQUIRE ((pd_good_fix.getFlags () & POSITION_FLAG_VALID_COORDS) != 0);
}

/* GLOBAL_POSITION_INT carries two altitude fields -- alt (MSL) and
 * relative_alt (AGL, relative to home) -- and PositionData now carries both
 * separately (alt_m / alt_agl_m). This proves the recv path reads each field
 * into the right one, by setting them to different values: if the
 * implementation accidentally read alt for both, this test would fail. */
TEST_CASE ("GLOBAL_POSITION_INT's alt and relative_alt populate separate PositionData fields", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* alt (MSL) = 500m, relative_alt (AGL) = 80m -- deliberately different. */
    constexpr int32_t alt_mm = 500000;
    constexpr int32_t relative_alt_mm = 80000;
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendPosition (-435000000, 1726000000, alt_mm, relative_alt_mm);
            return positions.count_now () > 0;
        },
        io_timeout));

    PositionData pd = positions.lastPosition ();
    /* Regression guard: this change did not touch alt_m's existing MSL
     * semantics. */
    REQUIRE (pd.getAltitudeMetres () == Catch::Approx (500.0));
    /* The specific bug-shaped assertion: alt_agl_m reflects relative_alt, not
     * alt -- if the implementation read alt twice, this would read 500.0. */
    REQUIRE (pd.getAltitudeAGLMetres () == Catch::Approx (80.0));
}

TEST_CASE ("GLOBAL_POSITION_INT with frozen coordinates is reported unchanged each time", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* Simulate an EKF position estimate that has frozen (e.g. GPS lost, no
     * further update) by sending the exact same coordinates repeatedly.
     * Today nothing detects the repetition or suppresses/flags it: each
     * report is delivered exactly like a fresh one, with no way downstream
     * to tell a frozen estimate from a genuinely updating one. */
    constexpr int32_t frozen_lat = -435000000;
    constexpr int32_t frozen_lon = 1726000000;
    for (int i = 0; i < 5; i++)
    {
        int before = positions.count_now ();
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPosition (frozen_lat, frozen_lon);
                return positions.count_now () > before;
            },
            io_timeout));

        PositionData pd = positions.lastPosition ();
        REQUIRE (pd.getP ().getLatitude () == Catch::Approx (-43.5));
        REQUIRE (pd.getP ().getLongitude () == Catch::Approx (172.6));
    }
    REQUIRE (positions.count_now () >= 5);
}

/* An ArduPilot reboot is typically a ~3s heartbeat gap, well under
 * heartbeat_loop()'s 5s link-down timeout, so it produces no comms edge and the
 * FSS connection never notices either. A backwards jump in the autopilot's own
 * uptime counter is the only evidence this end gets. These cases pin the
 * detector and the resync it drives: re-requested streams (the reboot discarded
 * the SET_MESSAGE_INTERVAL overrides) and a dropped belief about what mission is
 * loaded. */
namespace
{
/* Collect the three SET_MESSAGE_INTERVAL requests the FMU issues on completing
 * setup for the autopilot, returning the message ids it asked for (sorted, since
 * only the set matters here). */
auto
recv_stream_requests (MavLoopbackServer &server) -> std::vector<uint32_t>
{
    std::vector<uint32_t> ids;
    for (int i = 0; i < 3; i++)
    {
        mavlink_message_t msg;
        REQUIRE (server.recvMessage (MAVLINK_MSG_ID_COMMAND_LONG, msg, io_timeout));
        REQUIRE (mavlink_msg_command_long_get_command (&msg) == MAV_CMD_SET_MESSAGE_INTERVAL);
        ids.push_back (static_cast<uint32_t> (mavlink_msg_command_long_get_param1 (&msg)));
    }
    std::sort (ids.begin (), ids.end ());
    return ids;
}

const std::vector<uint32_t> expected_stream_ids{ MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                                 MAVLINK_MSG_ID_BATTERY_STATUS };
} // namespace

TEST_CASE ("a backwards jump in the autopilot's uptime is reported as a restart and re-requests the streams",
           "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;
    CapturingLogger capture;
    std::atomic<int> restarts{ 0 };

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.registerAutopilotRestartCB ([&restarts] () { restarts.fetch_add (1); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* Drain the streams requested on first setup, so the ones asserted after the
     * restart can only be the re-issued set. */
    std::vector<uint32_t> ids = recv_stream_requests (server);
    std::sort (ids.begin (), ids.end ());
    std::vector<uint32_t> expected = expected_stream_ids;
    std::sort (expected.begin (), expected.end ());
    REQUIRE (ids == expected);

    /* Send an uptime until a position from *this* call has been processed.
     * Re-sending the same uptime is safe: equal values are not a regression, so
     * the loop cannot itself trip the detector.
     *
     * Each call marks its positions with its own longitude, and waits for that
     * longitude to reach the callback. Waiting on the position count instead
     * would not be a barrier at all: a message still queued from an earlier
     * call can bump the count, ending the loop before anything it sent has been
     * seen. */
    int marker = 0;
    auto sendUptimeAndWait = [&] (uint32_t time_boot_ms)
    {
        /* Whole degrees apart, so no tolerance question arises. */
        const int32_t lon = 1726000000 + (++marker) * 10000000;
        const double want_lon = lon / 1.0e7;
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPositionAt (time_boot_ms, -435000000, lon);
                return positions.lastPosition ().getP ().getLongitude () == Catch::Approx (want_lon);
            },
            io_timeout));
    };

    /* An autopilot that has been up for 15 minutes. */
    sendUptimeAndWait (900000);
    REQUIRE (restarts.load () == 0);

    /* It reboots: uptime restarts from near zero, with the link never dropping. */
    sendUptimeAndWait (1200);
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return restarts.load () >= 1; }, io_timeout));
    REQUIRE (capture.containsSubstring ("Autopilot restart detected"));

    /* The reboot discarded the runtime stream requests, so they are issued again
     * on the *next* message from the autopilot: handleAutopilotRestart() re-arms
     * the setup latch, but processMavLinkMsg() only tests that latch on entry,
     * so the message that tripped the detector is already past it.
     *
     * Drive that next message explicitly rather than leaning on the wait loop
     * above having happened to send a spare one — that is what made this case
     * flake under tsan in CI, where the loop could exit having sent the
     * post-reboot uptime exactly once. Observing the restart callback is enough
     * to order this send after resetAllSetup(), which precedes it. */
    server.sendPositionAt (1200);
    REQUIRE (recv_stream_requests (server) == expected);

    /* Exactly one restart: the post-reboot uptime becomes the new baseline, so
     * the wait loop's repeats of it do not each look like another restart. */
    sendUptimeAndWait (1400);
    REQUIRE (restarts.load () == 1);
}

TEST_CASE ("SYSTEM_TIME feeds the restart detector, and only from the configured autopilot", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;
    CapturingLogger capture;
    std::atomic<int> restarts{ 0 };

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.registerAutopilotRestartCB ([&restarts] () { restarts.fetch_add (1); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* TCP delivery is ordered and mav_connection processes on a single recv
     * thread, so once a position sent afterwards is echoed back, everything
     * before it has been processed — the barrier every "and then nothing
     * happened" assertion below relies on.
     *
     * That only holds if the position echoed back is one this call sent, so
     * each marks its own with a distinct longitude (whole degrees apart, so no
     * tolerance question arises). Waiting on the position count instead would
     * let a message still queued from an earlier barrier end this one early,
     * leaving the assertion that follows to run against state the FMU has not
     * caught up with yet. */
    int marker = 0;
    auto barrier = [&] ()
    {
        const int32_t lon = 1726000000 + (++marker) * 10000000;
        const double want_lon = lon / 1.0e7;
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPosition (-435000000, lon);
                return positions.lastPosition ().getP ().getLongitude () == Catch::Approx (want_lon);
            },
            io_timeout));
    };

    server.sendSystemTime (900000);
    barrier ();
    REQUIRE (restarts.load () == 0);

    /* A backwards jump attributed to some other system on the link (a second
     * vehicle, a GCS) must not be read as this autopilot restarting. */
    server.sendSystemTime (1200, /*sysid*/ 2);
    barrier ();
    REQUIRE (restarts.load () == 0);

    /* The same jump from the autopilot itself is the restart. */
    server.sendSystemTime (1200);
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return restarts.load () >= 1; }, io_timeout));
    REQUIRE (restarts.load () == 1);
}

TEST_CASE ("an autopilot uptime that only advances is never read as a restart", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;
    std::atomic<int> restarts{ 0 };

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.registerAutopilotRestartCB ([&restarts] () { restarts.fetch_add (1); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* Interleave the two sources at their real relative rates (position 5Hz,
     * SYSTEM_TIME 1Hz): they are stamped at slightly different instants, and a
     * detector keyed on a bare "lower than last seen" would false-trip on the
     * skew between them. */
    for (uint32_t t = 900000; t < 901000; t += 200)
    {
        const int before = positions.count_now ();
        server.sendSystemTime (t - 150);
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPositionAt (t);
                return positions.count_now () > before;
            },
            io_timeout));
    }
    REQUIRE (restarts.load () == 0);
}

TEST_CASE ("a restart drops the loaded-search belief so the next load re-uploads", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;
    std::atomic<int> restarts{ 0 };

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.registerAutopilotRestartCB ([&restarts] () { restarts.fetch_add (1); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    auto search = std::make_shared<SMMSearch> ();
    conn.loadSearch (search);
    REQUIRE (run_mission_upload (server, 3).set_current == search_point_mission_seq (0));
    expect_auto_mode (server);

    /* Re-loading the same search while it is still believed loaded is a no-op —
     * this is the behaviour that would silently swallow the re-apply after a
     * reboot if the belief were not cleared. */
    conn.loadSearch (search);
    mavlink_message_t msg;
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, no_message_timeout));

    /* The autopilot reboots, wiping the uploaded mission. */
    auto sendUptimeAndWait = [&] (uint32_t time_boot_ms)
    {
        const int before = positions.count_now ();
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPositionAt (time_boot_ms);
                return positions.count_now () > before;
            },
            io_timeout));
    };
    sendUptimeAndWait (900000);
    sendUptimeAndWait (1200);
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return restarts.load () >= 1; }, io_timeout));

    /* Now the identical re-load genuinely re-uploads, which is what makes the
     * state machine's re-apply of fmu_state_searching mean anything. */
    conn.loadSearch (search);
    REQUIRE (run_mission_upload (server, 3).set_current == search_point_mission_seq (0));
}

TEST_CASE ("A heartbeat from a system other than the configured autopilot does not affect comms health or a "
           "deferred mode",
           "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    /* Cold start: no heartbeat from any system yet. */
    REQUIRE (
        MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout));

    /* Defer an RTL while the autopilot type is unknown (the deferred-mode
     * path): resolve_mav_mode(unknown, rtl) has no mapping, so the
     * SET_MODE is held until a heartbeat resolves the airframe type. */
    conn.commandRTL ();

    mavlink_message_t msg;
    /* A ground control station and a second vehicle sharing this link both
     * heartbeat. Neither may report comms up or resolve the deferred RTL —
     * only the configured autopilot system (sysid 1) may. */
    constexpr uint8_t gcs_sysid = 255;
    constexpr uint8_t other_vehicle_sysid = 2;
    for (int i = 0; i < 3; i++)
    {
        server.sendHeartbeat (MAV_TYPE_GCS, gcs_sysid);
        server.sendHeartbeat (MAV_TYPE_QUADROTOR, other_vehicle_sysid);
    }
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, no_message_timeout));
    REQUIRE (recorder.lastStatus () == MavCommsStatus::failure);

    /* The real autopilot's heartbeat now resolves both: comms reports up and
     * the deferred RTL is finally sent. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat ();
            return recorder.lastStatus () == MavCommsStatus::ok;
        },
        io_timeout));
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, io_timeout));
    REQUIRE (mavlink_msg_set_mode_get_custom_mode (&msg) == COPTER_MODE_RTL);
}

TEST_CASE ("A GLOBAL_POSITION_INT or GPS_RAW_INT from a system other than the configured autopilot has no effect",
           "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    auto waitForEchoedPosition = [&] (int32_t lat_e7, int32_t lon_e7) -> PositionData
    {
        REQUIRE (MavLoopbackServer::waitFor (
            [&] ()
            {
                server.sendPosition (lat_e7, lon_e7);
                PositionData last = positions.lastPosition ();
                return last.getP ().getLatitude () == Catch::Approx (lat_e7 * 1e-7)
                       && last.getP ().getLongitude () == Catch::Approx (lon_e7 * 1e-7);
            },
            io_timeout));
        return positions.lastPosition ();
    };

    /* Establish a good fix from the real autopilot. */
    server.sendGpsRawInt (GPS_FIX_TYPE_3D_FIX);
    PositionData pd_good = waitForEchoedPosition (-435000000, 1726000000);
    REQUIRE ((pd_good.getFlags () & POSITION_FLAG_VALID_COORDS) != 0);

    /* A second vehicle on the same link reports a wildly different position
     * and a lost fix. Neither may be observed anywhere: not the position
     * callback, not getLastPosition(), and not the next real report's
     * validity flag (which is driven by gps_fix_type). A strict callback-
     * count check would be racy here — a duplicate re-send of the prior
     * real position (from waitForEchoedPosition's retry loop) can still be
     * in flight and land during this window — so check content instead: the
     * foreign (0, 0) coordinates must never be observed. */
    constexpr uint8_t other_vehicle_sysid = 2;
    const auto deadline = std::chrono::steady_clock::now () + no_message_timeout;
    while (std::chrono::steady_clock::now () < deadline)
    {
        server.sendPosition (0, 0, other_vehicle_sysid);
        server.sendGpsRawInt (GPS_FIX_TYPE_NO_FIX, other_vehicle_sysid);
        std::this_thread::sleep_for (std::chrono::milliseconds (10));
    }
    REQUIRE (positions.lastPosition ().getP ().getLatitude () == Catch::Approx (-43.5));
    REQUIRE (positions.lastPosition ().getP ().getLongitude () == Catch::Approx (172.6));
    REQUIRE (conn.getLastPosition ().getLatitude () == Catch::Approx (-43.5));
    REQUIRE (conn.getLastPosition ().getLongitude () == Catch::Approx (172.6));

    /* The real autopilot's next report is still flagged valid — proving the
     * foreign GPS_RAW_INT(NO_FIX) never touched gps_fix_type. */
    PositionData pd_after = waitForEchoedPosition (-436000000, 1727000000);
    REQUIRE ((pd_after.getFlags () & POSITION_FLAG_VALID_COORDS) != 0);
}

TEST_CASE ("A BATTERY_STATUS or MISSION_ITEM_REACHED from a system other than the configured autopilot produces no "
           "callback",
           "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    BatteryRecorder battery;
    ReachedRecorder reached;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerBatteryCB ([&battery] (const BatteryData &bd) { battery.record (bd); });
    conn.registerReachedCB ([&reached] (int point) { reached.record (point); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    constexpr uint8_t other_vehicle_sysid = 2;
    constexpr uint16_t reached_seq = 3;

    /* A second vehicle on the same link reports its own battery status and
     * mission progress. Neither callback may fire. */
    const int before_battery = battery.count_now ();
    const int before_reached = reached.count_now ();
    const auto deadline = std::chrono::steady_clock::now () + no_message_timeout;
    while (std::chrono::steady_clock::now () < deadline)
    {
        server.sendBatteryStatus (/*remaining*/ 5, /*consumed*/ 100, /*cell0_mv*/ 10000, other_vehicle_sysid);
        server.sendMissionItemReached (reached_seq, other_vehicle_sysid);
        std::this_thread::sleep_for (std::chrono::milliseconds (10));
    }
    REQUIRE (battery.count_now () == before_battery);
    REQUIRE (reached.count_now () == before_reached);

    /* The real autopilot's equivalent reports still reach both callbacks. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendBatteryStatus (50, 500, 12000);
            return battery.count_now () > before_battery;
        },
        io_timeout));
    REQUIRE (battery.lastBattery ().getRemaining () == 50);

    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendMissionItemReached (reached_seq);
            return reached.count_now () > before_reached;
        },
        io_timeout));
    REQUIRE (reached.lastPoint () == static_cast<int> (reached_seq) - search_first_point_seq + 1);
}

TEST_CASE ("A MISSION_REQUEST_INT or MISSION_ACK from a system other than the configured autopilot has no mission "
           "side effects",
           "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, test_logger);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    conn.commandGoto (Point (-43.5, 172.6));

    mavlink_message_t msg;
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_COUNT, msg, io_timeout));
    const uint8_t mission_type = mavlink_msg_mission_count_get_mission_type (&msg);
    const uint16_t count = mavlink_msg_mission_count_get_count (&msg);

    constexpr uint8_t other_vehicle_sysid = 2;

    /* A second vehicle's request for the first item is not served — the
     * upload only responds to the configured autopilot. */
    server.sendMissionRequestInt (0, mission_type, other_vehicle_sysid);
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_MISSION_ITEM_INT, msg, no_message_timeout));

    /* The real autopilot's requests still drive the upload to completion. */
    for (uint16_t seq = 0; seq < count; seq++)
    {
        server.sendMissionRequestInt (seq, mission_type);
        REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_ITEM_INT, msg, io_timeout));
    }

    /* A second vehicle's accepted ack must not select current or engage AUTO. */
    server.sendMissionAck (mission_type, other_vehicle_sysid);
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_MISSION_SET_CURRENT, msg, no_message_timeout));
    REQUIRE_FALSE (server.recvMessage (MAVLINK_MSG_ID_SET_MODE, msg, no_message_timeout));

    /* The real autopilot's accepted ack still completes the upload. */
    server.sendMissionAck (mission_type);
    REQUIRE (server.recvMessage (MAVLINK_MSG_ID_MISSION_SET_CURRENT, msg, io_timeout));
    REQUIRE (mavlink_msg_mission_set_current_get_seq (&msg) == 0);
    expect_auto_mode (server);
}

TEST_CASE ("--terminate-action=terminate against AFS_ENABLE=0 logs a distinct failsafe-config warning", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::terminate);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    server.sendParamValue ("AFS_ENABLE", 0.0F);
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("AFS_ENABLE=0"); }, io_timeout));
}

TEST_CASE ("--terminate-action=terminate against a matching AFS config stays silent", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::terminate);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    server.sendParamValue ("AFS_ENABLE", 1.0F);
    server.sendParamValue ("AFS_TERM_ACTION", 1.0F);
    /* Prove the param values above were fully processed before checking for
     * absence of a warning: TCP delivery is ordered and mav_connection
     * processes on a single recv thread, so once a position sent after them
     * is echoed back to the callback, any warning they would have triggered
     * has already been logged (or not). */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendPosition ();
            return positions.count_at_least (1);
        },
        io_timeout));
    REQUIRE_FALSE (capture.containsSubstring ("AFS_ENABLE"));
    REQUIRE_FALSE (capture.containsSubstring ("AFS_TERM_ACTION"));
}

TEST_CASE ("--terminate-action=disarm never requests AFS params, but still checks the GCS failsafe", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::disarm);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    auto params = collectParamRequests (server);

    REQUIRE_FALSE (requested (params, "AFS_ENABLE"));
    REQUIRE_FALSE (requested (params, "AFS_TERM_ACTION"));
    /* waitForColdStartThenUp's heartbeats default to MAV_TYPE_QUADROTOR. */
    REQUIRE (requested (params, "FS_GCS_ENABLE"));
    /* The GCS-failsafe checks apply regardless of --terminate-action too: they
     * back the comms-loss/low-battery latches, not termination. */
    REQUIRE (requested (params, "FS_OPTIONS"));
    REQUIRE (requested (params, "SYSID_MYGCS"));
}

TEST_CASE ("the GCS-failsafe param name requested is airframe-specific", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::none);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    /* Cold start: no heartbeat yet, so the first one sent below is genuinely
     * the one that resolves the airframe type and drives the failsafe check
     * (mirroring the other-system heartbeat test above, not
     * waitForColdStartThenUp, since that helper's heartbeats are always
     * MAV_TYPE_QUADROTOR). */
    REQUIRE (
        MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout));
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat (MAV_TYPE_FIXED_WING);
            return recorder.lastStatus () == MavCommsStatus::ok;
        },
        io_timeout));

    auto params = collectParamRequests (server);
    REQUIRE (requested (params, "FS_GCS_ENABL"));
    REQUIRE_FALSE (requested (params, "FS_GCS_ENABLE"));
    /* Plane-only: Copter encodes the same trap in FS_OPTIONS instead. */
    REQUIRE (requested (params, "FS_LONG_ACTN"));
    REQUIRE_FALSE (requested (params, "FS_OPTIONS"));
}

TEST_CASE ("the failsafe-config check re-runs if a later heartbeat reports a different autopilot type", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::none);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    /* First heartbeat: MAV_TYPE_QUADROTOR, resolving to FS_GCS_ENABLE. */
    REQUIRE (waitForColdStartThenUp (server, recorder));

    REQUIRE (requested (collectParamRequests (server), "FS_GCS_ENABLE"));

    /* MAV_TYPE is fixed by firmware and never changes at runtime for a
     * genuine autopilot; this stands in for the residual risk (a second
     * vehicle sharing this sysid on a hub) to prove the check doesn't stay
     * silent forever once latched for the first-seen type. */
    server.sendHeartbeat (MAV_TYPE_FIXED_WING);
    REQUIRE (requested (collectParamRequests (server), "FS_GCS_ENABL"));
}

TEST_CASE ("a disabled GCS/telemetry failsafe on the autopilot logs a distinct warning", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::none);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* waitForColdStartThenUp's heartbeats default to MAV_TYPE_QUADROTOR, which
     * resolves to FS_GCS_ENABLE. */
    server.sendParamValue ("FS_GCS_ENABLE", 0.0F);
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("FS_GCS_ENABLE=0"); }, io_timeout));
}

/* The gap this closes is a *nonzero* value that still leaves no backstop --
 * the enable flag says the failsafe fires, not what it does. Each case below
 * would have passed a bare nonzero test in silence. */
TEST_CASE ("a GCS failsafe that continues the mission logs a warning despite being enabled", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::none);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    SECTION ("Copter: FS_GCS_ENABLE=2 continues the mission in AUTO")
    {
        server.sendParamValue ("FS_GCS_ENABLE", 2.0F);
        REQUIRE (
            MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("FS_GCS_ENABLE=2"); }, io_timeout));
    }

    SECTION ("Copter: FS_OPTIONS bit 1 set is the same trap")
    {
        server.sendParamValue ("FS_OPTIONS", 2.0F);
        REQUIRE (
            MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("FS_OPTIONS=2"); }, io_timeout));
    }

    SECTION ("SYSID_MYGCS at its 255 default watches MAVProxy, not the FMU")
    {
        server.sendParamValue ("SYSID_MYGCS", 255.0F);
        REQUIRE (
            MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("SYSID_MYGCS=255"); }, io_timeout));
    }
}

TEST_CASE ("Plane's FS_LONG_ACTN=0 warns even with FS_GCS_ENABL=1", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::none);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));

    /* Cold start with a Plane heartbeat, so the check resolves to the Plane
     * parameter set rather than waitForColdStartThenUp's quadrotor. */
    REQUIRE (
        MavLoopbackServer::waitFor ([&] () { return recorder.lastStatus () == MavCommsStatus::failure; }, io_timeout));
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendHeartbeat (MAV_TYPE_FIXED_WING);
            return recorder.lastStatus () == MavCommsStatus::ok;
        },
        io_timeout));

    /* Exactly the configuration this check exists for: the failsafe is
     * enabled, and its action is the firmware default, Continue. */
    server.sendParamValue ("FS_GCS_ENABL", 1.0F);
    server.sendParamValue ("FS_LONG_ACTN", 0.0F);
    REQUIRE (MavLoopbackServer::waitFor ([&] () { return capture.containsSubstring ("FS_LONG_ACTN=0"); }, io_timeout));
    REQUIRE_FALSE (capture.containsSubstring ("FS_GCS_ENABL="));
}

TEST_CASE ("a fleet-configured airframe stays silent", "[mav_io]")
{
    reset_mav_parser ();
    MavLoopbackServer server;
    CommsRecorder recorder;
    CapturingLogger capture;
    PositionRecorder positions;

    mav_connection conn ("127.0.0.1", server.port (), test_mav_params, capture, terminate_action::none);
    conn.registerMavCommsStatusCB ([&recorder] (MavCommsStatus status) { recorder.record (status); });
    conn.registerPositionCB ([&positions] (const PositionData &pd) { positions.record (pd); });
    conn.start ();
    REQUIRE (server.waitForClient (io_timeout));
    REQUIRE (waitForColdStartThenUp (server, recorder));

    /* The decided fleet configuration for a Copter (see the README table). */
    server.sendParamValue ("FS_GCS_ENABLE", 1.0F);
    server.sendParamValue ("FS_OPTIONS", 0.0F);
    server.sendParamValue ("SYSID_MYGCS", 200.0F);
    /* Same ordering argument as the AFS silence test above: once a position
     * sent after them is echoed back, they have already been processed. */
    REQUIRE (MavLoopbackServer::waitFor (
        [&] ()
        {
            server.sendPosition ();
            return positions.count_at_least (1);
        },
        io_timeout));
    REQUIRE_FALSE (capture.containsSubstring ("FS_GCS_ENABLE="));
    REQUIRE_FALSE (capture.containsSubstring ("FS_OPTIONS="));
    REQUIRE_FALSE (capture.containsSubstring ("SYSID_MYGCS="));
}
