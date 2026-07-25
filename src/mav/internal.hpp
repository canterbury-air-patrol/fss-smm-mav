#pragma once
#include <ardupilotmega/mavlink.h>
#include <atomic>
#include <condition_variable>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <sys/socket.h>
#include <thread>

#include "../fmu-types.hpp"
#include "../ilogger.hpp"
#include "../smm/smm.hpp"
#include "../util.hpp"
#include "mav-params.hpp"
#include "mav.hpp"
#include "mode-resolve.hpp"

class mav_comp
{
  private:
    uint8_t compid;

  public:
    explicit mav_comp (uint8_t t_compid) : compid (t_compid) {};
    auto
    getCompId () -> uint8_t
    {
        return this->compid;
    };
};

/* The autopilot_type/flight_mode/setup/failsafe_checked scalars are written
 * on the recv thread (via processMavLinkMsg) and read on the event-loop
 * thread (getAutoPilotType() in commandRTL/Hold/Manual/Auto), so they are
 * atomic to make that cross-thread access well-defined. The components list
 * is only ever touched on the recv thread (findComponent from
 * processMavLinkMsg), so it needs no lock. */
class mav_sys
{
  private:
    uint8_t sysid;
    std::list<std::shared_ptr<mav_comp>> components{};
    std::atomic<uint8_t> autopilot_type{ 0 };
    std::atomic<uint8_t> flight_mode{ 0 };
    std::atomic<bool> setup{ false };
    std::atomic<bool> failsafe_checked{ false };
    std::atomic<uint8_t> failsafe_checked_type{ 0 };

  public:
    explicit mav_sys (uint8_t t_sysid) : sysid (t_sysid) {};
    ~mav_sys () = default;
    auto
    getSysId () -> uint8_t
    {
        return this->sysid;
    };
    auto
    getAutoPilotType () -> uint8_t
    {
        return this->autopilot_type.load ();
    };
    auto
    getFlightMode () -> uint8_t
    {
        return this->flight_mode.load ();
    };
    void
    setAutoPilotMode (uint8_t type)
    {
        this->autopilot_type.store (type);
    };
    void
    setFlightMode (uint8_t mode)
    {
        this->flight_mode.store (mode);
    };
    auto findComponent (uint8_t compid) -> std::shared_ptr<mav_comp>;
    auto
    isSetup () -> bool
    {
        return this->setup.load ();
    };
    void
    setupComplete ()
    {
        this->setup.store (true);
    };
    /* Whether the todo/91 failsafe-config sanity check has already been
     * issued for this system for the given (heartbeat-reported) `type`.
     * MAV_TYPE is fixed by firmware and does not change at runtime for a
     * genuine autopilot, so in practice this only ever fires once; it is
     * keyed on `type` (not a plain done/not-done flag) so that if it ever
     * does differ on a later heartbeat — e.g. a second vehicle sharing this
     * sysid on a hub (todo/97) — the check re-runs for whichever type is now
     * claiming this system, rather than staying silent forever on the
     * first-seen type. */
    auto
    checkedFailsafeFor (uint8_t type) -> bool
    {
        return this->failsafe_checked.load () && this->failsafe_checked_type.load () == type;
    };
    void
    markFailsafeCheckedFor (uint8_t type)
    {
        this->failsafe_checked_type.store (type);
        this->failsafe_checked.store (true);
    };
};

/* The systems list is grown on the recv thread (findSystem find-or-create from
 * processMavLinkMsg) and read from the event-loop command paths, so all access
 * to the list is serialised by `lock`. Command paths use findExistingSystem(),
 * which never mutates the list, so dispatching a command cannot race a
 * concurrent recv-thread insertion. */
class mav_systems
{
  private:
    std::list<std::shared_ptr<mav_sys>> systems{};
    std::mutex lock{};

  public:
    mav_systems () = default;
    ~mav_systems () = default;
    /* Find the system for t_sysid, creating it if absent. Recv-thread only. */
    auto findSystem (uint8_t t_sysid) -> std::shared_ptr<mav_sys>;
    /* Return the system for t_sysid if it already exists, else nullptr. Never
     * mutates the list, so it is safe to call from the command paths. */
    auto findExistingSystem (uint8_t t_sysid) -> std::shared_ptr<mav_sys>;
};

class mav_connection
{
  private:
    std::string addr;
    uint16_t port;
    /* Atomic because the recv and heartbeat loops read it lock-free on their own
     * threads; send_lock additionally serialises its publish (connect_to_mav) and
     * retire (disconnect_from_mav) against in-flight sends, so the descriptor's
     * lifetime — not just its value — is safe across threads. */
    std::atomic<int> fd{ -1 };
    uint64_t last_tried{ 0 };
    uint16_t retry_count{ 0 };
    /* Guards both the socket write and MAVLink's per-channel transmit state.
     * Generated *_pack_chan() helpers update global sequence/status for the
     * channel they finalize on, so packing and sending must be serialized. */
    std::mutex send_lock{};
    std::thread recv_thread{};
    std::thread heartbeat_thread{};
    std::atomic<bool> broken{ false };
    std::atomic<bool> stopping{ false };
    std::atomic<bool> started{ false };
    std::mutex heartbeat_mutex{};
    std::condition_variable heartbeat_cv{};
    std::atomic<uint64_t> last_heartbeat_ts{ 0 };
    /* Last *reported* MAV comms status, owned solely by heartbeat_loop(). Starts
     * "up" to match the state machine's optimistic default, so the first observed
     * down state (no link/heartbeat at cold start) edge-triggers a failure report
     * that corrects it, rather than the link being silently assumed healthy. */
    std::atomic<bool> mav_comms_ok{ true };
    notify_mav_comms_cb mav_comms_cb{};
    mav_systems systems{};
    /* state_lock guards: last_position, search, search_loaded,
     * search_loading, goto_active, goto_position, goto_ack_pending,
     * retry_count, last_tried. Never held at the same time as send_lock (see
     * docs/threading.md for the full cross-thread ownership/lock-ordering
     * model this connection is one piece of). */
    std::mutex state_lock{};
    Point last_position{};
    std::shared_ptr<SMMSearch> search{ nullptr };
    notify_position_cb position_cb{};
    notify_battery_status_cb battery_cb{};
    notify_reached_cb reached_cb{};
    bool search_loading{ false };
    bool search_loaded{ false };
    Point goto_position{};
    bool goto_active{ false };
    /* True from a successfully-sent goto MISSION_COUNT until its MISSION_ACK
     * is processed (or it is superseded by a new goto/search). setMode()'s
     * "sent" for a goto only reflects that opening packet (todo/61) — the
     * rest of the upload is request-driven and the FMU has no replay-on-
     * recovery guarantee for it (unlike RTL/failsafe/low-battery/terminate,
     * todo/46). This flag lets a link drop mid-upload be reported instead of
     * silently treated as a successful goto. */
    bool goto_ack_pending{ false };
    /* The autopilot's own GPS health indicator (GPS_RAW_INT.fix_type), tracked
     * so a position report can carry whether its coordinates are backed by a
     * real fix (todo/79). Defaults to NO_GPS — invalid — so a position sent
     * before the first GPS_RAW_INT arrives is not mistaken for a good fix.
     * Written and read only on the recv thread (processMavLinkMsg), so it
     * needs no lock, same as goto_altitude_m above. */
    uint8_t gps_fix_type{ GPS_FIX_TYPE_NO_GPS };
    std::optional<MavModeCommand> pending_mode_command{};
    /* Altitude (metres AGL, relative to home) a goto waypoint is flown at;
     * supplied from config, already clamped to [floor, cap]. Set once at
     * construction and only read on the recv thread when building the goto
     * mission item, so it needs no locking. */
    uint16_t goto_altitude_m;
    /* Regulatory altitude bounds (metres AGL), supplied from config and used to
     * clamp a direct FSS altitude command in commandAltitude(). Set once at
     * construction; read on the command path only, so they need no locking. */
    uint16_t altitude_floor_m;
    uint16_t altitude_cap_m;
    /* Intervals (microseconds) the autopilot is asked to stream position and
     * battery at, supplied from config. Set once at construction and read only on
     * the recv thread when requesting the streams, so they need no locking. */
    uint32_t position_stream_interval_us;
    uint32_t battery_stream_interval_us;
    /* Bound (milliseconds) on how long connect_to_mav() waits for a TCP
     * connect to complete before giving up (todo/84). Set once at
     * construction; read only on whichever thread calls connect_to_mav()
     * (the main thread via start(), or the FSS reconnector thread via
     * attemptReconnect()), so it needs no locking — same pattern as
     * goto_altitude_m above. */
    uint32_t connect_timeout_ms;
    /* Bound (milliseconds) applied as SO_SNDTIMEO and (where available)
     * TCP_USER_TIMEOUT on a freshly connected socket, so a peer that stops
     * reading, or a network path that silently disappears, cannot block a
     * send indefinitely (todo/84). Set once at construction; read only
     * inside connect_to_mav() when configuring the socket, so it needs no
     * locking. */
    uint32_t send_timeout_ms;
    /* Selected --terminate-action, needed to decide which failsafe params are
     * worth sanity-checking (todo/91). Set once at construction; read only on
     * the recv thread (checkFailsafeConfig/checkFailsafeParamReply), so it
     * needs no locking — same pattern as goto_altitude_m above. */
    terminate_action term_action;
    ILogger &logger;
    auto sendMavLinkMsgLocked (mavlink_message_t *msg) -> bool;
    auto sendMavLinkMsg (mavlink_message_t *msg) -> bool;
    /* Returns whether the SET_MODE was actually transmitted to the autopilot
     * (false when the link is down so the send was skipped). */
    auto setFlightMode (uint8_t fmode) -> bool;
    /* Switch to a resolved flight mode that drops out of an active search (RTL,
     * hold, manual): set the mode and clear the loaded-search flag. An empty
     * fmode means the airframe-specific mode could not be resolved — the
     * autopilot type is not yet known (no heartbeat) — so warn (via
     * warnUnresolvedMode) and do nothing rather than silently dropping the
     * command. The mode is carried as an optional, not a 0 sentinel, because 0
     * is itself a valid mode (e.g. COPTER_MODE_STABILIZE, ROVER_MODE_MANUAL).
     * `command` names the command for the log. Returns whether the mode was
     * actually transmitted now: false when it was deferred (autopilot type not
     * yet known) or the link was down, true once the SET_MODE went out. */
    auto setResolvedMode (MavModeCommand command, bool clear_search_loaded) -> bool;
    void replayPendingMode (uint8_t autopilot_type);
    /* Log that `command` arrived before the autopilot type was known, so the
     * airframe-specific flight mode could not be resolved. Shared by every
     * command path so the wording stays identical. */
    void warnUnresolvedMode (MavModeCommand command);
    /* Clear goto/search upload state so a MISSION_ACK for an upload that is
     * no longer allowed to complete — because a newer safety-critical mode
     * (RTL, hold, manual, disarm, force-disarm, terminate) just took control
     * mid-upload — cannot select a mission item or command AUTO (todo/82).
     * Also drops a search that never reached search_loaded, so a lingering
     * MISSION_REQUEST for the abandoned upload's sequence numbers cannot be
     * served from it. Must be called with state_lock already held. Returns
     * whether an upload was genuinely in flight (its ack still pending), so
     * the caller can log it once state_lock is released. */
    auto invalidateInFlightUploadLocked () -> bool;
    /* Log that `action_name` invalidated an in-flight goto/search mission
     * upload (todo/82). Shared by every command that can take control
     * mid-upload so the wording stays identical. Must be called without
     * state_lock held. */
    void logUploadInvalidated (const std::string &action_name);
    /* Whether `msg` originates from the configured autopilot system
     * (TARGET_SYS_ID). Flight-critical message handling — heartbeat,
     * position, GPS fix, battery, mission progress/requests/acks — must
     * gate on this so any other system sharing this MAVLink link (a GCS, a
     * companion computer, or a second vehicle) cannot influence FMU state
     * (todo/83). Non-matching traffic is expected on a shared link, not
     * anomalous, so a rejection is logged at debug rather than warn/error. */
    auto isFromAutopilot (const mavlink_message_t *msg) -> bool;
    void processMavLinkMsg (mavlink_message_t *msg, mavlink_status_t *status);
    /* Issue a PARAM_REQUEST_READ for each param expected_failsafe_params()
     * returns for `autopilot_type`/this->term_action (todo/91). Called once,
     * on the first heartbeat that resolves the autopilot type (see
     * mav_sys::checkedFailsafe()/markFailsafeChecked()). Read-only: never
     * writes an autopilot param, only requests and later compares. */
    void checkFailsafeConfig (uint8_t autopilot_type);
    /* Match a PARAM_VALUE reply's param_id against the current
     * expected_failsafe_params() list; log a loud warning if it is one of
     * them and reads 0 (todo/91). Silent on no match or a nonzero value. */
    void checkFailsafeParamReply (const std::string &param_id, float value);
    /* Attempt a bounded TCP connect: puts `sock` in non-blocking mode, calls
     * connect(), and if it does not complete immediately, poll()s for
     * POLLOUT up to connect_timeout_ms before giving up. Returns true once
     * the socket is confirmed connected (via SO_ERROR); false on any
     * failure or timeout, having logged a distinguishing message either way
     * (todo/84). Restores `sock` to blocking mode before returning true,
     * since the rest of the connection relies on blocking I/O bounded by
     * SO_RCVTIMEO/SO_SNDTIMEO, not O_NONBLOCK semantics; on false, `sock` is
     * left as-is — the caller always closes it immediately. */
    auto connectWithTimeout (int sock, const struct sockaddr *remote, socklen_t remote_len) -> bool;
    void connect_to_mav ();
    void disconnect_from_mav ();
    void report_position (double t_lat, double t_lng, double alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver,
                          bool fix_valid);
    void report_battery_status (int8_t, int32_t, double);
    void report_reached (int);
    void send_waypoint (uint16_t, uint8_t);
    void mission_ack (bool);
    void setCurrentWP (uint16_t seq);
    void sendHeartBeat ();
    void heartbeat_loop ();

  public:
    mav_connection (std::string t_addr, uint16_t t_port, const MavParams &t_params, ILogger &t_logger,
                    terminate_action t_term_action = terminate_action::none);
    ~mav_connection ();
    mav_connection (mav_connection &) = delete;
    mav_connection (mav_connection &&) = delete;
    auto operator= (mav_connection &) -> mav_connection & = delete;
    auto operator= (mav_connection &&) -> mav_connection & = delete;
    /* Open the connection and start the recv/heartbeat threads. Must be called
     * once, after the callbacks have been registered, to avoid racing those
     * threads against callback registration. */
    void start ();
    void attemptReconnect ();
    void processMessages ();
    /* The command methods used by the state machine's action step return whether
     * the command was transmitted to the autopilot (false when the link is down,
     * or for a mode command, when it was deferred because the autopilot type is
     * not yet known). The state machine uses this to replay a safety-critical
     * action once the MAV link recovers (todo/46). */
    auto commandRTL () -> bool;
    auto commandGoto (Point p) -> bool;
    auto commandHold () -> bool;
    void commandAuto ();
    auto commandAltitude (uint32_t alt) -> bool;
    auto commandDisARM () -> bool;
    auto commandForceDisARM () -> bool;
    auto commandManual () -> bool;
    auto commandTerminate () -> bool;
    auto loadSearch () -> bool;
    void sendADSB (uint32_t icao_address, double lat, double lng, double altitude_m, uint8_t altitude_type,
                   uint16_t heading, uint16_t hor_vel, uint16_t ver_vel, char *callsign, uint8_t emitter_type,
                   uint8_t tslc, uint16_t flags, uint16_t squawk);
    void requestStream (int sysid, int compid, uint32_t command, uint32_t interval);
    auto
    getLastPosition () -> Point
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        return this->last_position;
    };
    void loadSearch (const std::shared_ptr<SMMSearch> &search);
    void registerPositionCB (notify_position_cb cb);
    void registerReachedCB (notify_reached_cb cb);
    void registerBatteryCB (notify_battery_status_cb cb);
    void registerMavCommsStatusCB (notify_mav_comms_cb cb);
};
