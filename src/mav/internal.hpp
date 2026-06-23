#pragma once
#include <ardupilotmega/mavlink.h>
#include <atomic>
#include <condition_variable>
#include <list>
#include <mutex>
#include <string>
#include <thread>

#include "../fmu-types.hpp"
#include "../smm/smm.hpp"
#include "../util.hpp"

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

/* The autopilot_type/flight_mode/setup scalars are written on the recv thread
 * (via processMavLinkMsg) and read on the event-loop thread (getAutoPilotType()
 * in commandRTL/Hold/Manual/Auto), so they are atomic to make that cross-thread
 * access well-defined. The components list is only ever touched on the recv
 * thread (findComponent from processMavLinkMsg), so it needs no lock. */
class mav_sys
{
  private:
    uint8_t sysid;
    std::list<std::shared_ptr<mav_comp>> components{};
    std::atomic<uint8_t> autopilot_type{ 0 };
    std::atomic<uint8_t> flight_mode{ 0 };
    std::atomic<bool> setup{ false };

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
    std::atomic<int> fd{ -1 };
    uint64_t last_tried{ 0 };
    uint16_t retry_count{ 0 };
    std::mutex send_lock{};
    std::thread recv_thread{};
    std::thread heartbeat_thread{};
    std::atomic<bool> broken{ false };
    std::atomic<bool> stopping{ false };
    std::atomic<bool> started{ false };
    std::mutex heartbeat_mutex{};
    std::condition_variable heartbeat_cv{};
    std::atomic<uint64_t> last_heartbeat_ts{ 0 };
    std::atomic<bool> mav_comms_ok{ false };
    notify_mav_comms_cb mav_comms_cb{};
    mav_systems systems{};
    /* state_lock guards: last_position, search, search_loaded,
     * search_loading, goto_active, goto_position, retry_count, last_tried. */
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
    auto sendMavLinkMsg (mavlink_message_t *msg) -> bool;
    void setFlightMode (uint8_t fmode);
    /* Switch to a resolved flight mode that drops out of an active search (RTL,
     * hold, manual): set the mode and clear the loaded-search flag. fmode == 0
     * means the airframe-specific mode could not be resolved — the autopilot
     * type is not yet known (no heartbeat) — so warn and do nothing rather than
     * silently dropping the command. `command` names the command for the log. */
    void setSearchExitMode (uint8_t fmode, const char *command);
    void processMavLinkMsg (mavlink_message_t *msg, mavlink_status_t *status);
    void connect_to_mav ();
    void disconnect_from_mav ();
    void report_position (double t_lat, double t_lng, double alt, uint16_t t_hdg, uint16_t t_vel_hor,
                          int16_t t_vel_ver);
    void report_battery_status (int8_t, int32_t, double);
    void report_reached (int);
    void send_waypoint (uint16_t, uint8_t);
    void mission_ack (bool);
    void setCurrentWP (uint16_t seq);
    void sendHeartBeat ();
    void heartbeat_loop ();

  public:
    mav_connection (std::string t_addr, uint16_t t_port, uint16_t t_goto_altitude_m, uint16_t t_altitude_floor_m,
                    uint16_t t_altitude_cap_m);
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
    void commandRTL ();
    void commandGoto (Point p);
    void commandHold ();
    void commandAuto ();
    void commandAltitude (uint16_t alt);
    void commandDisARM ();
    void commandForceDisARM ();
    void commandManual ();
    void commandTerminate ();
    void loadSearch ();
    void sendADSB (uint32_t icao_address, double lat, double lng, uint32_t altitude, uint8_t altitude_type,
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