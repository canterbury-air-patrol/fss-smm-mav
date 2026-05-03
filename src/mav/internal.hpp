#pragma once
#include <ardupilotmega/mavlink.h>
#include <condition_variable>
#include <string>
#include <thread>
#include <mutex>
#include <list>
#include <atomic>

#include "../fmu-types.hpp"
#include "../smm/smm.hpp"

class mav_comp {
    private:
        uint8_t compid;
    public:
        explicit mav_comp(uint8_t t_compid) : compid(t_compid) {};
        auto getCompId() -> uint8_t { return this->compid; };
};

/* Single-writer invariant: mav_sys/mav_systems members are mutated only on the
 * recv thread (via processMavLinkMsg). Main-thread reads of getAutoPilotType()
 * in commandRTL/Hold/Manual/Auto rely on this — if mutation ever moves off the
 * recv thread, add synchronisation here. */
class mav_sys {
    private:
        uint8_t sysid;
        std::list<std::shared_ptr<mav_comp>> components{};
        uint8_t autopilot_type;
        uint8_t flight_mode;
        bool setup{false};
    public:
        explicit mav_sys(uint8_t t_sysid) : sysid(t_sysid), autopilot_type(0), flight_mode(0) {};
        ~mav_sys() = default;
        auto getSysId() -> uint8_t { return this->sysid; };
        auto getAutoPilotType() -> uint8_t { return this->autopilot_type; };
        auto getFlightMode() -> uint8_t { return this->flight_mode; };
        void setAutoPilotMode(uint8_t type) { this->autopilot_type = type; };
        void setFlightMode(uint8_t mode) { this->flight_mode = mode; };
        auto findComponent(uint8_t compid) -> std::shared_ptr<mav_comp>;
        auto isSetup() -> bool { return this->setup; };
        void setupComplete() { this->setup = true; };
};

class mav_systems {
    private:
        std::list<std::shared_ptr<mav_sys>> systems{};
    public:
        mav_systems() = default;
        ~mav_systems() = default;
        auto findSystem(uint8_t t_sysid) -> std::shared_ptr<mav_sys>;
};

class mav_connection {
    private:
        std::string addr;
        uint16_t port;
        std::atomic<int> fd{-1};
        uint64_t last_tried{0};
        uint16_t retry_count{0};
        std::mutex send_lock{};
        std::thread recv_thread{};
        std::thread heartbeat_thread{};
        std::atomic<bool> broken{false};
        std::atomic<bool> stopping{false};
        std::mutex heartbeat_mutex{};
        std::condition_variable heartbeat_cv{};
        mav_systems systems{};
        /* state_lock guards: last_position, search, search_loaded,
         * search_loading, goto_active, goto_position, retry_count, last_tried. */
        std::mutex state_lock{};
        Point last_position{};
        std::shared_ptr<SMMSearch> search{nullptr};
        notify_position_cb position_cb{};
        notify_battery_status_cb battery_cb{};
        notify_reached_cb reached_cb{};
        bool search_loading{false};
        bool search_loaded{false};
        Point goto_position{};
        bool goto_active{false};
        auto sendMavLinkMsg(mavlink_message_t *msg) -> bool;
        void setFlightMode(uint8_t fmode);
        void processMavLinkMsg(mavlink_message_t *msg, mavlink_status_t *status);
        void connect_to_mav();
        void disconnect_from_mav();
        void report_position(double t_lat, double t_lng, double alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver);
        void report_battery_status(int8_t, int32_t, double);
        void report_reached(int);
        void send_waypoint(uint16_t, uint8_t);
        void mission_ack(bool);
        void setCurrentWP(uint16_t seq);
        void sendHeartBeat();
        void heartbeat_loop();
    public:
        mav_connection(std::string t_addr, uint16_t t_port);
        ~mav_connection();
        mav_connection(mav_connection&) = delete;
        mav_connection(mav_connection&&) = delete;
        auto operator=(mav_connection&) -> mav_connection& = delete;
        auto operator=(mav_connection&&) -> mav_connection& = delete;
        void attemptReconnect();
        void processMessages();
        void commandRTL();
        void commandGoto(Point p);
        void commandHold();
        void commandAuto();
        void commandAltitute();
        void commandDisARM();
        void commandForceDisARM();
        void commandManual();
        void commandTerminate();
        void loadSearch();
        void sendADSB(uint32_t icao_address, double lat, double lng, uint32_t altitude, uint8_t altitude_type, uint16_t heading, uint16_t hor_vel, uint16_t ver_vel, char *callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk);
        void requestStream(int sysid, int compid, uint32_t command, uint32_t interval);
        auto getLastPosition() -> Point { std::lock_guard<std::mutex> lk{this->state_lock}; return this->last_position; };
        void loadSearch(const std::shared_ptr<SMMSearch> &search);
        void registerPositionCB(notify_position_cb cb);
        void registerReachedCB(notify_reached_cb cb);
        void registerBatteryCB(notify_battery_status_cb cb);
};