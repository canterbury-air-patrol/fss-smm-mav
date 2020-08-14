#include <ardupilotmega/mavlink.h>
#include <string>
#include <thread>
#include <mutex>
#include <list>

#include "../fmu-types.hpp"
#include "../smm/smm.hpp"

class mav_comp {
    private:
        uint8_t compid;
    public:
        mav_comp(uint8_t t_compid) : compid(t_compid) {};
        uint8_t getCompId() { return this->compid; };
};

class mav_sys {
    private:
        uint8_t sysid;
        std::list<mav_comp *> components{};
        uint8_t autopilot_type;
        uint8_t flight_mode;
        bool setup{false};
    public:
        mav_sys(uint8_t t_sysid) : sysid(t_sysid), autopilot_type(0), flight_mode(0) {};
        uint8_t getSysId() { return this->sysid; };
        uint8_t getAutoPilotType() { return this->autopilot_type; };
        uint8_t getFlightMode() { return this->flight_mode; };
        void setAutoPilotMode(uint8_t type) { this->autopilot_type = type; };
        void setFlightMode(uint8_t mode) { this->flight_mode = mode; };
        mav_comp *findComponent(uint8_t compid);
        bool isSetup() { return this->setup; };
        void setupComplete() { this->setup = true; };
};

class mav_systems {
    private:
        std::list<mav_sys *> systems{};
    public:
        mav_systems() {}
        mav_sys *findSystem(uint8_t t_sysid);
};

class mav_connection {
    private:
        std::string addr;
        uint16_t port;
        int fd{-1};
        uint64_t last_tried{0};
        uint16_t retry_count{0};
        std::mutex send_lock{};
        std::thread recv_thread{};
        mav_systems systems{};
        Point last_position{};
        SMMSearch *search{nullptr};
        notify_position_cb position_cb{nullptr};
        void *position_cb_priv{nullptr};
        notify_battery_status_cb battery_cb{nullptr};
        void *battery_cb_priv{nullptr};
        notify_reached_cb reached_cb{nullptr};
        void *reached_cb_priv{nullptr};
        bool last_action_was_continue{false};
        bool search_loading{false};
        bool search_loaded{false};
        bool sendMavLinkMsg(mavlink_message_t *msg);
        void setFlightMode(uint8_t fmode);
        void processMavLinkMsg(mavlink_message_t *msg, mavlink_status_t *status);
        void connect_to_mav();
        void disconnect_from_mav();
        void report_position(double t_lat, double t_lng, double alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver);
        void report_battery_status(int8_t, int32_t);
        void report_reached(int);
        void send_waypoint(uint16_t, uint8_t);
        void mission_ack(bool);
        void setCurrentWP(uint16_t seq);
    public:
        mav_connection(std::string t_addr, uint16_t t_port);
        ~mav_connection();
        void attemptReconnect();
        void processMessages();
        void sendHeartBeat();
        void commandRTL();
        void commandGoto(double lat, double lng);
        void commandHold();
        void commandContinue();
        void commandAltitute();
        void commandDisARM();
        void commandManual();
        void commandTerminate();
        void loadSearch();
        void sendADSB(uint32_t icao_address, double lat, double lng, uint32_t altitude, uint8_t altitude_type, uint16_t heading, uint16_t hor_vel, uint16_t ver_vel, char *callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk);
        void requestStream(int sysid, int compid, uint32_t command, uint32_t interval);
        Point getLastPosition() { return this->last_position; };
        void loadSearch(SMMSearch *search);
        void registerPositionCB(notify_position_cb cb, void *priv);
};