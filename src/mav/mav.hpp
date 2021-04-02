#pragma once
#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include <bits/stdint-uintn.h>

#include <string>

enum flight_mode {
    flight_mode_unknown,
    flight_mode_manual,
    flight_mode_search,
    flight_mode_rtl,
    flight_mode_goto,
    flight_mode_hold,
};

class mav_connection;

class MAV {
private:
    mav_connection *connection;
    flight_mode mode{flight_mode_unknown};
    bool armed{false};
    Point goto_position{};
    uint16_t target_altitude{0};
public:
    MAV(std::string t_addr, uint16_t t_port);
    ~MAV();
    flight_mode getFlightMode() { return this->mode; };
    bool getArmed() { return this->armed; };
    void attemptReconnect();
    bool setMode(flight_mode fm);
    void disarm();
    void terminate();
    void gotoPosition(Point to);
    void setAltitude(uint16_t alt);
    Point getCurrentPosition();
    void loadSearch(SMMSearch *);
    void sendADSB(PositionData pd);
    void registerPositionCB(notify_position_cb cb);
    void registerReachedCB(notify_reached_cb);
    void registerBatteryCB(notify_battery_status_cb cb);
};