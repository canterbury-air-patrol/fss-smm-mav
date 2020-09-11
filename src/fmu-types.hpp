#pragma once

#include <stdint.h>

class Point {
private:
    bool valid{false};
    double latitude{0.0};
    double longitude{0.0};
public:
    Point() {};
    Point(double lat, double lng) : valid(true), latitude(lat), longitude(lng) {};
    double getLatitude() { return latitude; };
    double getLongitude() { return longitude; };
};

class PositionData {
private:
    Point p{};
    double alt{0.0};
    uint16_t hdg{0};
    uint16_t vel_hor{0};
    int16_t vel_ver{0};
public:
    PositionData() {};
    PositionData(double t_lat, double t_lng, double t_alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver) : p(Point(t_lat, t_lng)), alt(t_alt), hdg(t_hdg), vel_hor(t_vel_hor), vel_ver(t_vel_ver) {};
    Point getP() { return this->p; };
    double getAltitude() { return this->alt; };
    uint16_t getHeading() { return this->hdg; };
    uint16_t getVelocityHorizontal() { return this->vel_hor; };
    int16_t getVelocityVertical() { return this->vel_ver; };
};

class BatteryData {
private:
    int8_t remaining{-1};
    int32_t consumed{-1};
public:
    BatteryData() {};
    BatteryData(int8_t t_remaining, int32_t t_consumed) : remaining(t_remaining), consumed(t_consumed) {};
    int8_t getRemaining() { return this->remaining; };
    int32_t getConsumed() { return this->consumed; };
};

#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include <bits/stdint-uintn.h>
#include <bits/stdint-intn.h>

typedef void (*notify_fss_command_cb)(void *priv, FSSCommand cmd);
typedef void (*notify_fss_comms_cb)(void *priv, FSSCommsStatus status);

typedef void (*notify_position_cb)(void *priv, PositionData pd);
typedef void (*notify_battery_status_cb)(void *priv, BatteryData bd);
typedef void (*notify_reached_cb)(void *priv, int point);

typedef void (*notify_smm_settings_cb)(void *priv, SMMSettings settings);