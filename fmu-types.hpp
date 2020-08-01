#pragma once

#include <bits/stdint-uintn.h>
#include <bits/stdint-intn.h>

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

typedef void (*notify_position_cb)(double t_lat, double t_lng, double alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver);
typedef void (*notify_battery_status_cb)(int8_t remaining, int32_t consumed);
typedef void (*notify_reached_cb)(int point);