#pragma once

#include <string>
#include <stdint.h>

class Point {
private:
    double latitude{0.0};
    double longitude{0.0};
public:
    Point() {};
    Point(double lat, double lng) : latitude(lat), longitude(lng) {};
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
    std::string callsign{};
    uint16_t squawk{0};
    uint32_t icaoaddress{0};
    uint64_t timestamp{0};
    uint16_t flags{0};
    uint8_t altitude_type{0};
    uint8_t emitter_type{0};
public:
    PositionData() {};
    PositionData(double t_lat, double t_lng, double t_alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver) : p(Point(t_lat, t_lng)), alt(t_alt), hdg(t_hdg), vel_hor(t_vel_hor), vel_ver(t_vel_ver) {};
    PositionData(double t_lat, double t_lng, double t_alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver,
                 std::string t_callsign, uint16_t t_squawk, uint32_t t_icaoaddress, uint64_t t_timestamp,
                 uint16_t t_flags, uint8_t t_altitude_type, uint8_t t_emitter_type) :
                    p(Point(t_lat, t_lng)), alt(t_alt), hdg(t_hdg), vel_hor(t_vel_hor), vel_ver(t_vel_ver),
                    callsign(t_callsign), squawk(t_squawk), icaoaddress(t_icaoaddress), timestamp(t_timestamp),
                    flags(t_flags), altitude_type(t_altitude_type), emitter_type(t_emitter_type) {};
    void setICAOAddress(uint32_t t_icaoaddress) { this->icaoaddress = t_icaoaddress; };
    Point getP() { return this->p; };
    double getAltitude() { return this->alt; };
    uint16_t getHeading() { return this->hdg; };
    uint16_t getVelocityHorizontal() { return this->vel_hor; };
    int16_t getVelocityVertical() { return this->vel_ver; };
    std::string getCallSign() { return this->callsign; };
    uint32_t getICAOAddress() { return this->icaoaddress; };
    uint64_t getTimeStamp() { return this->timestamp; };
    uint16_t getSquawk() { return this->squawk; };
    uint16_t getFlags() { return this->flags; };
    uint8_t getAltitudeType() { return this->altitude_type; };
    uint8_t getEmitterType() { return this->emitter_type; };
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

typedef void (*notify_fss_command_cb)(FSSCommand cmd);
typedef void (*notify_fss_comms_cb)(FSSCommsStatus status);

typedef void (*notify_position_cb)(PositionData pd);
typedef void (*notify_battery_status_cb)(BatteryData bd);
typedef void (*notify_reached_cb)(int point);

typedef void (*notify_smm_settings_cb)(SMMSettings settings);