#pragma once

#include <string>
#include <cstdint>

class Point {
private:
    double latitude{0.0};
    double longitude{0.0};
public:
    Point() = default;
    Point(double lat, double lng) : latitude(lat), longitude(lng) {};
    auto getLatitude() -> double { return latitude; };
    auto getLongitude() -> double { return longitude; };
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
    PositionData() = default;
    PositionData(double t_lat, double t_lng, double t_alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver) : p(Point(t_lat, t_lng)), alt(t_alt), hdg(t_hdg), vel_hor(t_vel_hor), vel_ver(t_vel_ver) {};
    PositionData(double t_lat, double t_lng, double t_alt, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver,
                 std::string t_callsign, uint16_t t_squawk, uint32_t t_icaoaddress, uint64_t t_timestamp,
                 uint16_t t_flags, uint8_t t_altitude_type, uint8_t t_emitter_type) :
                    p(Point(t_lat, t_lng)), alt(t_alt), hdg(t_hdg), vel_hor(t_vel_hor), vel_ver(t_vel_ver),
                    callsign(std::move(t_callsign)), squawk(t_squawk), icaoaddress(t_icaoaddress), timestamp(t_timestamp),
                    flags(t_flags), altitude_type(t_altitude_type), emitter_type(t_emitter_type) {};
    void setICAOAddress(uint32_t t_icaoaddress) { this->icaoaddress = t_icaoaddress; };
    auto getP() -> Point { return this->p; };
    auto getAltitude() -> double { return this->alt; };
    auto getHeading() -> uint16_t { return this->hdg; };
    auto getVelocityHorizontal() -> uint16_t { return this->vel_hor; };
    auto getVelocityVertical() -> int16_t { return this->vel_ver; };
    auto getCallSign() -> std::string { return this->callsign; };
    auto getICAOAddress() -> uint32_t { return this->icaoaddress; };
    auto getTimeStamp() -> uint64_t { return this->timestamp; };
    auto getSquawk() -> uint16_t { return this->squawk; };
    auto getFlags() -> uint16_t { return this->flags; };
    auto getAltitudeType() -> uint8_t { return this->altitude_type; };
    auto getEmitterType() -> uint8_t { return this->emitter_type; };
};

class BatteryData {
private:
    int8_t remaining{-1};
    int32_t consumed{-1};
    double voltage{0.0};
public:
    BatteryData() = default;
    BatteryData(int8_t t_remaining, int32_t t_consumed, double t_voltage) : remaining(t_remaining), consumed(t_consumed), voltage(t_voltage) {};
    auto getRemaining() -> int8_t { return this->remaining; };
    auto getConsumed() -> int32_t { return this->consumed; };
    auto getVoltage() -> double { return this->voltage; };
};
