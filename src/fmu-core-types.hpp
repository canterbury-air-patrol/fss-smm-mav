#pragma once

#include <cmath>
#include <cstdint>
#include <string>

class Point
{
  private:
    double latitude{ 0.0 };
    double longitude{ 0.0 };

  public:
    Point () = default;
    Point (double lat, double lng) : latitude (lat), longitude (lng) {};
    auto
    getLatitude () -> double
    {
        return latitude;
    };
    auto
    getLongitude () -> double
    {
        return longitude;
    };
    auto
    operator== (const Point &other) const -> bool
    {
        return latitude == other.latitude && longitude == other.longitude;
    };
    /* Shared, pure coordinate validator (todo/85): every external source of a
     * commanded/reported position (FSS goto, SMM search waypoints, ADS-B
     * peer reports) must pass this before the coordinates reach
     * degrees_to_degE7(), whose float-to-int32 cast is undefined behavior on
     * a non-finite or out-of-range input. */
    auto
    isValid () const -> bool
    {
        return std::isfinite (latitude) && latitude >= -90.0 && latitude <= 90.0 && std::isfinite (longitude)
               && longitude >= -180.0 && longitude <= 180.0;
    };
};

/* Bit in PositionData's/the FSS wire message's flags word marking the lat/lon
 * as backed by a valid GPS fix (GPS_FIX_TYPE_2D_FIX or better), not a lost/
 * no-fix estimate. Same numbering as MAVLink's ADSB_FLAGS_VALID_COORDS and
 * fss_client_ssl::sendPosition's hardcoded valid_fields bit 1 (todo/79). */
constexpr uint16_t POSITION_FLAG_VALID_COORDS = 1;

class PositionData
{
  private:
    Point p{};
    /* Altitude in metres. Each protocol boundary converts to/from its own wire
     * unit via altitude-units.hpp (MAVLink mm, FSS feet, SMM metres). MSL
     * (mean sea level) — MAVLink GLOBAL_POSITION_INT's `alt` field. NOT the
     * same frame as altitude_cap_m/altitude_floor_m/goto_altitude_m, which are
     * all AGL; see alt_agl_m below and todo/99. */
    double alt_m{ 0.0 };
    /* AGL (above ground level, relative to home) altitude in metres —
     * MAVLink GLOBAL_POSITION_INT's `relative_alt` field, the same frame
     * altitude_cap_m uses. Only the FMU's own-aircraft report populates this
     * (mav_connection::report_position); ADS-B peers and any other source
     * leave it at 0.0, which is harmless since those paths never set
     * POSITION_FLAG_VALID_COORDS from a real reading either, and the only
     * consumer (the altitude-cap breach check, todo/92) requires a valid fix
     * before trusting it. */
    double alt_agl_m{ 0.0 };
    uint16_t hdg{ 0 };
    uint16_t vel_hor{ 0 };
    int16_t vel_ver{ 0 };
    std::string callsign{};
    uint16_t squawk{ 0 };
    uint32_t icaoaddress{ 0 };
    uint64_t timestamp{ 0 };
    uint16_t flags{ 0 };
    uint8_t altitude_type{ 0 };
    uint8_t emitter_type{ 0 };

  public:
    PositionData () = default;
    /* t_alt_m is altitude in metres (see alt_m above). */
    PositionData (double t_lat, double t_lng, double t_alt_m, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver)
        : p (Point (t_lat, t_lng)), alt_m (t_alt_m), hdg (t_hdg), vel_hor (t_vel_hor), vel_ver (t_vel_ver) {};
    /* Own-aircraft report with an explicit validity flags word (todo/79); ADSB
     * peers carry callsign/squawk/etc too and use the constructor below instead. */
    PositionData (double t_lat, double t_lng, double t_alt_m, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver,
                  uint16_t t_flags, double t_alt_agl_m = 0.0)
        : p (Point (t_lat, t_lng)), alt_m (t_alt_m), alt_agl_m (t_alt_agl_m), hdg (t_hdg), vel_hor (t_vel_hor),
          vel_ver (t_vel_ver), flags (t_flags) {};
    PositionData (double t_lat, double t_lng, double t_alt_m, uint16_t t_hdg, uint16_t t_vel_hor, int16_t t_vel_ver,
                  std::string t_callsign, uint16_t t_squawk, uint32_t t_icaoaddress, uint64_t t_timestamp,
                  uint16_t t_flags, uint8_t t_altitude_type, uint8_t t_emitter_type)
        : p (Point (t_lat, t_lng)), alt_m (t_alt_m), hdg (t_hdg), vel_hor (t_vel_hor), vel_ver (t_vel_ver),
          callsign (std::move (t_callsign)), squawk (t_squawk), icaoaddress (t_icaoaddress), timestamp (t_timestamp),
          flags (t_flags), altitude_type (t_altitude_type), emitter_type (t_emitter_type) {};
    void
    setICAOAddress (uint32_t t_icaoaddress)
    {
        this->icaoaddress = t_icaoaddress;
    };
    auto
    getP () -> Point
    {
        return this->p;
    };
    auto
    getAltitudeMetres () -> double
    {
        return this->alt_m;
    };
    auto
    getAltitudeAGLMetres () const -> double
    {
        return this->alt_agl_m;
    };
    auto
    getHeading () -> uint16_t
    {
        return this->hdg;
    };
    auto
    getVelocityHorizontal () -> uint16_t
    {
        return this->vel_hor;
    };
    auto
    getVelocityVertical () -> int16_t
    {
        return this->vel_ver;
    };
    auto
    getCallSign () -> std::string
    {
        return this->callsign;
    };
    auto
    getICAOAddress () -> uint32_t
    {
        return this->icaoaddress;
    };
    auto
    getTimeStamp () -> uint64_t
    {
        return this->timestamp;
    };
    auto
    getSquawk () -> uint16_t
    {
        return this->squawk;
    };
    auto
    getFlags () const -> uint16_t
    {
        return this->flags;
    };
    auto
    getAltitudeType () -> uint8_t
    {
        return this->altitude_type;
    };
    auto
    getEmitterType () -> uint8_t
    {
        return this->emitter_type;
    };
};

class BatteryData
{
  private:
    int8_t remaining{ -1 };
    int32_t consumed{ -1 };
    double voltage{ 0.0 };

  public:
    BatteryData () = default;
    BatteryData (int8_t t_remaining, int32_t t_consumed, double t_voltage)
        : remaining (t_remaining), consumed (t_consumed), voltage (t_voltage) {};
    auto
    getRemaining () -> int8_t
    {
        return this->remaining;
    };
    auto
    getConsumed () -> int32_t
    {
        return this->consumed;
    };
    auto
    getVoltage () -> double
    {
        return this->voltage;
    };
};

enum class MavCommsStatus
{
    ok,
    failure,
};

/* Logging verbosity, ordered least- to most-verbose. A message logged at
 * level L is emitted only when L <= the configured level, so `error` shows
 * only errors, `warning` adds degraded-but-handled conditions, `info` adds
 * normal operation, and `debug` shows everything.
 *
 * The level is the *only* place a line's severity is stated (todo/109): a
 * message must not carry its own "WARN:"/"ERROR:" prefix, because the two
 * then drift and the prefix wins the reader's eye while the enum is what
 * actually decides whether the line is emitted at all. Logger::log() renders
 * the level into the line, so nothing is lost by leaving it out of the text.
 * `warning` exists so that rule can be followed: before it, every
 * degraded-but-handled condition had to be logged at `error` and say "WARN:"
 * to be honest about what it was. */
enum class LogLevel
{
    error,
    warning,
    info,
    debug,
};
