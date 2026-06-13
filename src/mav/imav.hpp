#pragma once

#include "../fmu-core-types.hpp"
#include "../fmu-types.hpp"
#include <cstdint>

enum flight_mode
{
    flight_mode_unknown,
    flight_mode_manual,
    flight_mode_search,
    flight_mode_rtl,
    flight_mode_goto,
    flight_mode_hold,
};

class IMAV
{
  public:
    IMAV () = default;
    IMAV (const IMAV &) = delete;
    IMAV (IMAV &&) = delete;
    auto operator= (const IMAV &) -> IMAV & = delete;
    auto operator= (IMAV &&) -> IMAV & = delete;
    virtual ~IMAV () = default;

    virtual void setMode (flight_mode fm) = 0;
    virtual void disarm () = 0;
    virtual void terminate () = 0;
    virtual void gotoPosition (Point to) = 0;
    virtual void setAltitude (uint16_t alt) = 0;
    virtual auto getCurrentPosition () -> Point = 0;
    virtual void registerMavCommsStatusCB (notify_mav_comms_cb cb) = 0;
};
