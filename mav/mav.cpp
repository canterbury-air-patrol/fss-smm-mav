#include "mav.hpp"

bool MAV::setMode(flight_mode fm)
{
    bool res = false;
    switch (fm)
    {
        case flight_mode_manual:
            /* Drop out of Auto/RTL mode, probably for FBW-B */
            break;
        case flight_mode_goto:
            /* Load a track with a single waypoint + RTL */
            break;
        case flight_mode_hold:
            /* Circle or similar */
            break;
        case flight_mode_search:
            /* Load the search */
            break;
        case flight_mode_unknown:
        case flight_mode_rtl:
            /* Enter RTL */
            break;
    }
    return res;
}

void MAV::disarm()
{

}

void MAV::terminate()
{

}

void MAV::gotoPosition(Point to)
{
    this->goto_position = to;
}

void MAV::setAltitude(uint16_t alt)
{
    this->target_altitude = alt;
}

MAV::MAV(std::string t_addr, uint16_t t_port)
{

}