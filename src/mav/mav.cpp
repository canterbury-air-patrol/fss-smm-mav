#include "mav.hpp"
#include "internal.hpp"

bool MAV::setMode(flight_mode fm)
{
    bool res = false;
    switch (fm)
    {
        case flight_mode_manual:
            /* Drop out of Auto/RTL mode, probably for FBW-B */
            this->connection->commandManual();
            break;
        case flight_mode_goto:
            /* Load a track with a single waypoint + RTL */
            this->connection->commandGoto(this->goto_position.getLatitude(), this->goto_position.getLongitude());
            break;
        case flight_mode_hold:
            /* Circle or similar */
            this->connection->commandHold();
            break;
        case flight_mode_search:
            /* Load the search */
            this->connection->loadSearch();
            break;
        case flight_mode_unknown:
        case flight_mode_rtl:
            /* Enter RTL */
            this->connection->commandRTL();
            break;
    }
    return res;
}

void MAV::disarm()
{
    this->connection->commandDisARM();
}

void MAV::terminate()
{
    this->connection->commandTerminate();
}

void MAV::gotoPosition(Point to)
{
    this->goto_position = to;
}

void MAV::setAltitude(uint16_t alt)
{
    this->target_altitude = alt;
}

MAV::MAV(std::string t_addr, uint16_t t_port) : connection(new mav_connection(t_addr, t_port))
{
}