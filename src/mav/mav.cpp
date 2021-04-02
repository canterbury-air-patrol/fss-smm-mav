#include <cstring>

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
            this->connection->commandGoto(this->goto_position);
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

void MAV::attemptReconnect()
{
    this->connection->attemptReconnect();
}

void MAV::gotoPosition(Point to)
{
    this->goto_position = to;
}

void MAV::setAltitude(uint16_t alt)
{
    this->target_altitude = alt;
}

void MAV::sendADSB(PositionData pd)
{
    char *callsign = (char *)malloc(9);
    strncpy(callsign, pd.getCallSign().c_str(), 8);
    callsign[8] = '\0';
    this->connection->sendADSB(pd.getICAOAddress(), pd.getP().getLatitude(), pd.getP().getLongitude(), pd.getAltitude(), pd.getAltitudeType(), pd.getHeading(), pd.getVelocityHorizontal(), pd.getVelocityVertical(), callsign, pd.getEmitterType(), 0, pd.getFlags(), pd.getSquawk());
    free(callsign);
}

MAV::MAV(std::string t_addr, uint16_t t_port) : connection(new mav_connection(t_addr, t_port))
{
}

MAV::~MAV()
{
    delete this->connection;
    this->connection = nullptr;
}

Point MAV::getCurrentPosition()
{
    return this->connection->getLastPosition();
}

void MAV::loadSearch(SMMSearch *search)
{
    this->connection->loadSearch(search);
}

void MAV::registerPositionCB(notify_position_cb cb)
{
    this->connection->registerPositionCB(cb);
}

void MAV::registerReachedCB(notify_reached_cb cb)
{
    this->connection->registerReachedCB(cb);
}

void MAV::registerBatteryCB(notify_battery_status_cb cb)
{
    this->connection->registerBatteryCB(cb);
}