#include <cstring>
#include <iostream>

#include "mav.hpp"
#include "internal.hpp"

void
MAV::setMode(flight_mode fm)
{
    switch (fm)
    {
        case flight_mode_manual:
            /* Drop out of Auto/RTL mode, probably for FBW-B */
            this->connection->commandManual();
            break;
        case flight_mode_goto:
            /* Load a track with a single waypoint + RTL, skip if unchanged */
            if (!(this->goto_position == this->goto_position_sent))
            {
                this->connection->commandGoto(this->goto_position);
                this->goto_position_sent = this->goto_position;
            }
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
}

void MAV::disarm()
{
    this->connection->commandDisARM();
}

void MAV::terminate()
{
    switch (this->action)
    {
        case terminate_action::terminate:
            this->connection->commandTerminate();
            break;
        case terminate_action::disarm:
            this->connection->commandForceDisARM();
            break;
        case terminate_action::none:
            std::cerr << "WARN: terminate-action is none, falling through to RTL\n";
            this->connection->commandRTL();
            break;
    }
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
    char callsign[9] = {0};
    std::strncpy(callsign, pd.getCallSign().c_str(), 8);
    this->connection->sendADSB(pd.getICAOAddress(), pd.getP().getLatitude(), pd.getP().getLongitude(), static_cast<uint16_t>(pd.getAltitude()), pd.getAltitudeType(), pd.getHeading(), pd.getVelocityHorizontal(), pd.getVelocityVertical(), callsign, pd.getEmitterType(), 0, pd.getFlags(), pd.getSquawk());
}

MAV::MAV(std::string t_addr, uint16_t t_port, terminate_action ta) : connection(std::make_shared<mav_connection>(std::move(t_addr), t_port)), action(ta)
{
}

auto
MAV::getCurrentPosition() -> Point
{
    return this->connection->getLastPosition();
}

void MAV::loadSearch(const std::shared_ptr<SMMSearch> &search)
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

void MAV::registerMavCommsStatusCB(notify_mav_comms_cb cb)
{
    this->connection->registerMavCommsStatusCB(cb);
}