#include <cstring>

#include "internal.hpp"
#include "mav.hpp"

auto
MAV::setMode (flight_mode fm) -> bool
{
    switch (fm)
    {
        case flight_mode_manual:
            /* Drop out of Auto/RTL mode, probably for FBW-B */
            return this->connection->commandManual ();
        case flight_mode_goto:
            /* Load a track with a single waypoint + RTL.  setMode() is only
             * called by the state machine on an actual transition into goto,
             * so this must always re-send: a goto->hold->goto cycle with the
             * same waypoint still needs the track re-loaded to re-engage. */
            return this->connection->commandGoto (this->goto_position);
        case flight_mode_hold:
            /* Circle or similar */
            return this->connection->commandHold ();
        case flight_mode_search:
            /* Load the search */
            return this->connection->loadSearch ();
        case flight_mode_unknown:
        case flight_mode_rtl:
            /* Enter RTL */
            return this->connection->commandRTL ();
    }
    return false;
}

auto
MAV::disarm () -> bool
{
    return this->connection->commandDisARM ();
}

auto
MAV::terminate () -> bool
{
    switch (this->action)
    {
        case terminate_action::terminate:
            return this->connection->commandTerminate ();
        case terminate_action::disarm:
            return this->connection->commandForceDisARM ();
        case terminate_action::none:
            this->logger.log (LogLevel::error, "WARN: terminate-action is none, falling through to RTL");
            return this->connection->commandRTL ();
    }
    return false;
}

void
MAV::start ()
{
    this->connection->start ();
}

void
MAV::attemptReconnect ()
{
    this->connection->attemptReconnect ();
}

void
MAV::gotoPosition (Point to)
{
    this->goto_position = to;
}

auto
MAV::setAltitude (uint32_t alt) -> bool
{
    this->target_altitude = alt;
    return this->connection->commandAltitude (alt);
}

void
MAV::sendADSB (PositionData pd)
{
    char callsign[9] = { 0 };
    std::strncpy (callsign, pd.getCallSign ().c_str (), 8);
    this->connection->sendADSB (pd.getICAOAddress (), pd.getP ().getLatitude (), pd.getP ().getLongitude (),
                                pd.getAltitudeMetres (), pd.getAltitudeType (), pd.getHeading (),
                                pd.getVelocityHorizontal (), pd.getVelocityVertical (), callsign, pd.getEmitterType (),
                                0, pd.getFlags (), pd.getSquawk ());
}

MAV::MAV (std::string t_addr, uint16_t t_port, terminate_action ta, const MavParams &t_params, ILogger &t_logger)
    : connection (std::make_shared<mav_connection> (std::move (t_addr), t_port, t_params, t_logger, ta)), action (ta),
      logger (t_logger)
{
}

auto
MAV::getCurrentPosition () -> Point
{
    return this->connection->getLastPosition ();
}

void
MAV::loadSearch (const std::shared_ptr<SMMSearch> &search)
{
    this->connection->loadSearch (search);
}

void
MAV::registerPositionCB (notify_position_cb cb)
{
    this->connection->registerPositionCB (std::move (cb));
}

void
MAV::registerReachedCB (notify_reached_cb cb)
{
    this->connection->registerReachedCB (std::move (cb));
}

void
MAV::registerBatteryCB (notify_battery_status_cb cb)
{
    this->connection->registerBatteryCB (std::move (cb));
}

void
MAV::registerMavCommsStatusCB (notify_mav_comms_cb cb)
{
    this->connection->registerMavCommsStatusCB (std::move (cb));
}