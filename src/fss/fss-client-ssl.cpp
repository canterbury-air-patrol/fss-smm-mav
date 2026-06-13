#include "fmu-fss-types.hpp"
#include "internal.hpp"
#include <chrono>

void
fss_client_ssl::handleCommand (const std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> &msg)
{
    /* Don't execute commands older than the last one we handled
       This can happen when there are connections to multiple servers
       and the client that set the command didn't send it to all of them.

       If a server clock resets, we could drop all subsequent commands.
       We allow commands that are more than 60 seconds older than the last
       one to handle this case, while still deduplicating near-simultaneous
       messages. */
    static constexpr uint64_t command_dedup_tolerance_ms = 60000;
    if (last_command != nullptr && msg->getTimeStamp () < last_command->getTimeStamp ())
    {
        uint64_t diff = last_command->getTimeStamp () - msg->getTimeStamp ();
        if (diff < command_dedup_tolerance_ms)
        {
            return;
        }
    }
    last_command = msg;
    /* Convert Each Command into a MavLink Command */
    switch (msg->getCommand ())
    {
        case flight_safety_system::transport::asset_command_rtl:
            this->report_command (fss_cmd_rtl);
            break;
        case flight_safety_system::transport::asset_command_disarm:
            this->report_command (fss_cmd_disarm);
            break;
        case flight_safety_system::transport::asset_command_goto:
            this->report_goto_update (Point (msg->getLatitude (), msg->getLongitude ()));
            this->report_command (fss_cmd_goto);
            break;
        case flight_safety_system::transport::asset_command_altitude:
            this->report_altitude_update (msg->getAltitude ());
            this->report_command (fss_cmd_altitude);
            break;
        case flight_safety_system::transport::asset_command_hold:
            this->report_command (fss_cmd_hold);
            break;
        case flight_safety_system::transport::asset_command_resume:
            this->report_command (fss_cmd_continue);
            break;
        case flight_safety_system::transport::asset_command_terminate:
            this->report_command (fss_cmd_terminate);
            break;
        case flight_safety_system::transport::asset_command_manual:
            this->report_command (fss_cmd_manual);
            break;
        case flight_safety_system::transport::asset_command_unknown:
        default:
            break;
    }
}

void
fss_client_ssl::connectionStatusChange (flight_safety_system::client_ssl::connection_status status)
{
    switch (status)
    {
        case flight_safety_system::client_ssl::CLIENT_CONNECTION_STATUS_CONNECTED_1_SERVER:
        case flight_safety_system::client_ssl::CLIENT_CONNECTION_STATUS_CONNECTED_2_OR_MORE:
            /* No Need for RTL in these cases */
            this->report_comms_status (fss_comms_okay);
            break;
        case flight_safety_system::client_ssl::CLIENT_CONNECTION_STATUS_DISCONNECTED:
        case flight_safety_system::client_ssl::CLIENT_CONNECTION_STATUS_UNKNOWN:
            this->report_comms_status (fss_comms_failure);
            break;
    }
}

void
fss_client_ssl::handlePositionReport (
    const std::shared_ptr<flight_safety_system::transport::fss_message_position_report> &msg)
{
    this->report_position_data (PositionData (
        msg->getLatitude (), msg->getLongitude (), msg->getAltitude (), msg->getHeading (), msg->getHorzVel (),
        msg->getVertVel (), msg->getCallSign (), msg->getSquawk (), msg->getICAOAddress (), msg->getTimeStamp (),
        msg->getFlags (), msg->getAltitudeType (), msg->getEmitterType ()));
}

void
fss_client_ssl::handleSMMSettings (
    const std::shared_ptr<flight_safety_system::transport::fss_message_smm_settings> &msg)
{
    this->report_smm_settings (SMMSettings (msg->getServerURL (), msg->getUsername (), msg->getPassword ()));
}

void
fss_client_ssl::sendPosition (double lat, double lng, int16_t alt, uint16_t heading, uint16_t hor_vel, int16_t ver_vel)
{
    static constexpr std::chrono::milliseconds ts_1sec_interval{ 1000 };
    static constexpr uint16_t squawk_vfr = 1200;
    static constexpr uint32_t valid_fields = 1 | 2 | 4 | 8 | 16 | 32;
    static constexpr uint8_t aircraft_type = 14;

    auto now = std::chrono::steady_clock::now ();
    if ((now - this->position_last_sent) >= ts_1sec_interval)
    {
        uint64_t curr_ts = flight_safety_system::fss_current_timestamp ();
        auto msg_pos = std::make_shared<flight_safety_system::transport::fss_message_position_report> (
            lat, lng, alt, heading, hor_vel, ver_vel,
            /* No ICAO Code assigned */
            0,
            /* Use our name as the callsign */
            this->getAssetName (),
            /* Sqawk VFR */
            squawk_vfr,
            /* Time since last contact (0), we are annoncing now */
            0,
            /* Report valid for: coords, altitude, heading, velocity, callsign, squawk */
            valid_fields,
            /* Using GPS for altitude */
            1,
            /* Type is UAV */
            aircraft_type, curr_ts);
        this->sendMsgAll (msg_pos);
        this->position_last_sent = now;
    }
}

void
fss_client_ssl::reachedPoint (int point, int total_points)
{
    auto msg_search
        = std::make_shared<flight_safety_system::transport::fss_message_search_status> (0, point, total_points);
    this->sendMsgAll (msg_search);
}

void
fss_client_ssl::sendBatteryStatus (int8_t remaining, int32_t consumed, double voltage)
{
    auto msg_status
        = std::make_shared<flight_safety_system::transport::fss_message_system_status> (remaining, consumed, voltage);
    this->sendMsgAll (msg_status);
}

void
fss_client_ssl::report_command (FSSCommand cmd)
{
    if (this->command_cb)
    {
        this->command_cb (cmd);
    }
}

void
fss_client_ssl::report_comms_status (FSSCommsStatus status)
{
    if (this->comms_status_cb)
    {
        this->comms_status_cb (status);
    }
}

void
fss_client_ssl::report_goto_update (Point p)
{
    if (this->goto_cb)
    {
        this->goto_cb (p);
    }
}

void
fss_client_ssl::report_altitude_update (uint32_t alt)
{
    if (this->altitude_cb)
    {
        this->altitude_cb (alt);
    }
}

void
fss_client_ssl::report_smm_settings (const SMMSettings &settings)
{
    if (this->smm_settings_cb)
    {
        this->smm_settings_cb (settings);
    }
}

void
fss_client_ssl::report_position_data (const PositionData &pd)
{
    if (this->position_data_cb)
    {
        this->position_data_cb (pd);
    }
}