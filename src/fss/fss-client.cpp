#include "fmu-fss-types.hpp"
#include "internal.hpp"

flight_safety_system::transport::fss_message_asset_command last_command(flight_safety_system::transport::asset_command_unknown, 0, 0);

void
fss_client::handleCommand(flight_safety_system::transport::fss_message_asset_command *msg)
{
    /* Don't execute commands older than the last one we handled
       This can happen when there are connections to multiple servers
       and the client that set the command didn't send it to all of them */
    if(msg->getTimeStamp() < last_command.getTimeStamp())
    {
        return;
    }
    last_command = *msg;
    /* Convert Each Command into a MavLink Command */
    switch(msg->getCommand())
    {
        case flight_safety_system::transport::asset_command_rtl:
            this->report_command(fss_cmd_rtl);
            break;
        case flight_safety_system::transport::asset_command_disarm:
            this->report_command(fss_cmd_disarm);
            break;
        case flight_safety_system::transport::asset_command_goto:
            this->report_goto_update(Point(msg->getLatitude(), msg->getLongitude()));
            this->report_command(fss_cmd_goto);
            break;
        case flight_safety_system::transport::asset_command_hold:
            this->report_command(fss_cmd_hold);
            break;
        case flight_safety_system::transport::asset_command_resume:
            this->report_command(fss_cmd_continue);
            break;
        case flight_safety_system::transport::asset_command_terminate:
            this->report_command(fss_cmd_terminate);
            break;
        case flight_safety_system::transport::asset_command_manual:
            this->report_command(fss_cmd_manual);
            break;
        case flight_safety_system::transport::asset_command_unknown:
        default:
            break;
    }
}

void
fss_client::connectionStatusChange(flight_safety_system::client::connection_status status)
{
    switch (status)
    {
        case flight_safety_system::client::CLIENT_CONNECTION_STATUS_CONNECTED_1_SERVER:
        case flight_safety_system::client::CLIENT_CONNECTION_STATUS_CONNECTED_2_OR_MORE:
            /* No Need for RTL in these cases */
            this->report_comms_status(fss_comms_okay);
            break;
        case flight_safety_system::client::CLIENT_CONNECTION_STATUS_DISCONNECTED:
        case flight_safety_system::client::CLIENT_CONNECTION_STATUS_UNKNOWN:
            this->report_comms_status(fss_comms_failure);
            break;
    }
}

void
fss_client::handlePositionReport(flight_safety_system::transport::fss_message_position_report *msg)
{
    this->report_position_data(PositionData(msg->getLatitude(), msg->getLongitude(), msg->getAltitude(),
        msg->getHeading(), msg->getHorzVel(), msg->getVertVel(), msg->getCallSign(), msg->getSquawk(), msg->getICAOAddress(),
        msg->getTimeStamp(), msg->getFlags(), msg->getAltitudeType(), msg->getEmitterType()));
}

void
fss_client::handleSMMSettings(flight_safety_system::transport::fss_message_smm_settings *msg)
{
    this->report_smm_settings(SMMSettings(msg->getServerURL(), msg->getUsername(), msg->getPassword()));
}

void
fss_client::sendPosition(double lat, double lng, int16_t alt, uint16_t heading, uint16_t hor_vel, int16_t ver_vel)
{
    static uint64_t position_last_sent = 0;
    uint64_t curr_ts = flight_safety_system::fss_current_timestamp();
    bool res = curr_ts > (position_last_sent + 1000);
    if (res)
    {
        auto msg_pos = new flight_safety_system::transport::fss_message_position_report(lat, lng, alt, heading, hor_vel, ver_vel,
            /* No ICAO Code assigned */
            0,
            /* Use our name as the callsign */
            this->getAssetName(),
            /* Sqawk VFR */
            1200,
            /* Time since last contact (0), we are annoncing now */
            0,
            /* Report valid for: coords, altitude, heading, velocity, callsign, squawk */
            1 | 2 | 4 | 8 | 16 | 32,
            /* Using GPS for altitude */
            1,
            /* Type is UAV */
            14,
            curr_ts);
        this->sendMsgAll(msg_pos);
        delete msg_pos;
        position_last_sent = curr_ts;
    }
}

void
fss_client::reachedPoint(int point, int total_points)
{
    auto msg_search = new flight_safety_system::transport::fss_message_search_status(0, point, total_points);
    this->sendMsgAll(msg_search);
    delete msg_search;
}

void
fss_client::sendBatteryStatus(int8_t remaining, int32_t consumed)
{
    auto msg_status = new flight_safety_system::transport::fss_message_system_status(remaining, consumed);
    this->sendMsgAll (msg_status);
    delete msg_status;
}

void
fss_client::report_command(FSSCommand cmd)
{
    if (this->command_cb != nullptr)
    {
        this->command_cb(this->command_cb_priv, cmd);
    }
}

void
fss_client::report_comms_status(FSSCommsStatus status)
{
    if (this->comms_status_cb != nullptr)
    {
        this->comms_status_cb (this->comms_status_cb_priv, status);
    }
}

void
fss_client::report_goto_update(Point p)
{
    if (this->goto_cb != nullptr)
    {
        this->goto_cb (this->goto_cb_priv, p);
    }
}

void
fss_client::report_smm_settings(SMMSettings settings)
{
    if (this->smm_settings_cb != nullptr)
    {
        this->smm_settings_cb (this->smm_settings_cb_priv, settings);
    }
}

void
fss_client::report_position_data(PositionData pd)
{
    if (this->position_data_cb != nullptr)
    {
        this->position_data_cb (this->position_data_cb_priv, pd);
    }
}