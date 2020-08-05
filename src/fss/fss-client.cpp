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