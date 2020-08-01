#include <fss-client.hpp>

#include "../fmu-types.hpp"
#include "fmu-fss-types.hpp"

typedef void (*notify_goto_update_cb)(Point);

class fss_client : public flight_safety_system::client::fss_client {
private:
    notify_fss_command_cb command_cb{nullptr};
    notify_fss_comms_cb comms_status_cb{nullptr};
    notify_goto_update_cb goto_cb{nullptr};
    void report_command(FSSCommand cmd);
    void report_goto_update(Point);
    void report_comms_status(FSSCommsStatus);
protected:
    virtual void connectionStatusChange(flight_safety_system::client::connection_status status);
public:
    fss_client(const char *t_config_file) : flight_safety_system::client::fss_client(t_config_file) {};
    virtual void handleCommand(flight_safety_system::transport::fss_message_asset_command *msg);
    virtual void handlePositionReport(flight_safety_system::transport::fss_message_position_report *msg);
};