#include <fss-client.hpp>

#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include "fmu-fss-types.hpp"

typedef void (*notify_goto_update_cb)(void *, Point);

class fss_client : public flight_safety_system::client::fss_client {
private:
    notify_fss_command_cb command_cb{nullptr};
    void *command_cb_priv{nullptr};
    notify_fss_comms_cb comms_status_cb{nullptr};
    void *comms_status_cb_priv{nullptr};
    notify_goto_update_cb goto_cb{nullptr};
    void *goto_cb_priv{nullptr};
    notify_smm_settings_cb smm_settings_cb{nullptr};
    void *smm_settings_cb_priv{nullptr};
    void report_command(FSSCommand cmd);
    void report_goto_update(Point);
    void report_comms_status(FSSCommsStatus);
    void report_smm_settings(SMMSettings);
protected:
    virtual void connectionStatusChange(flight_safety_system::client::connection_status status);
public:
    fss_client(const char *t_config_file) : flight_safety_system::client::fss_client(t_config_file) {};
    virtual void handleCommand(flight_safety_system::transport::fss_message_asset_command *msg);
    virtual void handlePositionReport(flight_safety_system::transport::fss_message_position_report *msg);
    virtual void handleSMMSettings(flight_safety_system::transport::fss_message_smm_settings *msg);
    void registerCommandCB(notify_fss_command_cb cb, void *priv) { this->command_cb = cb; this->command_cb_priv = priv; };
    void registerCommsStatusCB(notify_fss_comms_cb cb, void *priv) { this->comms_status_cb = cb; this->comms_status_cb_priv = priv; };
    void registerGotoUpdateCB(notify_goto_update_cb cb, void *priv) { this->goto_cb = cb; this->goto_cb_priv = priv; };
    void registerSMMSettingsCB(notify_smm_settings_cb cb, void *priv) { this->smm_settings_cb = cb; this->smm_settings_cb_priv = priv; };
    void sendPosition(double lat, double lng, int16_t alt, uint16_t heading, uint16_t hor_vel, int16_t ver_vel);
    void reachedPoint(int point, int total_points);
};