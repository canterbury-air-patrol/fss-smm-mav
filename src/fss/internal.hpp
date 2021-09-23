#pragma once
#include <fss-client-ssl.hpp>

#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include "fmu-fss-types.hpp"

using notify_goto_update_cb = void (*)(void *, Point);

class fss_client : public flight_safety_system::client::fss_client {
private:
    notify_fss_command_cb command_cb{nullptr};
    notify_fss_comms_cb comms_status_cb{nullptr};
    notify_goto_update_cb goto_cb{nullptr};
    void *goto_cb_priv{nullptr};
    notify_smm_settings_cb smm_settings_cb{nullptr};
    notify_position_cb position_data_cb{nullptr};
    void report_command(FSSCommand cmd);
    void report_goto_update(Point);
    void report_comms_status(FSSCommsStatus);
    void report_smm_settings(SMMSettings);
    void report_position_data(PositionData);
    std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> last_command{};
protected:
    void connectionStatusChange(flight_safety_system::client::connection_status status) override;
public:
    explicit fss_client(const char *t_config_file) : flight_safety_system::client::fss_client(t_config_file) {};
    void handleCommand(const std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> &msg) override;
    void handlePositionReport(const std::shared_ptr<flight_safety_system::transport::fss_message_position_report> &msg) override;
    void handleSMMSettings(const std::shared_ptr<flight_safety_system::transport::fss_message_smm_settings> &msg) override;
    void registerCommandCB(notify_fss_command_cb cb) { this->command_cb = cb; };
    void registerCommsStatusCB(notify_fss_comms_cb cb) { this->comms_status_cb = cb; };
    void registerGotoUpdateCB(notify_goto_update_cb cb, void *priv) { this->goto_cb = cb; this->goto_cb_priv = priv; };
    void registerSMMSettingsCB(notify_smm_settings_cb cb) { this->smm_settings_cb = cb; };
    void registerPositionDataCB(notify_position_cb cb) { this->position_data_cb = cb; };
    void sendPosition(double lat, double lng, int16_t alt, uint16_t heading, uint16_t hor_vel, int16_t ver_vel);
    void reachedPoint(int point, int total_points);
    void sendBatteryStatus(int8_t remaining, int32_t consumed);
    void reconnectAll();
};

class fss_client_ssl : public flight_safety_system::client_ssl::fss_client {
private:
    notify_fss_command_cb command_cb{nullptr};
    notify_fss_comms_cb comms_status_cb{nullptr};
    notify_goto_update_cb goto_cb{nullptr};
    void *goto_cb_priv{nullptr};
    notify_smm_settings_cb smm_settings_cb{nullptr};
    notify_position_cb position_data_cb{nullptr};
    void report_command(FSSCommand cmd);
    void report_goto_update(Point);
    void report_comms_status(FSSCommsStatus);
    void report_smm_settings(SMMSettings);
    void report_position_data(PositionData);
    std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> last_command{};
protected:
    void connectionStatusChange(flight_safety_system::client::connection_status status) override;
public:
    explicit fss_client_ssl(const char *t_config_file) : flight_safety_system::client_ssl::fss_client(t_config_file) {};
    void handleCommand(const std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> &msg) override;
    void handlePositionReport(const std::shared_ptr<flight_safety_system::transport::fss_message_position_report> &msg) override;
    void handleSMMSettings(const std::shared_ptr<flight_safety_system::transport::fss_message_smm_settings> &msg) override;
    void registerCommandCB(notify_fss_command_cb cb) { this->command_cb = cb; };
    void registerCommsStatusCB(notify_fss_comms_cb cb) { this->comms_status_cb = cb; };
    void registerGotoUpdateCB(notify_goto_update_cb cb, void *priv) { this->goto_cb = cb; this->goto_cb_priv = priv; };
    void registerSMMSettingsCB(notify_smm_settings_cb cb) { this->smm_settings_cb = cb; };
    void registerPositionDataCB(notify_position_cb cb) { this->position_data_cb = cb; };
    void sendPosition(double lat, double lng, int16_t alt, uint16_t heading, uint16_t hor_vel, int16_t ver_vel);
    void reachedPoint(int point, int total_points);
    void sendBatteryStatus(int8_t remaining, int32_t consumed);
    void reconnectAll();
};