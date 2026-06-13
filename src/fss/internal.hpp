#pragma once
#include <fss-client-ssl.hpp>

#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include "fmu-fss-types.hpp"

using notify_goto_update_cb = std::function<void (Point)>;
using notify_altitude_update_cb = std::function<void (uint32_t)>;

class fss_client_ssl : public flight_safety_system::client_ssl::fss_client
{
  private:
    notify_fss_command_cb command_cb{};
    notify_fss_comms_cb comms_status_cb{};
    notify_goto_update_cb goto_cb{};
    notify_altitude_update_cb altitude_cb{};
    notify_smm_settings_cb smm_settings_cb{};
    notify_position_cb position_data_cb{};
    void report_command (FSSCommand cmd);
    void report_goto_update (Point);
    void report_altitude_update (uint32_t);
    void report_comms_status (FSSCommsStatus);
    void report_smm_settings (const SMMSettings &);
    void report_position_data (const PositionData &);
    std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> last_command{};
    std::chrono::steady_clock::time_point position_last_sent{};

  protected:
    void connectionStatusChange (flight_safety_system::client_ssl::connection_status status) override;

  public:
    explicit fss_client_ssl (const char *t_config_file)
        : flight_safety_system::client_ssl::fss_client (t_config_file) {};
    fss_client_ssl (fss_client_ssl &) = delete;
    fss_client_ssl (fss_client_ssl &&) = delete;
    auto operator= (fss_client_ssl &) -> fss_client_ssl & = delete;
    auto operator= (fss_client_ssl &&) -> fss_client_ssl & = delete;
    void
    handleCommand (const std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> &msg) override;
    void handlePositionReport (
        const std::shared_ptr<flight_safety_system::transport::fss_message_position_report> &msg) override;
    void
    handleSMMSettings (const std::shared_ptr<flight_safety_system::transport::fss_message_smm_settings> &msg) override;
    void
    registerCommandCB (notify_fss_command_cb cb)
    {
        this->command_cb = cb;
    };
    void
    registerCommsStatusCB (notify_fss_comms_cb cb)
    {
        this->comms_status_cb = cb;
    };
    void
    registerGotoUpdateCB (notify_goto_update_cb cb)
    {
        this->goto_cb = cb;
    };
    void
    registerAltitudeUpdateCB (notify_altitude_update_cb cb)
    {
        this->altitude_cb = cb;
    };
    void
    registerSMMSettingsCB (notify_smm_settings_cb cb)
    {
        this->smm_settings_cb = cb;
    };
    void
    registerPositionDataCB (notify_position_cb cb)
    {
        this->position_data_cb = cb;
    };
    void sendPosition (double lat, double lng, int16_t alt, uint16_t heading, uint16_t hor_vel, int16_t ver_vel);
    void reachedPoint (int point, int total_points);
    void sendBatteryStatus (int8_t remaining, int32_t consumed, double voltage);
    void reconnectAll ();
};