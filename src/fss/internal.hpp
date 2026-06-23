#pragma once
#include <fss-client-ssl.hpp>

#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include "command-ack-group.hpp"
#include "fmu-fss-types.hpp"

#include <cstdint>
#include <memory>

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
    void report_command (FSSCommand cmd, const fss_command_ack_responder &ack);
    /* Send a command acknowledgement on the originating connection, gated on
     * that connection having negotiated FSS_FEATURE_COMMAND_ACK. acked_id is the
     * received command's header id (echoed back); raw_command is the
     * fss_asset_command being acked. A no-op when the feature is not negotiated
     * or the connection has gone away (conn == nullptr). The caller passes the
     * connection (not the fss_server) because the phase-2 ack is resolved
     * asynchronously, after the originating server may have been destroyed; the
     * connection is captured as a weak_ptr at command-receipt time so no raw
     * fss_server is held across that boundary. */
    void sendCommandAck (const std::shared_ptr<flight_safety_system::transport::fss_connection> &conn,
                         uint64_t acked_id, flight_safety_system::transport::fss_asset_command raw_command,
                         flight_safety_system::transport::fss_command_ack_outcome outcome,
                         flight_safety_system::transport::fss_command_ack_reason reason);
    void report_goto_update (Point);
    void report_altitude_update (uint32_t);
    void report_comms_status (FSSCommsStatus);
    void report_smm_settings (const SMMSettings &);
    void report_position_data (const PositionData &);
    std::chrono::steady_clock::time_point position_last_sent{};

    /* One per-server copy of a logical command that is awaiting (or replaying) the
     * terminal ack. acked_id is that copy's own header id; the connection is held
     * weakly so a torn-down server is a silent no-op (see e010c24). raw_command is
     * echoed back in the ack. These are the ack targets the CommandAckGroup below
     * groups and hands back for the FMU to ack. */
    struct pending_command_ack
    {
        std::weak_ptr<flight_safety_system::transport::fss_connection> conn{};
        uint64_t acked_id{ 0 };
        flight_safety_system::transport::fss_asset_command raw_command{
            flight_safety_system::transport::asset_command_unknown
        };
    };

    /* Groups the redundant per-server deliveries of one logical command so they
     * all get the same terminal ack (see command-ack-group.hpp). Tolerance is the
     * same 60 s window the legacy dedup used. */
    static constexpr uint64_t command_dedup_tolerance_ms = 60000;
    CommandAckGroup<pending_command_ack> command_group{ command_dedup_tolerance_ms };

    /* Ack a single command copy with the terminal outcome of `res`, on its own
     * connection and with its own acked_id. Used to drain a group's pending list
     * when the resolution lands and to replay the cached resolution to a
     * late-arriving duplicate. */
    void ackPending (const pending_command_ack &target, const FSSCommandResolution &res);

    /* Ack a single command copy as superseded by a newer command — the stale /
     * displaced-older-command outcome. Both the stale-on-arrival path and the
     * displaced-by-a-newer-command path share this so the operator-facing label
     * cannot drift between them (only the arrival timing differs). */
    void ackStale (const pending_command_ack &target);

  protected:
    void connectionStatusChange (flight_safety_system::client_ssl::connection_status status) override;

  public:
    explicit fss_client_ssl (const char *t_config_file)
        : flight_safety_system::client_ssl::fss_client (t_config_file) {};
    fss_client_ssl (fss_client_ssl &) = delete;
    fss_client_ssl (fss_client_ssl &&) = delete;
    auto operator= (fss_client_ssl &) -> fss_client_ssl & = delete;
    auto operator= (fss_client_ssl &&) -> fss_client_ssl & = delete;
    void handleCommandFrom (const std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> &msg,
                            flight_safety_system::client_ssl::fss_server *origin) override;
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