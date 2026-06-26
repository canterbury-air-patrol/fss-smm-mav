#include "altitude-units.hpp"
#include "command-ack.hpp"
#include "fmu-fss-types.hpp"
#include "internal.hpp"
#include <chrono>

void
fss_client_ssl::sendCommandAck (const std::shared_ptr<flight_safety_system::transport::fss_connection> &conn,
                                uint64_t acked_id, flight_safety_system::transport::fss_asset_command raw_command,
                                flight_safety_system::transport::fss_command_ack_outcome outcome,
                                flight_safety_system::transport::fss_command_ack_reason reason)
{
    /* The command id and the negotiated feature flags are both per-connection,
     * so the ack must be read from and sent back on the originating connection
     * only — never broadcast. The caller resolves the originating fss_server to
     * its connection while that server is known live (on the recv thread); the
     * phase-2 responder then carries only the connection across the async
     * boundary, so a torn-down server cannot dangle here. */
    if (conn == nullptr)
    {
        /* Connection torn down between receiving the command and acking it. */
        return;
    }
    if ((conn->getNegotiatedFeatureFlags () & flight_safety_system::transport::FSS_FEATURE_COMMAND_ACK) == 0)
    {
        /* Peer did not negotiate command-ack support: stay silent, preserving
         * legacy behaviour. */
        return;
    }
    auto ack = std::make_shared<flight_safety_system::transport::fss_message_command_ack> (
        acked_id, raw_command, outcome, reason, flight_safety_system::fss_current_timestamp ());
    conn->sendMsg (ack);
}

void
fss_client_ssl::ackPending (const pending_command_ack &target, const FSSCommandResolution &res)
{
    this->sendCommandAck (target.conn.lock (), target.acked_id, target.raw_command, fss_command_ack_outcome_for (res),
                          fss_command_ack_reason_for (res));
}

void
fss_client_ssl::ackStale (const pending_command_ack &target)
{
    this->sendCommandAck (target.conn.lock (), target.acked_id, target.raw_command, stale_command_ack_outcome,
                          stale_command_ack_reason);
}

void
fss_client_ssl::handleCommandFrom (
    const std::shared_ptr<flight_safety_system::transport::fss_message_asset_command> &msg,
    flight_safety_system::client_ssl::fss_server *origin)
{
    /* The ack is keyed by this copy's own header id and must return on the
     * connection it arrived on. */
    uint64_t acked_id = msg->getId ();
    uint64_t ts = msg->getTimeStamp ();
    flight_safety_system::transport::fss_asset_command raw_command = msg->getCommand ();

    /* Resolve the originating server to its connection here, on the recv thread,
     * while `origin` is guaranteed live. The terminal ack is sent later, from the
     * event-loop thread, by which point the fss_server may have been destroyed
     * (comms-loss teardown, updateServers); carrying the raw origin across that
     * boundary would dangle. We carry the connection instead — a weak_ptr, so the
     * ack never resurrects a torn-down connection and stays silent if it has gone
     * away. */
    std::shared_ptr<flight_safety_system::transport::fss_connection> conn = origin->getConnection ();
    pending_command_ack this_copy{ conn, acked_id, raw_command };

    /* Phase 1: confirm receipt immediately, before the command is actioned. Every
     * copy gets its own received ack on its own connection. */
    this->sendCommandAck (conn, acked_id, raw_command, flight_safety_system::transport::command_ack_received,
                          flight_safety_system::transport::supersede_none);

    /* Map the wire command to the FMU command up front. An unactionable command
     * (unknown/unrecognised) never reaches the state machine, so it forms no
     * command group: reject it synchronously and return rather than opening a
     * group whose terminal ack would never be fired. */
    bool actionable = true;
    FSSCommand fss_command = fss_cmd_unknown;
    switch (raw_command)
    {
        case flight_safety_system::transport::asset_command_rtl:
            fss_command = fss_cmd_rtl;
            break;
        case flight_safety_system::transport::asset_command_disarm:
            fss_command = fss_cmd_disarm;
            break;
        case flight_safety_system::transport::asset_command_goto:
            fss_command = fss_cmd_goto;
            break;
        case flight_safety_system::transport::asset_command_altitude:
            fss_command = fss_cmd_altitude;
            break;
        case flight_safety_system::transport::asset_command_hold:
            fss_command = fss_cmd_hold;
            break;
        case flight_safety_system::transport::asset_command_resume:
            fss_command = fss_cmd_continue;
            break;
        case flight_safety_system::transport::asset_command_terminate:
            fss_command = fss_cmd_terminate;
            break;
        case flight_safety_system::transport::asset_command_manual:
            fss_command = fss_cmd_manual;
            break;
        case flight_safety_system::transport::asset_command_unknown:
        default:
            actionable = false;
            break;
    }
    if (!actionable)
    {
        this->sendCommandAck (conn, acked_id, raw_command, flight_safety_system::transport::command_ack_rejected,
                              flight_safety_system::transport::supersede_none);
        return;
    }

    /* The FMU is connected to all FSS servers and the web frontend pushes the same
     * command to each, so this logical command arrives once per connection. The
     * group decides whether to action it (new command), queue/replay its ack
     * (redundant delivery), or reject it as stale. */
    auto delivery = this->command_group.onDelivery (static_cast<int> (raw_command), ts, this_copy);

    switch (delivery.disposition)
    {
        case CommandAckGroup<pending_command_ack>::Disposition::pending:
            /* Redundant delivery, outcome not yet known: it was queued and will be
             * acked when the resolution lands. Nothing more to do. */
            return;
        case CommandAckGroup<pending_command_ack>::Disposition::already_resolved:
            /* Redundant delivery whose outcome is already cached: ack it now.
             * already_resolved always carries the resolution, but guard the access
             * so a future change can't turn it into a silent unchecked deref. */
            if (delivery.resolution.has_value ())
            {
                this->ackPending (this_copy, *delivery.resolution);
            }
            return;
        case CommandAckGroup<pending_command_ack>::Disposition::stale_superseded:
            /* A genuinely older, different command from a slower server: the newer
             * command is already in effect, so do NOT actuate this one. Ack it as
             * superseded with the dedicated newer-command reason (a later operator
             * command replaced it — distinct from the autonomous safety latches)
             * rather than returning silently and leaving a false 'no ack'. */
            this->ackStale (this_copy);
            return;
        case CommandAckGroup<pending_command_ack>::Disposition::actuate:
            break;
    }

    /* A new logical command: ack any copies of a now-displaced older command as
     * superseded (their resolution never arrived before this one). This is the
     * same situation as the stale_superseded path above — an older command
     * replaced by a newer operator command — so it shares the same outcome and
     * newer-command reason; the only difference is arrival timing, which must not
     * change the operator-facing label. */
    for (const auto &target : delivery.superseded)
    {
        this->ackStale (target);
    }

    /* Phase 2: once the state machine resolves the command, ack the resolved
     * outcome to every copy in the group on its own connection (the group caches
     * it for any copy that arrives after resolution). The responder carries the
     * epoch of the group it opened; resolve() drops it if a newer command has
     * since superseded the group (those copies were already acked above). */
    uint64_t my_epoch = delivery.epoch;
    fss_command_ack_responder ack = [this, my_epoch] (const FSSCommandResolution &res)
    {
        /* Send acks outside any lock: sendMsg touches the wire and a recv thread
         * may be waiting to add a late duplicate. */
        for (const auto &target : this->command_group.resolve (my_epoch, res))
        {
            this->ackPending (target, res);
        }
    };

    /* goto and altitude carry a payload (target position / altitude) that must be
     * reported before the command is actioned. */
    if (raw_command == flight_safety_system::transport::asset_command_goto)
    {
        this->report_goto_update (Point (msg->getLatitude (), msg->getLongitude ()));
    }
    else if (raw_command == flight_safety_system::transport::asset_command_altitude)
    {
        this->report_altitude_update (msg->getAltitude ());
    }
    this->report_command (fss_command, ack);
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
    /* The FSS wire altitude is feet; PositionData carries metres. */
    this->report_position_data (PositionData (
        msg->getLatitude (), msg->getLongitude (), feet_to_metres (msg->getAltitude ()), msg->getHeading (),
        msg->getHorzVel (), msg->getVertVel (), msg->getCallSign (), msg->getSquawk (), msg->getICAOAddress (),
        msg->getTimeStamp (), msg->getFlags (), msg->getAltitudeType (), msg->getEmitterType ()));
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

    /* steady_clock is used only for local rate limiting (it is monotonic and
     * immune to wall-clock jumps). The on-the-wire timestamp below deliberately
     * uses fss_current_timestamp() (wall clock) so it is comparable across
     * hosts; do not collapse these two into a single clock. */
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
fss_client_ssl::report_command (FSSCommand cmd, const fss_command_ack_responder &ack)
{
    if (this->command_cb)
    {
        this->command_cb (cmd, ack);
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