#include "altitude-units.hpp"
#include "battery-voltage.hpp"
#include "internal.hpp"
#include "mav-comms.hpp"
#include "mav.hpp"
#include "mission-plan.hpp"
#include "smm/search-altitude.hpp"
#include "util.hpp"
#include "velocity.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <optional>
#include <thread>

#include <cerrno>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ardupilotmega/mavlink.h>

constexpr int BUFFER_LEN = 2048;

constexpr uint8_t SYS_ID = 200;
constexpr uint8_t COMP_ID = 1;

/* The autopilot this FMU talks to. Every direct command (mode, arm/disarm,
 * terminate, altitude, stream requests) targets this system/component pair;
 * only mission traffic differs (see MISSION_TARGET_SYS_ID below). */
constexpr uint8_t TARGET_SYS_ID = 1;
constexpr uint8_t TARGET_COMP_ID = 1;
/* Mission upload (MISSION_COUNT, mission items, MISSION_SET_CURRENT) targets
 * sysid 0 rather than TARGET_SYS_ID — a deliberate ArduPilot-ism (broadcast
 * mission upload accepted by whichever system is listening), not an
 * oversight; kept distinct from TARGET_SYS_ID so a future change to one
 * cannot accidentally also move the other (todo/73). */
constexpr uint8_t MISSION_TARGET_SYS_ID = 0;
constexpr mavlink_channel_t MAV_RECV_CHANNEL = MAVLINK_COMM_0;
constexpr mavlink_channel_t MAV_SEND_CHANNEL = MAVLINK_COMM_1;

/* Convert double/float into int32_t */
constexpr double LAT_LNG_COV = 0.0000001;

void
mav_connection::sendHeartBeat ()
{
    mavlink_message_t msg;
    std::lock_guard<std::mutex> lk (this->send_lock);
    mavlink_msg_heartbeat_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                                     MAV_STATE_ACTIVE);
    this->sendMavLinkMsgLocked (&msg);
}

void
mav_connection::heartbeat_loop ()
{
    constexpr uint64_t heartbeat_timeout_ms = 5000;
    while (!this->stopping)
    {
        bool fd_open = this->fd.load () != -1;
        if (fd_open)
        {
            this->sendHeartBeat ();
        }
        /* This loop is the single authority for the MAV comms status: it edge-
         * triggers the callback on every up<->down change. mav_comms_ok holds the
         * last reported state, initialised to "up" to match the state machine's
         * optimistic default, so the first down observation (no link / no
         * heartbeat yet at cold start) reports a failure and corrects it. A real
         * heartbeat then reports the link back up. Keeping the report in one place
         * also means the callback is only ever invoked from this thread. */
        bool up
            = mav_comms_is_up (fd_open, current_timestamp_ms (), this->last_heartbeat_ts.load (), heartbeat_timeout_ms);
        if (up != this->mav_comms_ok.load ())
        {
            this->mav_comms_ok.store (up);
            if (!up)
            {
                this->logger.log (LogLevel::error, std::string ("WARN: Autopilot link down (")
                                                       + (fd_open ? "no heartbeat" : "no link")
                                                       + ") — MAV comms failure");
                /* A goto's MISSION_COUNT can reach the autopilot with the rest
                 * of the upload (request-driven) never completing if the link
                 * then drops. Unlike RTL/failsafe/low-battery/terminate this
                 * is not replayed on recovery (todo/61), so make the silent
                 * no-op visible instead. */
                bool goto_upload_lost;
                {
                    std::lock_guard<std::mutex> state_lk{ this->state_lock };
                    goto_upload_lost = this->goto_ack_pending;
                    this->goto_ack_pending = false;
                }
                if (goto_upload_lost)
                {
                    this->logger.log (LogLevel::error,
                                      "WARN: MAV link lost with a goto mission upload in flight (MISSION_COUNT sent, "
                                      "no MISSION_ACK) — the goto did not take effect");
                }
            }
            if (this->mav_comms_cb)
            {
                this->mav_comms_cb (up ? MavCommsStatus::ok : MavCommsStatus::failure);
            }
        }
        std::unique_lock<std::mutex> lk (this->heartbeat_mutex);
        this->heartbeat_cv.wait_for (lk, std::chrono::seconds (1), [this] { return this->stopping.load (); });
    }
}

void
mav_connection::sendADSB (uint32_t icao_address, double lat, double lng, double altitude_m, uint8_t altitude_type,
                          uint16_t heading, uint16_t hor_vel, uint16_t ver_vel, char *callsign, uint8_t emitter_type,
                          uint8_t tslc, uint16_t flags, uint16_t squawk)
{
    mavlink_message_t msg;
    /* ADSB_VEHICLE.altitude is millimetres; PositionData carries metres. */
    std::lock_guard<std::mutex> lk (this->send_lock);
    mavlink_msg_adsb_vehicle_pack_chan (
        SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, icao_address, static_cast<int32_t> (lat / LAT_LNG_COV),
        static_cast<int32_t> (lng / LAT_LNG_COV), altitude_type, metres_to_mav_mm (altitude_m), heading, hor_vel,
        static_cast<int16_t> (ver_vel), callsign, emitter_type, tslc, flags, squawk);
    this->sendMavLinkMsgLocked (&msg);
}

void
mav_connection::warnUnresolvedMode (MavModeCommand command)
{
    this->logger.log (LogLevel::error, std::string ("WARN: ") + mav_mode_command_name (command)
                                           + " command received before the autopilot type is known (no heartbeat "
                                             "yet); deferring");
}

auto
mav_connection::setFlightMode (uint8_t fmode) -> bool
{
    mavlink_message_t msg;
    std::lock_guard<std::mutex> lk (this->send_lock);
    mavlink_msg_set_mode_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, TARGET_SYS_ID,
                                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED
                                        | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED
                                        | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
                                    fmode);
    return this->sendMavLinkMsgLocked (&msg);
}

auto
mav_connection::setResolvedMode (MavModeCommand command, bool clear_search_loaded) -> bool
{
    auto sys = this->systems.findExistingSystem (TARGET_SYS_ID);
    std::optional<uint8_t> fmode = resolve_mav_mode (sys != nullptr ? sys->getAutoPilotType () : 0, command);
    if (!fmode.has_value ())
    {
        {
            std::lock_guard<std::mutex> lk{ this->state_lock };
            this->pending_mode_command = command;
        }
        warnUnresolvedMode (command);
        /* Deferred, not transmitted: replayPendingMode() re-sends it once the
         * autopilot type is known. Report not-sent so a safety-critical caller
         * also tracks it for replay on link recovery. */
        return false;
    }
    bool sent = this->setFlightMode (*fmode);
    std::lock_guard<std::mutex> lk{ this->state_lock };
    this->pending_mode_command.reset ();
    if (clear_search_loaded)
    {
        this->search_loaded = false;
    }
    return sent;
}

void
mav_connection::replayPendingMode (uint8_t autopilot_type)
{
    std::optional<MavModeCommand> command;
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        command = this->pending_mode_command;
    }
    if (!command.has_value ())
    {
        return;
    }
    std::optional<uint8_t> fmode = resolve_mav_mode (autopilot_type, *command);
    if (!fmode.has_value ())
    {
        return;
    }
    this->setFlightMode (*fmode);
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        if (this->pending_mode_command == command)
        {
            this->pending_mode_command.reset ();
        }
        if (*command != MavModeCommand::auto_mode)
        {
            this->search_loaded = false;
        }
    }
}

auto
mav_connection::commandRTL () -> bool
{
    return this->setResolvedMode (MavModeCommand::rtl, true);
}

auto
mav_connection::commandDisARM () -> bool
{
    mavlink_message_t msg;
    bool sent;
    {
        std::lock_guard<std::mutex> lk (this->send_lock);
        mavlink_msg_command_long_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, TARGET_SYS_ID, TARGET_COMP_ID,
                                            MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
        sent = this->sendMavLinkMsgLocked (&msg);
    }
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
    return sent;
}

auto
mav_connection::commandGoto (Point p) -> bool
{
    mavlink_message_t msg;
    // RTL the aircraft so we can load a mission
    this->commandRTL ();
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->goto_position = p;
        this->goto_active = true;
        this->search_loaded = false;
    }
    bool sent;
    {
        std::lock_guard<std::mutex> lk (this->send_lock);
        mavlink_msg_mission_count_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MISSION_TARGET_SYS_ID,
                                             TARGET_COMP_ID, mission_count_for (0, MissionPlanMode::go_to),
                                             MAV_MISSION_TYPE_MISSION, 0);
        /* The goto is a mission upload: report whether its opening MISSION_COUNT
         * reached the autopilot (the rest is request-driven). Unlike RTL/
         * failsafe/low-battery/terminate, a goto is not replayed on link
         * recovery if the upload never completes (todo/61) — so a send
         * failure here is surfaced immediately rather than silently treated
         * as a successful goto by the caller. */
        sent = this->sendMavLinkMsgLocked (&msg);
    }
    if (sent)
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->goto_ack_pending = true;
    }
    else
    {
        this->logger.log (LogLevel::error,
                          "WARN: goto MISSION_COUNT failed to send (MAV link down); goto did not take effect");
    }
    return sent;
}

auto
mav_connection::commandManual () -> bool
{
    return this->setResolvedMode (MavModeCommand::manual, true);
}

auto
mav_connection::commandHold () -> bool
{
    return this->setResolvedMode (MavModeCommand::hold, true);
}

void
mav_connection::commandAuto ()
{
    this->setResolvedMode (MavModeCommand::auto_mode, false);
}

auto
mav_connection::commandAltitude (uint32_t alt) -> bool
{
    mavlink_message_t msg;
    /* alt is feet (the FSS wire unit). Convert to metres and clamp into the
     * regulatory [floor, cap] range so a direct operator altitude command
     * cannot fly above the ceiling — or below the floor — that the search and
     * goto altitudes already honour. */
    float alt_m = static_cast<float> (clamp_command_altitude (alt, this->altitude_floor_m, this->altitude_cap_m));

    /* MAV_CMD_DO_CHANGE_ALTITUDE:
       Param 1: Altitude (float, meters)
       Param 2: Frame (MAV_FRAME_GLOBAL_RELATIVE_ALT = 3)
    */
    std::lock_guard<std::mutex> lk (this->send_lock);
    mavlink_msg_command_long_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, TARGET_SYS_ID, TARGET_COMP_ID,
                                        MAV_CMD_DO_CHANGE_ALTITUDE, 0, alt_m, MAV_FRAME_GLOBAL_RELATIVE_ALT, 0, 0, 0, 0,
                                        0);
    return this->sendMavLinkMsgLocked (&msg);
}

auto
mav_connection::commandForceDisARM () -> bool
{
    constexpr float force_magic = 21196.0f;
    mavlink_message_t msg;
    bool sent;
    {
        std::lock_guard<std::mutex> lk (this->send_lock);
        mavlink_msg_command_long_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, TARGET_SYS_ID, TARGET_COMP_ID,
                                            MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, force_magic, 0, 0, 0, 0, 0);
        sent = this->sendMavLinkMsgLocked (&msg);
    }
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
    return sent;
}

auto
mav_connection::commandTerminate () -> bool
{
    mavlink_message_t msg;
    bool sent;
    {
        std::lock_guard<std::mutex> lk (this->send_lock);
        mavlink_msg_command_long_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, TARGET_SYS_ID, TARGET_COMP_ID,
                                            MAV_CMD_DO_FLIGHTTERMINATION, 0, 1, 0, 0, 0, 0, 0, 0);
        sent = this->sendMavLinkMsgLocked (&msg);
    }
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
    return sent;
}

void
mav_connection::send_waypoint (uint16_t seq, uint8_t mission_type)
{
    constexpr int acceptable_radius = 5;
    mavlink_message_t msg;
    bool local_goto_active;
    Point local_goto_position;
    std::shared_ptr<SMMSearch> local_search;
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        local_goto_active = this->goto_active;
        local_goto_position = this->goto_position;
        local_search = this->search;
    }
    /* In search mode with no search loaded there is nothing to send. */
    if (!local_goto_active && local_search == nullptr)
    {
        return;
    }

    /* In goto mode there is no search, and the point count is irrelevant to the
     * goto layout anyway; read the count without copying the point vector. */
    std::size_t num_points = local_search != nullptr ? static_cast<std::size_t> (local_search->getPointsCount ()) : 0;
    MissionPlanMode mode = local_goto_active ? MissionPlanMode::go_to : MissionPlanMode::search;
    MissionItem item = mission_item_for (seq, num_points, mode);

    std::lock_guard<std::mutex> lk (this->send_lock);
    switch (item.kind)
    {
        case MissionItemKind::rtl:
            mavlink_msg_mission_item_int_pack_chan (
                SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MISSION_TARGET_SYS_ID, TARGET_COMP_ID,
                seq,                           /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Use altitude relative to the home point */
                MAV_CMD_NAV_RETURN_TO_LAUNCH,  /* Return home */
                0,                             /* Not the current point */
                0,                             /* Auto continue: No */
                0, 0, 0, 0, 0, 0, 0,           /* Parameters ignored */
                mission_type);
            break;
        case MissionItemKind::goto_point:
            /* The scaled int32 lat/lon legitimately sit next to the float altitude in this MAVLink message;
             * clang-tidy's swapped-arguments heuristic cannot tell them apart. */
            // NOLINTNEXTLINE(bugprone-swapped-arguments)
            mavlink_msg_mission_item_int_pack_chan (
                SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MISSION_TARGET_SYS_ID, TARGET_COMP_ID,
                seq,                           /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Use altitude relative to the home point */
                MAV_CMD_NAV_WAYPOINT,          /* Navigate to a point */
                0,                             /* This waypoint is the current target */
                1,                             /* Auto continue */
                0,                             /* Hold time: 0s */
                acceptable_radius,             /* Accept radius: m */
                0,                             /* Pass radius: 0m */
                NAN,                           /* Yaw: NaN for dont care */
                static_cast<int32_t> (local_goto_position.getLatitude () / LAT_LNG_COV),  /* Latitude */
                static_cast<int32_t> (local_goto_position.getLongitude () / LAT_LNG_COV), /* Longitude */
                static_cast<float> (this->goto_altitude_m),                               /* Altitude (m AGL) */
                mission_type);
            break;
        case MissionItemKind::takeoff:
            /* Most versions of ArduPilot ignore the zeroth mission command, so we need to send the first one twice */
            mavlink_msg_mission_item_int_pack_chan (
                SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MISSION_TARGET_SYS_ID, TARGET_COMP_ID,
                seq,                                                    /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT,                          /* Use altitude relative to the home point */
                MAV_CMD_NAV_TAKEOFF,                                    /* Take off and climb */
                (seq == 1 && local_search->getCurrentPointIdx () == 0), /* Are we at the beginning of the search */
                1,                                                      /* Auto continue: No */
                5,                                                      /* Pitch/climb angle (plane only) */
                0,                                                      /* Ignored */
                0,                                                      /* Ignored */
                0,                                                      /* Yaw angle */
                0,                                                      /* Latitude */
                0,                                                      /* Longitude */
                local_search->getAltitude (),                           /* Altitude */
                mission_type);
            break;
        case MissionItemKind::search_point:
        {
            /* Load each point of the search, the first 2 mission items are setup, so the seq is offset */
            Point p = local_search->getPoint (item.point_index);
            /* The scaled int32 lat/lon legitimately sit next to the float altitude in this MAVLink message;
             * clang-tidy's swapped-arguments heuristic cannot tell them apart. */
            // NOLINTNEXTLINE(bugprone-swapped-arguments)
            mavlink_msg_mission_item_int_pack_chan (
                SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MISSION_TARGET_SYS_ID, TARGET_COMP_ID,
                seq,                           /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Use altitude relative to the home point */
                MAV_CMD_NAV_WAYPOINT,          /* Navigate to a point */
                (local_search->getCurrentPointIdx () == static_cast<int> (item.point_index)), /* The current target? */
                1,                                                                            /* Auto continue */
                0,                                                                            /* Hold time: 0s */
                acceptable_radius,                                                            /* Accept radius: m */
                0,                                                                            /* Pass radius: 0m */
                NAN,                                                    /* Yaw: NaN for dont care */
                static_cast<int32_t> (p.getLatitude () / LAT_LNG_COV),  /* Latitude */
                static_cast<int32_t> (p.getLongitude () / LAT_LNG_COV), /* Longitude */
                static_cast<float> (local_search->getAltitude ()),      /* Altitude (m) */
                mission_type);
            break;
        }
    }
    this->sendMavLinkMsgLocked (&msg);
}

void
mav_connection::setCurrentWP (uint16_t seq)
{
    mavlink_message_t msg;
    std::lock_guard<std::mutex> lk (this->send_lock);
    mavlink_msg_mission_set_current_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MISSION_TARGET_SYS_ID,
                                               TARGET_COMP_ID, seq);
    this->sendMavLinkMsgLocked (&msg);
}

void
mav_connection::mission_ack (bool accepted)
{
    bool goto_set_current = false;
    bool search_set_current = false;
    uint16_t search_seq = 0;
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        /* The upload this MISSION_ACK completes is no longer at risk of a
         * silent link-drop no-op (todo/61), whether accepted or rejected. */
        this->goto_ack_pending = false;
        if (this->goto_active && accepted)
        {
            goto_set_current = true;
        }
        if (this->search_loading)
        {
            this->search_loading = false;
            if (accepted)
            {
                this->search_loaded = true;
                search_set_current = true;
                if (this->search != nullptr)
                {
                    /* getCurrentPointIdx is a search-point index; the autopilot
                     * needs the mission sequence number, which is offset past the
                     * two setup/takeoff items. */
                    search_seq = search_point_mission_seq (this->search->getCurrentPointIdx ());
                }
            }
            else
            {
                this->search = nullptr;
            }
        }
    }
    if (goto_set_current)
    {
        /* Select the freshly-uploaded mission item before engaging AUTO. ArduPilot
         * can otherwise race the mode change against its post-upload mission
         * current reset and stay in the previous mode (todo/77). */
        this->setCurrentWP (0);
        this->commandAuto ();
    }
    if (search_set_current)
    {
        /* Keep the search-resume path in the same post-upload order as goto. */
        this->setCurrentWP (search_seq);
        this->commandAuto ();
    }
}

auto
mav_connection::loadSearch () -> bool
{
    /* Enter RTL while loading the search */
    this->commandRTL ();
    /* Load the existing search into the FC, and jump to the current target point */
    /* Tell the FC how many items there are: 2 setup slots, the N search points,
     * and one RTL terminator (mission_count_for). The count must include the RTL
     * slot, otherwise the FC never requests it and the search has no return-home
     * item at the end. */
    mavlink_message_t msg;
    std::size_t count;
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->goto_active = false;
        /* A pending goto upload is superseded by this search upload, not lost
         * to a link drop; nothing to warn about (todo/61). */
        this->goto_ack_pending = false;
        this->search_loaded = false;
        this->search_loading = true;
        count = mission_count_for (this->search->getPoints ().size (), MissionPlanMode::search);
        /* MISSION_COUNT is a 16-bit field. A real search has a handful of
         * waypoints so this never trips, but log if it ever does rather than
         * silently uploading a truncated mission with no idea why. */
        if (count > UINT16_MAX)
        {
            this->logger.log (LogLevel::error, "WARN: search has " + std::to_string (this->search->getPoints ().size ())
                                                   + " waypoints; mission count " + std::to_string (count)
                                                   + " exceeds the 16-bit MAVLink field and will be truncated");
        }
    }
    {
        std::lock_guard<std::mutex> lk (this->send_lock);
        mavlink_msg_mission_count_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, MISSION_TARGET_SYS_ID,
                                             TARGET_COMP_ID, static_cast<uint16_t> (count), MAV_MISSION_TYPE_MISSION,
                                             0);
        /* Report whether the opening MISSION_COUNT reached the autopilot; the
         * remaining items are request-driven. */
        return this->sendMavLinkMsgLocked (&msg);
    }
}

void
mav_connection::loadSearch (const std::shared_ptr<SMMSearch> &t_search)
{
    bool need_reload = false;
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        if (this->search != t_search || !this->search_loaded)
        {
            this->search = t_search;
            need_reload = (this->search != nullptr);
        }
    }
    if (need_reload)
    {
        this->loadSearch ();
    }
}

void
mav_connection::requestStream (int sysid, int compid, uint32_t command, uint32_t interval)
{
    mavlink_message_t msg;
    std::lock_guard<std::mutex> lk (this->send_lock);
    mavlink_msg_command_long_pack_chan (SYS_ID, COMP_ID, MAV_SEND_CHANNEL, &msg, sysid, compid,
                                        MAV_CMD_SET_MESSAGE_INTERVAL, 0, static_cast<float> (command),
                                        static_cast<float> (interval), 0, 0, 0, 0, 1);
    this->sendMavLinkMsgLocked (&msg);
}

void
mav_connection::processMavLinkMsg (mavlink_message_t *msg, mavlink_status_t *status __attribute__ ((unused)))
{
    auto sys = this->systems.findSystem (msg->sysid);
    /* ensure this component is registered (find-or-create side effect); the
     * returned component object is not needed here. */
    (void)sys->findComponent (msg->compid);

    if (!sys->isSetup ())
    {
        if (msg->sysid == TARGET_SYS_ID)
        {
            /* Request the position and battery streams at the configured
             * intervals (microseconds; defaults are 200ms position / 1s
             * battery). Restricted to the autopilot system: any other system
             * whose first message arrives on this link (e.g. another GCS)
             * would only ignore or NAK the request (todo/73). */
            this->requestStream (msg->sysid, msg->compid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                 this->position_stream_interval_us);
            this->requestStream (msg->sysid, msg->compid, MAVLINK_MSG_ID_BATTERY_STATUS,
                                 this->battery_stream_interval_us);
        }
        sys->setupComplete ();
    }

    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            /* Record the autopilot metadata BEFORE the receipt timestamp. The
             * comms-up signal is driven off last_heartbeat_ts by heartbeat_loop(),
             * the single owner of the comms status; recording the type first means
             * "link up" implies the FMU can already resolve airframe-specific
             * commands, so a command issued the instant the link comes up is not
             * needlessly deferred. */
            uint8_t autopilot_type = mavlink_msg_heartbeat_get_type (msg);
            sys->setAutoPilotMode (autopilot_type);
            sys->setFlightMode (mavlink_msg_heartbeat_get_custom_mode (msg));
            this->last_heartbeat_ts.store (current_timestamp_ms ());
            this->replayPendingMode (autopilot_type);
        }
        break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            /* New position data */
            int32_t lat = mavlink_msg_global_position_int_get_lat (msg);
            int32_t lng = mavlink_msg_global_position_int_get_lon (msg);
            int32_t alt = mavlink_msg_global_position_int_get_alt (msg);
            double latd = (static_cast<double> (lat) * LAT_LNG_COV);
            double lngd = (static_cast<double> (lng) * LAT_LNG_COV);
            uint16_t heading = mavlink_msg_global_position_int_get_hdg (msg);
            int16_t vx = mavlink_msg_global_position_int_get_vx (msg);
            int16_t vy = mavlink_msg_global_position_int_get_vy (msg);
            int16_t vz = mavlink_msg_global_position_int_get_vz (msg);
            uint16_t vh = horizontal_velocity (vx, vy);
            {
                std::lock_guard<std::mutex> lk (this->state_lock);
                this->last_position = Point (latd, lngd);
            }
            /* GLOBAL_POSITION_INT.alt is millimetres; PositionData carries metres. */
            this->report_position (latd, lngd, mav_mm_to_metres (alt), heading, vh, vz);
        }
        break;
        case MAVLINK_MSG_ID_BATTERY_STATUS:
        {
            /* Battery Status */
            int32_t current_consumed = mavlink_msg_battery_status_get_current_consumed (msg);
            int8_t remaining = mavlink_msg_battery_status_get_battery_remaining (msg);
            uint16_t voltages[10] = { 0 };
            mavlink_msg_battery_status_get_voltages (msg, voltages);
            this->report_battery_status (remaining, current_consumed, battery_pack_voltage_v (voltages[0]));
        }
        break;
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
        {
            /* Reached a new waypoint, update the current search progress */
            uint16_t seq = mavlink_msg_mission_item_reached_get_seq (msg);
            this->report_reached (seq);
        }
        break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            if (mavlink_msg_mission_request_get_target_system (msg) == SYS_ID
                && mavlink_msg_mission_request_get_target_component (msg) == COMP_ID)
            {
                /* Getting asked for a specific point in mission */
                uint16_t seq = mavlink_msg_mission_request_get_seq (msg);
                uint8_t mt = mavlink_msg_mission_request_get_mission_type (msg);
                this->send_waypoint (seq, mt);
            }
        }
        break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        {
            if (mavlink_msg_mission_request_int_get_target_system (msg) == SYS_ID
                && mavlink_msg_mission_request_int_get_target_component (msg) == COMP_ID)
            {
                /* Getting asked for a specific point in mission */
                uint16_t seq = mavlink_msg_mission_request_int_get_seq (msg);
                uint8_t mt = mavlink_msg_mission_request_int_get_mission_type (msg);
                this->send_waypoint (seq, mt);
            }
        }
        break;
        case MAVLINK_MSG_ID_MISSION_ACK:
        {
            if (mavlink_msg_mission_ack_get_target_system (msg) == SYS_ID
                && mavlink_msg_mission_ack_get_target_component (msg) == COMP_ID)
            {
                /* Our mission was acknowledged */
                this->mission_ack (mavlink_msg_mission_ack_get_type (msg) == MAV_MISSION_ACCEPTED);
            }
        }
        break;
        case MAVLINK_MSG_ID_ADSB_VEHICLE:
        case MAVLINK_MSG_ID_COLLISION:
        case MAVLINK_MSG_ID_COMMAND_ACK:
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_PARAM_VALUE:
        case MAVLINK_MSG_ID_TIMESYNC:
        case MAVLINK_MSG_ID_SCALED_PRESSURE2:
        case MAVLINK_MSG_ID_HOME_POSITION:
        case MAVLINK_MSG_ID_COMMAND_LONG:
        case MAVLINK_MSG_ID_AIRSPEED_AUTOCAL:
        case MAVLINK_MSG_ID_MISSION_CURRENT:
        case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
        case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        case MAVLINK_MSG_ID_HWSTATUS:
        case MAVLINK_MSG_ID_WIND:
        case MAVLINK_MSG_ID_AOA_SSA:
        case MAVLINK_MSG_ID_TERRAIN_REPORT:
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        case MAVLINK_MSG_ID_VIBRATION:
        case MAVLINK_MSG_ID_SYSTEM_TIME:
        case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
        case MAVLINK_MSG_ID_ATTITUDE:
        case MAVLINK_MSG_ID_AHRS:
        case MAVLINK_MSG_ID_AHRS2:
        case MAVLINK_MSG_ID_AHRS3:
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
        case MAVLINK_MSG_ID_RAW_IMU:
        case MAVLINK_MSG_ID_SCALED_IMU2:
        case MAVLINK_MSG_ID_SCALED_IMU3:
        case MAVLINK_MSG_ID_SIM_STATE:
        case MAVLINK_MSG_ID_RC_CHANNELS:
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        case MAVLINK_MSG_ID_VFR_HUD:
        case MAVLINK_MSG_ID_SIMSTATE:
        case MAVLINK_MSG_ID_MEMINFO:
        case MAVLINK_MSG_ID_SENSOR_OFFSETS:
        case MAVLINK_MSG_ID_TERRAIN_REQUEST:
        case MAVLINK_MSG_ID_POWER_STATUS:
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
        case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4:
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            /* The STATUSTEXT text field is a fixed 50 bytes and is NOT guaranteed
             * NUL-terminated on the wire (a full 50-char message has no
             * terminator). Size the buffer one larger and zero-initialise it so
             * the trailing byte is always 0; get_text() writes only the 50 field
             * bytes, leaving that terminator intact. Printing the raw field
             * directly would over-read past the copied bytes. */
            char text_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = { 0 };
            mavlink_msg_statustext_get_text (msg, text_buf);
            this->logger.log (LogLevel::info, std::string ("Status: ") + text_buf);
        }
        break;
        default:
            /* A genuinely unrecognised message id (everything we expect is
             * enumerated above). Keep it to a single concise debug-level line —
             * the old multi-field stdout dump fanned out across the operator
             * console and, for any unexpected high-rate stream, would behave as
             * a firehose at the default log level too. */
            this->logger.log (LogLevel::debug, "WARN: unhandled MAVLink msg id " + std::to_string (msg->msgid)
                                                   + " from sys " + std::to_string (msg->sysid) + " comp "
                                                   + std::to_string (msg->compid));
    }
}

auto
convert_str_to_sa (const std::string &addr, uint16_t port, struct sockaddr_storage *sa) -> bool
{
    struct addrinfo hints = {};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *ai = nullptr;
    if (getaddrinfo (addr.c_str (), nullptr, &hints, &ai) != 0)
        return false;

    if (ai->ai_addrlen > sizeof (struct sockaddr_storage))
    {
        freeaddrinfo (ai);
        return false;
    }

    memcpy (sa, ai->ai_addr, ai->ai_addrlen);
    int family = ai->ai_family;
    freeaddrinfo (ai);

    switch (family)
    {
        case AF_INET:
        {
            auto *sa_in = reinterpret_cast<struct sockaddr_in *> (sa);
            sa_in->sin_port = htons (port);
        }
        break;
        case AF_INET6:
        {
            auto *sa_in = reinterpret_cast<struct sockaddr_in6 *> (sa);
            sa_in->sin6_port = htons (port);
        }
        break;
        default:
            return false;
    }

    return true;
}

void
mav_connection::processMessages ()
{
    int cur_fd;
    while ((cur_fd = this->fd.load ()) != -1)
    {
        char buf[BUFFER_LEN];
        mavlink_message_t msg;
        mavlink_status_t status;
        ssize_t received = recv (cur_fd, buf, sizeof (buf), 0);
        if (received > 0)
        {
            for (ssize_t i = 0; i < received; i++)
            {
                if (mavlink_parse_char (MAV_RECV_CHANNEL, buf[i], &msg, &status))
                {
                    this->processMavLinkMsg (&msg, &status);
                }
            }
            continue;
        }
        if (received < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
        {
            continue;
        }
        break;
    }
    /* Recv thread cannot tear itself down — flag for the reconnector. */
    this->broken = true;
}

static void
recv_mav_thread (mav_connection *conn)
{
    conn->processMessages ();
}

void
mav_connection::connect_to_mav ()
{
    struct sockaddr_storage remote = {};
    if (!convert_str_to_sa (this->addr, this->port, &remote))
    {
        return;
    }

    /* Build the socket in a local descriptor and only publish it once it is fully
     * connected and configured. A send must never observe a half-open fd (one
     * returned by socket() but not yet connected), and the fd must not become
     * visible — or its number reused — until the previous connection has been
     * fully retired by disconnect_from_mav(). */
    int new_fd = socket (remote.ss_family == AF_INET ? PF_INET : PF_INET6, SOCK_STREAM, IPPROTO_TCP);
    if (new_fd < 0)
    {
        perror ("Failed to create MAV socket");
        return;
    }

    if (connect (new_fd, reinterpret_cast<struct sockaddr *> (&remote),
                 remote.ss_family == AF_INET ? sizeof (struct sockaddr_in) : sizeof (struct sockaddr_in6))
        < 0)
    {
        perror (("Failed to connect to " + this->addr).c_str ());
        close (new_fd);
        return;
    }

    /* Wake recv() periodically so the thread can notice fd being torn down. */
    struct timeval rcv_timeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt (new_fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof (rcv_timeout));

    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->retry_count = 0;
    }
    this->broken = false;
    /* Deliberately do NOT seed last_heartbeat_ts here: a freshly (re)connected
     * socket has not yet produced a heartbeat, so the link stays "down" until the
     * autopilot is actually heard from. heartbeat_loop() owns mav_comms_ok and
     * reports the link up only once a real heartbeat lands. */

    /* Publish the ready fd under send_lock so a concurrent send sees either the
     * old fd, -1, or this fully-connected one — never an intermediate state. */
    {
        std::lock_guard<std::mutex> lk{ this->send_lock };
        this->fd.store (new_fd);
    }
    this->recv_thread = std::thread (recv_mav_thread, this);
}

void
mav_connection::disconnect_from_mav ()
{
    int orig_fd;
    {
        /* Read and retire the fd atomically under send_lock: an in-flight send
         * (which holds send_lock for its whole syscall) completes on the still-open
         * descriptor, every later send short-circuits on fd == -1 rather than
         * landing on a descriptor about to be closed and reused, and the captured
         * orig_fd cannot be a value a concurrent connect_to_mav() is publishing. */
        std::lock_guard<std::mutex> lk{ this->send_lock };
        orig_fd = this->fd.exchange (-1);
    }
    if (orig_fd != -1)
    {
        /* Wake the recv thread out of its blocking recv() so it observes the
         * teardown and exits, without yet closing the descriptor. */
        shutdown (orig_fd, SHUT_RDWR);
    }
    /* Join before close(): closing a descriptor another thread is blocked in
     * recv() on is undefined on Linux, so the recv thread must have left recv()
     * (and returned) before the fd is closed and its number freed for reuse. */
    if (this->recv_thread.joinable ())
    {
        this->recv_thread.join ();
    }
    if (orig_fd != -1)
    {
        close (orig_fd);
    }
}

mav_connection::mav_connection (std::string t_addr, uint16_t t_port, const MavParams &t_params, ILogger &t_logger)
    : addr (std::move (t_addr)), port (t_port), goto_altitude_m (t_params.goto_altitude_m),
      altitude_floor_m (t_params.altitude_floor_m), altitude_cap_m (t_params.altitude_cap_m),
      position_stream_interval_us (t_params.position_stream_interval_us),
      battery_stream_interval_us (t_params.battery_stream_interval_us), logger (t_logger)
{
}

void
mav_connection::start ()
{
    /* Idempotent: a second call must not overwrite the (joinable) heartbeat
     * thread handle — which would std::terminate — nor open a second recv
     * connection that leaks the first. Only the first call does any work. */
    bool expected = false;
    if (!this->started.compare_exchange_strong (expected, true))
    {
        return;
    }
    /* Deferred so the recv and heartbeat threads — which invoke the registered
     * callbacks — do not exist until the owner has finished registering them.
     * Starting them in the constructor would race a recv/heartbeat thread's
     * read of a callback against the main thread still assigning it. */
    this->connect_to_mav ();
    this->heartbeat_thread = std::thread ([this] { this->heartbeat_loop (); });
}

mav_connection::~mav_connection ()
{
    this->stopping = true;
    this->heartbeat_cv.notify_one ();
    if (this->heartbeat_thread.joinable ())
    {
        this->heartbeat_thread.join ();
    }
    /* Tear the link down through the shared path: shutdown + join the recv thread
     * before closing the fd, so teardown is not racy at process exit either. */
    this->disconnect_from_mav ();
}

auto
mav_connection::sendMavLinkMsgLocked (mavlink_message_t *msg) -> bool
{
    int cur_fd = this->fd.load ();
    if (cur_fd == -1)
    {
        /* The link is down: there is nothing to send on. Skip the syscall
         * (send(-1) would only return EBADF), do NOT flag `broken` — there is no
         * live connection to tear down — and stay silent so a disconnected window
         * (cold start, reconnect) does not spam WARN; the heartbeat loop already
         * reports comms-down once. State-machine actions dispatched while the link
         * is down (e.g. the comms-loss failsafe RTL) are therefore best-effort,
         * with ArduPilot's own failsafe as the backstop. */
        return false;
    }
    uint8_t buf[BUFFER_LEN];
    size_t to_send = mavlink_msg_to_send_buffer (buf, msg);
    size_t sent = 0;
    while (sent < to_send)
    {
        ssize_t transfered = send (cur_fd, buf + sent, to_send - sent, 0);
        if (transfered < 0)
        {
            /* Preserve the failure detail before tearing anything down. */
            this->logger.log (LogLevel::error, std::string ("WARN: MAV send() failed: ") + std::strerror (errno));
            /* Only flag the connection broken and let the reconnector tear it
             * down. We must not call disconnect_from_mav() here: sendMavLinkMsg
             * can run on the recv thread (via processMessages), and that path
             * would join the recv thread to itself and deadlock. */
            this->broken = true;
            return false;
        }
        sent += transfered;
    }
    return true;
}

auto
mav_connection::sendMavLinkMsg (mavlink_message_t *msg) -> bool
{
    std::lock_guard<std::mutex> lk (this->send_lock);
    return this->sendMavLinkMsgLocked (msg);
}

void
mav_connection::attemptReconnect ()
{
    constexpr uint64_t msecs_in_sec = 1000;
    if (this->broken)
    {
        this->disconnect_from_mav ();
        this->broken = false;
    }
    if (this->fd.load () == -1)
    {
        uint64_t ts = current_timestamp_ms ();
        bool try_now = false;
        {
            std::lock_guard<std::mutex> lk{ this->state_lock };
            uint64_t elapsed_time = ts - this->last_tried;
            switch (this->retry_count)
            {
                case 0:
                    try_now = (elapsed_time > msecs_in_sec);
                    break;
                case 1:
                    try_now = (elapsed_time > 2 * msecs_in_sec);
                    break;
                case 2:
                    try_now = (elapsed_time > 4 * msecs_in_sec);
                    break;
                case 3:
                    try_now = (elapsed_time > 8 * msecs_in_sec);
                    break;
                case 4:
                    try_now = (elapsed_time > 15 * msecs_in_sec);
                    break;
                default:
                    try_now = (elapsed_time > 30 * msecs_in_sec);
                    break;
            }
            if (try_now)
            {
                this->retry_count++;
                this->last_tried = ts;
            }
        }
        if (try_now)
        {
            this->connect_to_mav ();
        }
    }
}

void
mav_connection::report_battery_status (int8_t remaining, int32_t consumed, double voltage)
{
    if (this->battery_cb)
    {
        this->battery_cb (BatteryData (remaining, consumed, voltage));
    }
}

void
mav_connection::report_position (double lat, double lng, double alt, uint16_t hdg, uint16_t vh, int16_t vv)
{
    if (this->position_cb)
    {
        this->position_cb (PositionData (lat, lng, alt, hdg, vh, vv));
    }
}

void
mav_connection::report_reached (int point)
{
    /* seq 0/1 (< search_first_point_seq) are the setup/takeoff items, which
     * have no corresponding search point and are dropped rather than passed
     * on (see todo/69 for why a goto/RTL's own reached events must also be
     * gated by isSearching() upstream, not just this check). The translation
     * for a real search-point seq is next_search_point_after_reached_seq's
     * "resume from the next point" mapping, not mission_item_for's inverse
     * (todo/64). */
    if (this->reached_cb && point >= search_first_point_seq)
    {
        this->reached_cb (next_search_point_after_reached_seq (static_cast<uint16_t> (point)));
    }
}

void
mav_connection::registerPositionCB (notify_position_cb cb)
{
    this->position_cb = std::move (cb);
}

void
mav_connection::registerReachedCB (notify_reached_cb cb)
{
    this->reached_cb = std::move (cb);
}

void
mav_connection::registerBatteryCB (notify_battery_status_cb cb)
{
    this->battery_cb = std::move (cb);
}

void
mav_connection::registerMavCommsStatusCB (notify_mav_comms_cb cb)
{
    this->mav_comms_cb = std::move (cb);
}
