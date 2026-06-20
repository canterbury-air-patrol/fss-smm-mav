#include "internal.hpp"
#include "mav.hpp"
#include "util.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
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

constexpr uint8_t TARGET_SYS_ID = 1;

/* Convert double/float into int32_t */
constexpr double LAT_LNG_COV = 0.0000001;
/* Convert ft into mm */
constexpr double ALT_COV = 304.8;

void
mav_connection::sendHeartBeat ()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack (SYS_ID, COMP_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::heartbeat_loop ()
{
    constexpr uint64_t heartbeat_timeout_ms = 5000;
    while (!this->stopping)
    {
        if (this->fd.load () != -1)
        {
            this->sendHeartBeat ();
            uint64_t ts = this->last_heartbeat_ts.load ();
            bool timed_out = (current_timestamp_ms () - ts) > heartbeat_timeout_ms;
            if (timed_out)
            {
                bool expected = true;
                if (this->mav_comms_ok.compare_exchange_strong (expected, false))
                {
                    std::cerr << "WARN: Autopilot heartbeat timeout — MAV comms failure\n";
                    if (this->mav_comms_cb)
                        this->mav_comms_cb (MavCommsStatus::failure);
                }
            }
        }
        std::unique_lock<std::mutex> lk (this->heartbeat_mutex);
        this->heartbeat_cv.wait_for (lk, std::chrono::seconds (1), [this] { return this->stopping.load (); });
    }
}

void
mav_connection::sendADSB (uint32_t icao_address, double lat, double lng, uint32_t altitude, uint8_t altitude_type,
                          uint16_t heading, uint16_t hor_vel, uint16_t ver_vel, char *callsign, uint8_t emitter_type,
                          uint8_t tslc, uint16_t flags, uint16_t squawk)
{
    mavlink_message_t msg;
    mavlink_msg_adsb_vehicle_pack (SYS_ID, COMP_ID, &msg, icao_address, static_cast<int32_t> (lat / LAT_LNG_COV),
                                   static_cast<int32_t> (lng / LAT_LNG_COV), altitude_type,
                                   static_cast<int32_t> (altitude * ALT_COV), heading, hor_vel,
                                   static_cast<int16_t> (ver_vel), callsign, emitter_type, tslc, flags, squawk);
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::setFlightMode (uint8_t fmode)
{
    auto sys = this->systems.findSystem (TARGET_SYS_ID);
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack (SYS_ID, COMP_ID, &msg, 1,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED
                                   | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED
                                   | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED,
                               fmode);
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::commandRTL ()
{
    /* Map type to RTL mode */
    auto sys = this->systems.findSystem (TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType ())
    {
        case MAV_TYPE_FIXED_WING:
            fmode = PLANE_MODE_RTL;
            break;
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
            fmode = COPTER_MODE_RTL;
            break;
        case MAV_TYPE_GROUND_ROVER:
            fmode = ROVER_MODE_RTL;
            break;
        default:
            fmode = 0;
            break;
    }
    if (fmode != 0)
    {
        this->setFlightMode (fmode);
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
}

void
mav_connection::commandDisARM ()
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack (SYS_ID, COMP_ID, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
    this->sendMavLinkMsg (&msg);
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
}

void
mav_connection::commandGoto (Point p)
{
    mavlink_message_t msg;
    // RTL the aircraft so we can load a mission
    this->commandRTL ();
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->goto_position = p;
        this->goto_active = true;
        this->search_loaded = false;
        mavlink_msg_mission_count_pack (SYS_ID, COMP_ID, &msg, 0, 1, 3, MAV_MISSION_TYPE_MISSION, 0);
    }
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::commandManual ()
{
    /* Map type to RTL mode */
    auto sys = this->systems.findSystem (TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType ())
    {
        case MAV_TYPE_FIXED_WING:
            fmode = PLANE_MODE_FLY_BY_WIRE_B;
            break;
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
            fmode = COPTER_MODE_STABILIZE;
            break;
        case MAV_TYPE_GROUND_ROVER:
            fmode = ROVER_MODE_MANUAL;
            break;
        default:
            fmode = 0;
            break;
    }
    if (fmode != 0)
    {
        this->setFlightMode (fmode);
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
}

void
mav_connection::commandHold ()
{
    /* Map type to LOITER/HOLD mode */
    auto sys = this->systems.findSystem (TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType ())
    {
        case MAV_TYPE_FIXED_WING:
            fmode = PLANE_MODE_LOITER;
            break;
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
            fmode = COPTER_MODE_POSHOLD;
            break;
        case MAV_TYPE_GROUND_ROVER:
            fmode = ROVER_MODE_HOLD;
            break;
        default:
            fmode = 0;
            break;
    }
    if (fmode != 0)
    {
        this->setFlightMode (fmode);
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
}

void
mav_connection::commandAuto ()
{
    /* Map type to AUTO mode */
    auto sys = this->systems.findSystem (TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType ())
    {
        case MAV_TYPE_FIXED_WING:
            fmode = PLANE_MODE_AUTO;
            break;
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
            fmode = COPTER_MODE_AUTO;
            break;
        case MAV_TYPE_GROUND_ROVER:
            fmode = ROVER_MODE_AUTO;
            break;
        default:
            fmode = 0;
            break;
    }
    if (fmode != 0)
    {
        this->setFlightMode (fmode);
    }
}

void
mav_connection::commandAltitude (uint16_t alt)
{
    mavlink_message_t msg;
    /* Map feet to meters for MAVLink */
    constexpr float FEET_TO_METERS = 0.3048f;
    float alt_m = static_cast<float> (alt) * FEET_TO_METERS;

    /* MAV_CMD_DO_CHANGE_ALTITUDE:
       Param 1: Altitude (float, meters)
       Param 2: Frame (MAV_FRAME_GLOBAL_RELATIVE_ALT = 3)
    */
    mavlink_msg_command_long_pack (SYS_ID, COMP_ID, &msg, TARGET_SYS_ID, 1, MAV_CMD_DO_CHANGE_ALTITUDE, 0, alt_m,
                                   MAV_FRAME_GLOBAL_RELATIVE_ALT, 0, 0, 0, 0, 0);
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::commandForceDisARM ()
{
    constexpr float force_magic = 21196.0f;
    mavlink_message_t msg;
    mavlink_msg_command_long_pack (SYS_ID, COMP_ID, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, force_magic, 0, 0,
                                   0, 0, 0);
    this->sendMavLinkMsg (&msg);
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
}

void
mav_connection::commandTerminate ()
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack (SYS_ID, COMP_ID, &msg, 1, 1, MAV_CMD_DO_FLIGHTTERMINATION, 0, 1, 0, 0, 0, 0, 0, 0);
    this->sendMavLinkMsg (&msg);
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->search_loaded = false;
    }
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
    if (local_goto_active)
    {
        if (seq >= 2)
        {
            mavlink_msg_mission_item_int_pack (
                SYS_ID, COMP_ID, &msg, 0, 1, seq, /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT,    /* Use altitude relative to the home point */
                MAV_CMD_NAV_RETURN_TO_LAUNCH,     /* Return home */
                0,                                /* Not the current point */
                0,                                /* Auto continue: No */
                0, 0, 0, 0, 0, 0, 0,              /* Parameters ignored */
                mission_type);
        }
        else
        {
            /* The scaled int32 lat/lon legitimately sit next to the float altitude in this MAVLink message;
             * clang-tidy's swapped-arguments heuristic cannot tell them apart. */
            // NOLINTNEXTLINE(bugprone-swapped-arguments)
            mavlink_msg_mission_item_int_pack (
                SYS_ID, COMP_ID, &msg, 0, 1, seq, /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT,    /* Use altitude relative to the home point */
                MAV_CMD_NAV_WAYPOINT,             /* Navigate to a point */
                0,                                /* This waypoint is the current target */
                1,                                /* Auto continue */
                0,                                /* Hold time: 0s */
                acceptable_radius,                /* Accept radius: m */
                0,                                /* Pass radius: 0m */
                NAN,                              /* Yaw: NaN for dont care */
                static_cast<int32_t> (local_goto_position.getLatitude () / LAT_LNG_COV),  /* Latitude */
                static_cast<int32_t> (local_goto_position.getLongitude () / LAT_LNG_COV), /* Longitude */
                static_cast<float> (this->goto_altitude_m),                               /* Altitude (m AGL) */
                mission_type);
        }
    }
    else
    {
        if (local_search == nullptr)
        {
            return;
        }
        /* Find the point */
        auto points = local_search->getPoints ();
        if (seq == 0 || seq == 1)
        {
            /* Most versions of ArduPilot ignore the zeroth mission command, so we need to send the first one twice */
            mavlink_msg_mission_item_int_pack (
                SYS_ID, COMP_ID, &msg, 0, 1, seq,                       /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT,                          /* Use altitude relative to the home point */
                MAV_CMD_NAV_TAKEOFF,                                    /* Return home */
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
        }
        else if (seq > (points.size () + 1))
        {
            /* Make sure the mission defaults to ending with sending the asset home */
            mavlink_msg_mission_item_int_pack (
                SYS_ID, COMP_ID, &msg, 0, 1, seq, /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT,    /* Use altitude relative to the home point */
                MAV_CMD_NAV_RETURN_TO_LAUNCH,     /* Return home */
                0,                                /* Not the current point */
                0,                                /* Auto continue: No */
                0, 0, 0, 0, 0, 0, 0,              /* Parameters ignored */
                mission_type);
        }
        else
        {
            /* Load each point of the search, the first 2 mission items are setup, so the seq is offset */
            Point p = points[seq - 2];
            /* The scaled int32 lat/lon legitimately sit next to the float altitude in this MAVLink message;
             * clang-tidy's swapped-arguments heuristic cannot tell them apart. */
            // NOLINTNEXTLINE(bugprone-swapped-arguments)
            mavlink_msg_mission_item_int_pack (
                SYS_ID, COMP_ID, &msg, 0, 1, seq,                       /* Which waypoint is this */
                MAV_FRAME_GLOBAL_RELATIVE_ALT,                          /* Use altitude relative to the home point */
                MAV_CMD_NAV_WAYPOINT,                                   /* Navigate to a point */
                (local_search->getCurrentPointIdx () == (seq - 2)),     /* Is this waypoint is the current target? */
                1,                                                      /* Auto continue */
                0,                                                      /* Hold time: 0s */
                acceptable_radius,                                      /* Accept radius: m */
                0,                                                      /* Pass radius: 0m */
                NAN,                                                    /* Yaw: NaN for dont care */
                static_cast<int32_t> (p.getLatitude () / LAT_LNG_COV),  /* Latitude */
                static_cast<int32_t> (p.getLongitude () / LAT_LNG_COV), /* Longitude */
                static_cast<float> (local_search->getAltitude ()),      /* Altitude (m) */
                mission_type);
        }
    }
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::setCurrentWP (uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_set_current_pack (SYS_ID, COMP_ID, &msg, 0, 1, seq);
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::mission_ack (bool accepted)
{
    bool goto_set_current = false;
    bool search_set_current = false;
    uint16_t search_seq = 0;
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
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
                    search_seq = this->search->getCurrentPointIdx ();
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
        this->commandAuto ();
        this->setCurrentWP (0);
    }
    if (search_set_current)
    {
        this->commandAuto ();
        this->setCurrentWP (search_seq);
    }
}

void
mav_connection::loadSearch ()
{
    /* Enter RTL while loading the search */
    this->commandRTL ();
    /* Load the existing search into the FC, and jump to the current target point */
    /* Tell the FC how many points there are (+2 slots for setup, +1 for RTL) */
    mavlink_message_t msg;
    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->goto_active = false;
        this->search_loaded = false;
        this->search_loading = true;
        mavlink_msg_mission_count_pack (SYS_ID, COMP_ID, &msg, 0, 1, this->search->getPoints ().size () + 2,
                                        MAV_MISSION_TYPE_MISSION, 0);
    }
    this->sendMavLinkMsg (&msg);
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
    mavlink_msg_command_long_pack (SYS_ID, COMP_ID, &msg, sysid, compid, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   static_cast<float> (command), static_cast<float> (interval), 0, 0, 0, 0, 1);
    this->sendMavLinkMsg (&msg);
}

void
mav_connection::processMavLinkMsg (mavlink_message_t *msg, mavlink_status_t *status __attribute__ ((unused)))
{
    auto sys = this->systems.findSystem (msg->sysid);
    auto comp = sys->findComponent (msg->compid);

    if (!sys->isSetup ())
    {
        constexpr int position_rate = 200000;
        constexpr int battery_rate = 1000000;
        /* Request current position at a rate of 5 per second */
        this->requestStream (msg->sysid, msg->compid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, position_rate);
        /* Request battery status every 1 second */
        this->requestStream (msg->sysid, msg->compid, MAVLINK_MSG_ID_BATTERY_STATUS, battery_rate);
        sys->setupComplete ();
    }

    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            this->last_heartbeat_ts.store (current_timestamp_ms ());
            bool expected = false;
            if (this->mav_comms_ok.compare_exchange_strong (expected, true))
            {
                if (this->mav_comms_cb)
                    this->mav_comms_cb (MavCommsStatus::ok);
            }
            sys->setAutoPilotMode (mavlink_msg_heartbeat_get_type (msg));
            sys->setFlightMode (mavlink_msg_heartbeat_get_custom_mode (msg));
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
            uint16_t vh = static_cast<uint16_t> (sqrt ((vx * vx) + (vy * vy)));
            {
                std::lock_guard<std::mutex> lk (this->state_lock);
                this->last_position = Point (latd, lngd);
            }
            this->report_position (latd, lngd, (alt / ALT_COV), heading, vh, vz);
        }
        break;
        case MAVLINK_MSG_ID_BATTERY_STATUS:
        {
            /* Battery Status */
            int32_t current_consumed = mavlink_msg_battery_status_get_current_consumed (msg);
            int8_t remaining = mavlink_msg_battery_status_get_battery_remaining (msg);
            uint16_t voltages[10] = { 0 };
            mavlink_msg_battery_status_get_voltages (msg, voltages);
            this->report_battery_status (remaining, current_consumed, (double)voltages[0] / 1000.0);
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
            if (mavlink_msg_mission_request_get_target_system (msg) == SYS_ID
                && mavlink_msg_mission_request_get_target_component (msg) == COMP_ID)
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
            std::cout << "Status: ";
            char text_buf[BUFFER_LEN];
            mavlink_msg_statustext_get_text (msg, text_buf);
            std::cout << text_buf << '\n';
        }
        break;
        default:
            std::cout << '\n'
                      << "Received packet: SYS: " << (short)msg->sysid << ", COMP: " << (short)msg->compid
                      << ", LEN: " << (short)msg->len << ", MSG ID: " << msg->msgid << '\n';
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
                if (mavlink_parse_char (MAVLINK_COMM_0, buf[i], &msg, &status))
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

    this->fd.store (socket (remote.ss_family == AF_INET ? PF_INET : PF_INET6, SOCK_STREAM, IPPROTO_TCP));

    if (connect (this->fd.load (), reinterpret_cast<struct sockaddr *> (&remote),
                 remote.ss_family == AF_INET ? sizeof (struct sockaddr_in) : sizeof (struct sockaddr_in6))
        < 0)
    {
        perror (("Failed to connect to " + this->addr).c_str ());
        close (this->fd.load ());
        this->fd.store (-1);
        return;
    }

    /* Wake recv() periodically so the thread can notice fd being torn down. */
    struct timeval rcv_timeout = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt (this->fd.load (), SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof (rcv_timeout));

    {
        std::lock_guard<std::mutex> lk{ this->state_lock };
        this->retry_count = 0;
    }
    this->broken = false;
    this->last_heartbeat_ts.store (current_timestamp_ms ());
    this->mav_comms_ok.store (false);

    this->recv_thread = std::thread (recv_mav_thread, this);
}

void
mav_connection::disconnect_from_mav ()
{
    if (this->fd.load () != -1)
    {
        close (this->fd.load ());
        this->fd.store (-1);
    }
    if (this->recv_thread.joinable ())
    {
        this->recv_thread.join ();
    }
}

mav_connection::mav_connection (std::string t_addr, uint16_t t_port, uint16_t t_goto_altitude_m)
    : addr (std::move (t_addr)), port (t_port), goto_altitude_m (t_goto_altitude_m)
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
    if (this->fd.load () != -1)
    {
        int orig_fd = this->fd.load ();
        this->fd.store (-1);
        shutdown (orig_fd, 2);
        close (orig_fd);
    }
    if (this->recv_thread.joinable ())
    {
        this->recv_thread.join ();
    }
}

auto
mav_connection::sendMavLinkMsg (mavlink_message_t *msg) -> bool
{
    std::lock_guard<std::mutex> lk (this->send_lock);
    uint8_t buf[BUFFER_LEN];
    size_t to_send = mavlink_msg_to_send_buffer (buf, msg);
    size_t sent = 0;
    while (sent < to_send)
    {
        ssize_t transfered = send (this->fd.load (), buf + sent, to_send - sent, 0);
        if (transfered < 0)
        {
            /* Preserve the failure detail before tearing anything down. */
            std::cerr << "WARN: MAV send() failed: " << std::strerror (errno) << "\n";
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
    if (this->reached_cb && point > 1)
    {
        this->reached_cb (point - 1);
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
