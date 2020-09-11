#include "internal.hpp"
#include "mav.hpp"

#include <bits/stdint-intn.h>
#include <iostream>
#include <mutex>
#include <thread>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/time.h>

#include <ardupilotmega/mavlink.h>

#define BUFFER_LENGTH 2048

#define SYS_ID 200
#define COMP_ID 1

#define TARGET_SYS_ID 1

/* Convert double/float into int32_t */
#define LAT_LNG_COV 0.0000001
/* Convert ft into mm */
#define ALT_COV 304.8

void
mav_connection::sendHeartBeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    this->sendMavLinkMsg(&msg);
}

void
mav_connection::sendADSB(uint32_t icao_address, double lat, double lng, uint32_t altitude, uint8_t altitude_type, uint16_t heading, uint16_t hor_vel, uint16_t ver_vel, char *callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
    mavlink_message_t msg;
    mavlink_msg_adsb_vehicle_pack(SYS_ID, COMP_ID, &msg, icao_address, lat / LAT_LNG_COV, lng / LAT_LNG_COV, altitude_type, altitude * ALT_COV, heading, hor_vel, ver_vel, callsign, emitter_type, tslc, flags, squawk);
    this->sendMavLinkMsg(&msg);
}


void
mav_connection::setFlightMode(uint8_t fmode)
{
    auto sys = this->systems.findSystem(TARGET_SYS_ID);
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(SYS_ID, COMP_ID, &msg, 1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED, fmode);
    this->sendMavLinkMsg(&msg);
}

void
mav_connection::commandRTL()
{
    /* Map type to RTL mode */
    auto sys = this->systems.findSystem(TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType())
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
        this->setFlightMode(fmode);
        this->search_loaded = false;
    }
}

void
mav_connection::commandDisARM()
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
    this->sendMavLinkMsg(&msg);
    this->search_loaded = false;
}

void
mav_connection::commandGoto(Point p)
{
    mavlink_message_t msg;
    // RTL the aircraft so we can load a mission
    this->commandRTL();
    this->goto_position = p;
    this->goto_active = true;
    mavlink_msg_mission_count_pack(SYS_ID, COMP_ID, &msg, 0, 1, 3, MAV_MISSION_TYPE_MISSION);
    this->sendMavLinkMsg(&msg);
    this->search_loaded = false;
}

void
mav_connection::commandManual()
{
    /* Map type to RTL mode */
    auto sys = this->systems.findSystem(TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType())
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
        this->setFlightMode(fmode);
        this->search_loaded = false;
    }

}

void
mav_connection::commandHold()
{
    /* Map type to LOITER/HOLD mode */
    auto sys = this->systems.findSystem(TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType())
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
        this->setFlightMode(fmode);
        this->search_loaded = false;
    }
}

void
mav_connection::commandAuto()
{
    /* Map type to AUTO mode */
    auto sys = this->systems.findSystem(TARGET_SYS_ID);
    uint8_t fmode = 0;
    switch (sys->getAutoPilotType())
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
        this->setFlightMode(fmode);
    }
}

void
mav_connection::commandTerminate()
{
    /* Do nothing for now */
    /* TODO: Implement terminate */
    this->search_loaded = false;
}

void
mav_connection::send_waypoint(uint16_t seq, uint8_t mission_type)
{
    mavlink_message_t msg;
    if (this->goto_active)
    {
        if (seq >= 2)
        {
            mavlink_msg_mission_item_int_pack (SYS_ID, COMP_ID, &msg, 0, 1,
                        seq, /* Which waypoint is this */
                        MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Use altitude relative to the home point */
                        MAV_CMD_NAV_RETURN_TO_LAUNCH, /* Return home */
                        0, /* Not the current point */
                        0, /* Auto continue: No */
                        0, 0, 0, 0, 0, 0, 0, /* Parameters ignored */
                        mission_type);
        }
        else
        {
            mavlink_msg_mission_item_int_pack (SYS_ID, COMP_ID, &msg, 0, 1,
                    seq, /* Which waypoint is this */
                    MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Use altitude relative to the home point */
                    MAV_CMD_NAV_WAYPOINT, /* Navigate to a point */
                    0, /* This waypoint is the current target */
                    1, /* Auto continue */
                    0, /* Hold time: 0s */
                    5, /* Accept radius: 5m */
                    0, /* Pass radius: 0m */
                    NAN, /* Yaw: NaN for dont care */
                    this->goto_position.getLatitude() / 0.0000001, /* Latitude */
                    this->goto_position.getLongitude() / 0.0000001, /* Longitude */
                    50,  /* Altitude (m) */
                    mission_type);
        }
    }
    else
    {
        if (this->search == nullptr)
        {
            return;
        }
        /* Find the point */
        auto points = this->search->getPoints();
        if (seq == 0 || seq > points.size())
        {
            mavlink_msg_mission_item_int_pack (SYS_ID, COMP_ID, &msg, 0, 1,
                        seq, /* Which waypoint is this */
                        MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Use altitude relative to the home point */
                        MAV_CMD_NAV_RETURN_TO_LAUNCH, /* Return home */
                        0, /* Not the current point */
                        0, /* Auto continue: No */
                        0, 0, 0, 0, 0, 0, 0, /* Parameters ignored */
                        mission_type);
        }
        else
        {
            Point p = points[seq-1];
            mavlink_msg_mission_item_int_pack (SYS_ID, COMP_ID, &msg, 0, 1,
                        seq, /* Which waypoint is this */
                        MAV_FRAME_GLOBAL_RELATIVE_ALT, /* Use altitude relative to the home point */
                        MAV_CMD_NAV_WAYPOINT, /* Navigate to a point */
                        0, /* This waypoint is the current target */
                        1, /* Auto continue */
                        0, /* Hold time: 0s */
                        5, /* Accept radius: 5m */
                        0, /* Pass radius: 0m */
                        NAN, /* Yaw: NaN for dont care */
                        p.getLatitude() / 0.0000001, /* Latitude */
                        p.getLongitude() / 0.0000001, /* Longitude */
                        this->search->getAltitude(),  /* Altitude (m) */
                        mission_type);
        }
    }
    this->sendMavLinkMsg(&msg);
}

void
mav_connection::setCurrentWP(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_set_current_pack(SYS_ID, COMP_ID, &msg, 0, 1, seq);
    this->sendMavLinkMsg(&msg);
}

void
mav_connection::mission_ack(bool accepted)
{
    if (this->goto_active)
    {
        if (accepted)
        {
            this->commandAuto();
            this->setCurrentWP(0);
        }
    }
    if (this->search_loading)
    {
        this->search_loading = false;
        if (accepted)
        {
            this->search_loaded = true;
            this->commandAuto();
            this->setCurrentWP(this->search->getCurrentPointIdx());
        }
        else
        {
            this->search = nullptr;
        }
    }
}

void
mav_connection::loadSearch()
{
    /* Enter RTL while loading the search */
    this->commandRTL();
    /* Load the existing search into the FC, and jump to the current target point */
    /* Tell the FC how many points there are */
    this->goto_active = false;
    this->search_loaded = false;
    this->search_loading = true;
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(SYS_ID, COMP_ID, &msg, 0, 1, this->search->getPoints().size() + 1, MAV_MISSION_TYPE_MISSION);
    this->sendMavLinkMsg(&msg);
}

void
mav_connection::loadSearch(SMMSearch *search)
{
    if (this->search != search || !this->search_loaded)
    {
        this->search = search;
        if (this->search != nullptr)
        {
            this->loadSearch();
        }
    }
}

void
mav_connection::requestStream(int sysid, int compid, uint32_t command, uint32_t interval)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, sysid, compid, MAV_CMD_SET_MESSAGE_INTERVAL, 0, command, interval, 0, 0, 0, 0, 1);
    this->sendMavLinkMsg(&msg);
}

void
mav_connection::processMavLinkMsg(mavlink_message_t *msg, mavlink_status_t *status)
{
    auto sys = this->systems.findSystem(msg->sysid);
    auto comp = sys->findComponent(msg->compid);

    if (!sys->isSetup())
    {
        /* Request current position at a rate of 5 per second */
        this->requestStream(msg->sysid, msg->compid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000);
        /* Request battery status every 1 second */
        this->requestStream(msg->sysid, msg->compid, MAVLINK_MSG_ID_BATTERY_STATUS, 1000000);
        sys->setupComplete();
    }

    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            /* Update the type and flight mode */
            sys->setAutoPilotMode(mavlink_msg_heartbeat_get_type(msg));
            sys->setFlightMode(mavlink_msg_heartbeat_get_custom_mode(msg));
        }
        break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            {
                /* New position data */
                int32_t lat = mavlink_msg_global_position_int_get_lat(msg);
                int32_t lng = mavlink_msg_global_position_int_get_lon(msg);
                int32_t alt = mavlink_msg_global_position_int_get_alt(msg);
                double latd = ((double)lat * LAT_LNG_COV);
                double lngd = ((double)lng * LAT_LNG_COV);
                uint16_t heading = mavlink_msg_global_position_int_get_hdg(msg);
                int16_t vx = mavlink_msg_global_position_int_get_vx(msg);
                int16_t vy = mavlink_msg_global_position_int_get_vy(msg);
                int16_t vz = mavlink_msg_global_position_int_get_vz(msg);
                uint16_t vh = sqrt((vx * vx) + (vy * vy));
                {
                    std::lock_guard<std::mutex> lk(this->position_lock);
                    this->last_position = Point(latd, lngd);
                }
                this->report_position(latd, lngd, (alt / ALT_COV), heading, vh, vz);
        } break;
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            {
                /* Battery Status */
                int32_t current_consumed = mavlink_msg_battery_status_get_current_consumed(msg);
                int8_t remaining = mavlink_msg_battery_status_get_battery_remaining(msg);
                this->report_battery_status(remaining, current_consumed);
            } break;
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
            {
                /* Reached a new waypoint, update the current search progress */
                uint16_t seq = mavlink_msg_mission_item_reached_get_seq(msg);
                this->report_reached(seq);
            } break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
            {
                if (mavlink_msg_mission_request_get_target_system(msg) == SYS_ID && mavlink_msg_mission_request_get_target_component(msg) == COMP_ID)
                {
                    /* Getting asked for a specific point in mission */
                    uint16_t seq = mavlink_msg_mission_request_get_seq(msg);
                    uint8_t mt = mavlink_msg_mission_request_get_mission_type(msg);
                    this->send_waypoint(seq, mt);
                }
            } break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            {
                if (mavlink_msg_mission_request_get_target_system(msg) == SYS_ID && mavlink_msg_mission_request_get_target_component(msg) == COMP_ID)
                {
                    /* Getting asked for a specific point in mission */
                    uint16_t seq = mavlink_msg_mission_request_int_get_seq(msg);
                    uint8_t mt = mavlink_msg_mission_request_int_get_mission_type(msg);
                    this->send_waypoint(seq, mt);
                }
            } break;
        case MAVLINK_MSG_ID_MISSION_ACK:
            {
                if (mavlink_msg_mission_ack_get_target_system(msg) == SYS_ID && mavlink_msg_mission_ack_get_target_component(msg) == COMP_ID)
                {
                    /* Our mission was acknowledged */
                    this->mission_ack(mavlink_msg_mission_ack_get_type(msg) == MAV_MISSION_ACCEPTED);
                }
            } break;
        case MAVLINK_MSG_ID_ADSB_VEHICLE:
        case MAVLINK_MSG_ID_COLLISION:
        case MAVLINK_MSG_ID_COMMAND_ACK:
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_PARAM_VALUE:
        case MAVLINK_MSG_ID_TIMESYNC:
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
            break;
        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            std::cout << "Status: ";
            char text_buf[BUFFER_LENGTH];
            mavlink_msg_statustext_get_text(msg, text_buf);
            std::cout << text_buf << std::endl;
        }    break;
        default:
            printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg->sysid, msg->compid, msg->len, msg->msgid);
    }
}

bool
convert_str_to_sa(std::string addr, uint16_t port, struct sockaddr_storage *sa)
{
    int family = AF_UNSPEC;
    /* Try converting an IP(v4) address first */
    if (family == AF_UNSPEC)
    {
        struct in_addr ia;
        if (inet_pton(AF_INET, addr.c_str(), &ia) == 1)
        {
            family = AF_INET;
            struct sockaddr_in *sa_in = (struct sockaddr_in *)sa;
            memset(sa_in, 0, sizeof(struct sockaddr_in));
            sa_in->sin_family = AF_INET;
            sa_in->sin_addr = ia;
        }
    }
    /* Try converting an IPv6 address */
    if (family == AF_UNSPEC)
    {
        struct in6_addr ia;
        if (inet_pton (AF_INET6, addr.c_str(), &ia) == 1)
        {
            family = AF_INET6;
            struct sockaddr_in6 *sa_in = (struct sockaddr_in6 *)sa;
            memset(sa_in, 0, sizeof(struct sockaddr_in6));
            sa_in->sin6_family = AF_INET6;
            sa_in->sin6_addr = ia;
        }
    }
    /* Use host name lookup (probably DNS) to resolve the name */
    if (family == AF_UNSPEC)
    {
        struct addrinfo *ai = nullptr;
        
        if (getaddrinfo(addr.c_str(), nullptr, nullptr, &ai) == 0)
        {
            memcpy (sa, ai->ai_addr, ai->ai_addrlen);
            family = ai->ai_family;
        }
        
        freeaddrinfo(ai);
    }

    switch (family)
    {
        case AF_INET:
        {
            struct sockaddr_in *sa_in = (struct sockaddr_in *)sa;
            sa_in->sin_port = ntohs (port);
        } break;
        case AF_INET6:
        {
            struct sockaddr_in6 *sa_in = (struct sockaddr_in6 *)sa;
            sa_in->sin6_port = ntohs (port);
        }
    }
    
    return family != AF_UNSPEC;
}

void
mav_connection::processMessages()
{
    while (this->fd != -1)
    {
        char buf[BUFFER_LENGTH];
        mavlink_message_t msg;
        mavlink_status_t status;
        ssize_t received = recv(this->fd, buf, sizeof(buf), 0);
        if (received > 0)
        {
            for (size_t i = 0; i < BUFFER_LENGTH; i++)
            {
                if (mavlink_parse_char (MAVLINK_COMM_0, buf[i], &msg, &status))
                {
                    this->processMavLinkMsg(&msg, &status);
                }
            }
        }
    }
}

static void
recv_mav_thread(mav_connection *conn)
{
    conn->processMessages();
}

void
mav_connection::connect_to_mav()
{
    struct sockaddr_storage remote;
    if (!convert_str_to_sa(this->addr, this->port, &remote))
    {
        return;
    }

    this->fd = socket(remote.ss_family == AF_INET ? PF_INET : PF_INET6, SOCK_STREAM, IPPROTO_TCP);

    if (connect(this->fd, (struct sockaddr *)&remote, remote.ss_family == AF_INET ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6)) < 0)
    {
        perror(("Failed to connect to " + this->addr).c_str());
        this->fd = -1;
        return;
    }

    this->retry_count = 0;

    this->recv_thread = std::thread(recv_mav_thread, this);
}

void
mav_connection::disconnect_from_mav()
{
    if (this->fd != -1)
    {
        close(this->fd);
        this->fd = -1;
    }
    if(this->recv_thread.joinable())
    {
        this->recv_thread.join();
    }
}

mav_connection::mav_connection(std::string t_addr, uint16_t t_port) : addr(t_addr), port(t_port)
{
    this->connect_to_mav();
}

mav_connection::~mav_connection()
{
    if (this->fd != -1)
    {
        uint32_t orig_fd = this->fd;
        this->fd = -1;
        shutdown (orig_fd, 2);
        close (orig_fd);
    }
    if (this->recv_thread.joinable())
    {
        this->recv_thread.join();
    }
}


bool
mav_connection::sendMavLinkMsg(mavlink_message_t *msg)
{
    this->send_lock.lock();
    uint8_t buf[BUFFER_LENGTH];
    size_t to_send = mavlink_msg_to_send_buffer(buf, msg);
    size_t sent = 0;
    while (sent < to_send)
    {
        ssize_t transfered = send(this->fd, buf + sent, to_send - sent, 0);
        if (transfered < 0)
        {
            this->send_lock.unlock();
            this->disconnect_from_mav();
            return false;
        }
        sent += transfered;
    }
    this->send_lock.unlock();
    return true;
}

static uint64_t
current_timestamp()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000 + (tv.tv_usec / 1000);
}

void
mav_connection::attemptReconnect()
{
    if (this->fd == -1)
    {
        uint64_t ts = current_timestamp();
        bool try_now = false;
        uint64_t elapsed_time = ts - this->last_tried;
        switch (this->retry_count)
        {
            case 0:
                try_now = (elapsed_time > 1000);
                break;
            case 1:
                try_now = (elapsed_time > 2000);
                break;
            case 2:
                try_now = (elapsed_time > 4000);
                break;
            case 3:
                try_now = (elapsed_time > 8000);
                break;
            case 4:
                try_now = (elapsed_time > 15000);
                break;
            default:
                try_now = (elapsed_time > 30000);
                break;
        }
        if (try_now)
        {
            this->retry_count++;
            this->last_tried = ts;
            this->connect_to_mav();
        }
    }

}


void mav_connection::report_battery_status(int8_t remaining, int32_t consumed)
{
    if (this->battery_cb != nullptr)
    {
        this->battery_cb(this->battery_cb_priv, remaining, consumed);
    }
}

void mav_connection::report_position(double lat, double lng, double alt, uint16_t hdg, uint16_t vh, int16_t vv)
{
    if (this->position_cb != nullptr)
    {
        this->position_cb(this->position_cb_priv, PositionData(lat, lng, alt, hdg, vh, vv));
    }
}

void mav_connection::report_reached(int point)
{
    if (this->reached_cb != nullptr)
    {
        this->reached_cb(this->reached_cb_priv, point);
    }
}

void mav_connection::registerPositionCB(notify_position_cb cb, void *priv)
{
    this->position_cb_priv = priv;
    this->position_cb = cb;
}

void mav_connection::registerReachedCB(notify_reached_cb cb, void *priv)
{
    this->reached_cb_priv = priv;
    this->reached_cb = cb;
}