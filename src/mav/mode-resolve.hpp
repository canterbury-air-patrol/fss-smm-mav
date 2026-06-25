#pragma once

#include <cstdint>
#include <optional>

#include <ardupilotmega/mavlink.h>

enum class MavModeCommand
{
    rtl,
    manual,
    hold,
    auto_mode,
};

inline auto
resolve_mav_mode (uint8_t autopilot_type, MavModeCommand command) -> std::optional<uint8_t>
{
    switch (command)
    {
        case MavModeCommand::rtl:
            switch (autopilot_type)
            {
                case MAV_TYPE_FIXED_WING:
                    return PLANE_MODE_RTL;
                case MAV_TYPE_QUADROTOR:
                case MAV_TYPE_COAXIAL:
                case MAV_TYPE_HELICOPTER:
                case MAV_TYPE_HEXAROTOR:
                case MAV_TYPE_OCTOROTOR:
                case MAV_TYPE_TRICOPTER:
                    return COPTER_MODE_RTL;
                case MAV_TYPE_GROUND_ROVER:
                    return ROVER_MODE_RTL;
                default:
                    return std::nullopt;
            }
        case MavModeCommand::manual:
            switch (autopilot_type)
            {
                case MAV_TYPE_FIXED_WING:
                    return PLANE_MODE_FLY_BY_WIRE_B;
                case MAV_TYPE_QUADROTOR:
                case MAV_TYPE_COAXIAL:
                case MAV_TYPE_HELICOPTER:
                case MAV_TYPE_HEXAROTOR:
                case MAV_TYPE_OCTOROTOR:
                case MAV_TYPE_TRICOPTER:
                    return COPTER_MODE_STABILIZE;
                case MAV_TYPE_GROUND_ROVER:
                    return ROVER_MODE_MANUAL;
                default:
                    return std::nullopt;
            }
        case MavModeCommand::hold:
            switch (autopilot_type)
            {
                case MAV_TYPE_FIXED_WING:
                    return PLANE_MODE_LOITER;
                case MAV_TYPE_QUADROTOR:
                case MAV_TYPE_COAXIAL:
                case MAV_TYPE_HELICOPTER:
                case MAV_TYPE_HEXAROTOR:
                case MAV_TYPE_OCTOROTOR:
                case MAV_TYPE_TRICOPTER:
                    return COPTER_MODE_POSHOLD;
                case MAV_TYPE_GROUND_ROVER:
                    return ROVER_MODE_HOLD;
                default:
                    return std::nullopt;
            }
        case MavModeCommand::auto_mode:
            switch (autopilot_type)
            {
                case MAV_TYPE_FIXED_WING:
                    return PLANE_MODE_AUTO;
                case MAV_TYPE_QUADROTOR:
                case MAV_TYPE_COAXIAL:
                case MAV_TYPE_HELICOPTER:
                case MAV_TYPE_HEXAROTOR:
                case MAV_TYPE_OCTOROTOR:
                case MAV_TYPE_TRICOPTER:
                    return COPTER_MODE_AUTO;
                case MAV_TYPE_GROUND_ROVER:
                    return ROVER_MODE_AUTO;
                default:
                    return std::nullopt;
            }
    }
    return std::nullopt;
}

inline auto
mav_mode_command_name (MavModeCommand command) -> const char *
{
    switch (command)
    {
        case MavModeCommand::rtl:
            return "RTL";
        case MavModeCommand::manual:
            return "manual";
        case MavModeCommand::hold:
            return "hold";
        case MavModeCommand::auto_mode:
            return "auto";
    }
    return "unknown";
}
