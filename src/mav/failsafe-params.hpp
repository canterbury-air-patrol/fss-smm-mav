#pragma once

#include <ardupilotmega/mavlink.h>
#include <optional>
#include <string>
#include <vector>

#include "mav.hpp"

/* todo/91: sanity-check the autopilot's own failsafe config against the
 * safety backstops the FMU's design leans on while the MAV link is down (see
 * FMUStateMachine::actionState() / sendMavLinkMsgLocked()) — AFS for
 * --terminate-action=terminate, and the autopilot's GCS/telemetry failsafe
 * for the comms-loss/low-battery latches. Read-only and advisory: every
 * check here reduces to "this param must read nonzero", so the caller need
 * only compare a PARAM_VALUE reply against 0. */
struct FailsafeParamCheck
{
    /* MAVLink param_id to request (PARAM_REQUEST_READ) and match a
     * PARAM_VALUE reply against. <=16 chars per the MAVLink field width. */
    const char *name = nullptr;
    /* Log text if the param reads 0, carrying no severity prefix of its own —
     * the caller logs it at LogLevel::warning and the line names the level
     * (todo/109). A std::string (not const char *) since the GCS-failsafe
     * entry below builds its text from the resolved param name. */
    std::string warning;
};

/* The GCS/telemetry-failsafe enable param name is vehicle-firmware
 * dependent, unlike AFS_ENABLE/AFS_TERM_ACTION (uniform across Plane/Copter/
 * Rover, part of the shared AP_AdvancedFailsafe library). The Plane and
 * Copter names were read back by name from ArduPilot 4.6 (SITL); the Rover
 * name is taken from Rover/Parameters.cpp and has never been requested from
 * a running vehicle (see README) — an autopilot_type this doesn't
 * recognise, or a wrong name for one it does, simply means that check is
 * skipped/never answered, degrading to "not checked" rather than a false
 * "all clear". */
inline auto
resolve_gcs_failsafe_param_name (uint8_t autopilot_type) -> std::optional<const char *>
{
    switch (autopilot_type)
    {
        case MAV_TYPE_FIXED_WING:
            return "FS_GCS_ENABL";
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
        case MAV_TYPE_GROUND_ROVER:
            return "FS_GCS_ENABLE";
        default:
            return std::nullopt;
    }
}

/* The params worth requesting/sanity-checking for this autopilot type and
 * the selected --terminate-action. AFS_ENABLE/AFS_TERM_ACTION only matter
 * when `action` can actually issue MAV_CMD_DO_FLIGHTTERMINATION (matches
 * MAV::terminate()'s dispatch in mav.cpp); the GCS-failsafe entry backstops
 * the comms-loss/low-battery latches, so it applies regardless of `action`,
 * whenever the airframe is recognised. */
inline auto
expected_failsafe_params (uint8_t autopilot_type, terminate_action action) -> std::vector<FailsafeParamCheck>
{
    std::vector<FailsafeParamCheck> params;
    if (action == terminate_action::terminate)
    {
        params.push_back ({ "AFS_ENABLE", "AFS_ENABLE=0 on the autopilot but --terminate-action=terminate requires "
                                          "AFS enabled for MAV_CMD_DO_FLIGHTTERMINATION to have any effect" });
        params.push_back ({ "AFS_TERM_ACTION", "AFS_TERM_ACTION is not configured (0) on the autopilot but "
                                               "--terminate-action=terminate requires it to be set" });
    }
    if (auto gcs_param = resolve_gcs_failsafe_param_name (autopilot_type))
    {
        params.push_back (
            { *gcs_param, std::string (*gcs_param)
                              + "=0 on the autopilot: its own GCS/telemetry failsafe is disabled, removing the "
                                "backstop the FMU relies on while the MAV link is down" });
    }
    return params;
}
