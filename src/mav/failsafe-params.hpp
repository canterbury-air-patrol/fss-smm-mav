#pragma once

#include <ardupilotmega/mavlink.h>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mav.hpp"

/* Sanity-check the autopilot's own failsafe config against the safety backstops
 * the FMU's design leans on while the MAV link is down (see
 * FMUStateMachine::actionState() / sendMavLinkMsgLocked()) — AFS for
 * --terminate-action=terminate, and the autopilot's GCS/telemetry failsafe for
 * the comms-loss/low-battery latches. Read-only and advisory: the FMU requests
 * parameters, never writes them, and a mismatch warns rather than blocking
 * flight. The ground tool (tools/apconfig_check.py) is where a misconfiguration
 * is meant to be caught; this is the last-chance warning when it was not, so the
 * two expectation tables mirror each other and must be kept in step.
 *
 * These checks first shipped with every one reduced to "this param must read
 * nonzero", which is not sufficient: an enable flag says the failsafe *fires*,
 * not what it does when it does. FS_GCS_ENABL=1 with FS_LONG_ACTN=0 (the Plane
 * default) triggers and then continues the mission in AUTO and GUIDED — the
 * modes this FMU flies — so the aircraft has no backstop and the check passed
 * anyway. Copter has the same trap in FS_OPTIONS bit 1, and Rover in FS_ACTION.
 * Hence a predicate per check rather than a shared nonzero test. */

/* cap-fmu's own MAVLink system id (SYS_ID in mavlink.cpp, which static_asserts
 * against this). The autopilot's GCS failsafe must watch *this* heartbeat: under
 * the deployed topology MAVProxy runs on the same companion computer and
 * heartbeats as 255, so at SYSID_MYGCS's default the failsafe tracks MAVProxy
 * and the death of the FMU process is invisible to it. The fleet's decided
 * configuration is SYSID_MYGCS=200. */
constexpr uint8_t fmu_gcs_sys_id = 200;

/* FS_OPTIONS bit 1 (Copter): "continue if in Auto on GCS failsafe" — the same
 * defect as FS_LONG_ACTN=0 on Plane, expressed as a bitmask. */
constexpr uint32_t fs_options_continue_in_auto_bit = 1U << 1U;

/* One parameter expectation: what to request, how to judge the reply, and why it
 * matters. Mirrors tools/apconfig_check.py's Check.
 *
 * The exact float comparisons the predicates below make are deliberate, not a
 * float-equality footgun: every parameter checked here is an ArduPilot
 * integer/enum/bitmask parameter, and small integers are exactly representable
 * in IEEE-754 float — there is no rounding or scaling in play, so a parameter
 * the autopilot holds as N always arrives as precisely N.0F, never a near value
 * a tolerance would be needed for. */
struct FailsafeParamCheck
{
    /* MAVLink param_id to request (PARAM_REQUEST_READ) and match a
     * PARAM_VALUE reply against. <=16 chars per the MAVLink field width. */
    const char *name = nullptr;
    /* True when the value read back leaves the backstop intact. A captureless
     * lambda converts to this, so each entry below states its own rule next to
     * the text that explains it. */
    bool (*accept) (float) = nullptr;
    /* Logged if accept() rejects the value, carrying no severity prefix of its
     * own — the caller logs it at LogLevel::warning, prefixed with the param
     * name and the value actually read. A std::string (not const char *) since
     * the GCS-failsafe entries build their text from the resolved param name.
     */
    std::string warning;
};

/* ArduPilot vehicle families, which share a parameter set. Mirrors
 * FAMILY_BY_MAV_TYPE in tools/apconfig_check.py. */
enum class vehicle_family : uint8_t
{
    plane,
    copter,
    rover,
};

/* The failsafe parameter set is vehicle-firmware dependent, unlike
 * AFS_ENABLE/AFS_TERM_ACTION (uniform across Plane/Copter/Rover, part of the
 * shared AP_AdvancedFailsafe library). Plane and Copter parameters were read
 * back by name from ArduPilot 4.6 (SITL); the Rover ones are taken from
 * Rover/Parameters.cpp and have never been requested from a running vehicle
 * (see README) — an autopilot_type this doesn't recognise, or a wrong name for
 * one it does, simply means those checks are skipped/never answered, degrading
 * to "not checked" rather than a false "all clear". */
inline auto
resolve_vehicle_family (uint8_t autopilot_type) -> std::optional<vehicle_family>
{
    switch (autopilot_type)
    {
        case MAV_TYPE_FIXED_WING:
            return vehicle_family::plane;
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
            return vehicle_family::copter;
        case MAV_TYPE_GROUND_ROVER:
            return vehicle_family::rover;
        default:
            return std::nullopt;
    }
}

/* The GCS/telemetry-failsafe enable parameter's name. Plane alone drops the
 * trailing E. */
inline auto
gcs_failsafe_param_name (vehicle_family family) -> const char *
{
    return family == vehicle_family::plane ? "FS_GCS_ENABL" : "FS_GCS_ENABLE";
}

/* The same, for an airframe that may not be one this recognises. */
inline auto
resolve_gcs_failsafe_param_name (uint8_t autopilot_type) -> std::optional<const char *>
{
    auto family = resolve_vehicle_family (autopilot_type);
    if (!family)
    {
        return std::nullopt;
    }
    return gcs_failsafe_param_name (*family);
}

/* The params worth requesting/sanity-checking for this autopilot type and the
 * selected --terminate-action. AFS_ENABLE/AFS_TERM_ACTION only matter when
 * `action` can actually issue MAV_CMD_DO_FLIGHTTERMINATION (matches
 * MAV::terminate()'s dispatch in mav.cpp); everything else backstops the
 * comms-loss/low-battery latches, so it applies regardless of `action`,
 * whenever the airframe is recognised.
 *
 * Deliberately not checked here, though tools/apconfig_check.py does check them:
 * the failsafe *timeouts* (a slow backstop is still a backstop, and the budget
 * they feed is TC-FS-001's to measure) and SYSID_ENFORCE (an operational
 * lockout of the pilot's GCS, not a question of whether the aircraft fails
 * safe). This list is scoped to "does a backstop exist at all". */
inline auto
expected_failsafe_params (uint8_t autopilot_type, terminate_action action) -> std::vector<FailsafeParamCheck>
{
    auto is_nonzero = [] (float value) { return value != 0.0F; };

    std::vector<FailsafeParamCheck> params;
    if (action == terminate_action::terminate)
    {
        params.push_back ({ "AFS_ENABLE", is_nonzero,
                            "AFS is disabled on the autopilot but --terminate-action=terminate requires it enabled "
                            "for MAV_CMD_DO_FLIGHTTERMINATION to have any effect" });
        params.push_back ({ "AFS_TERM_ACTION", is_nonzero,
                            "the AFS termination action is not configured on the autopilot but "
                            "--terminate-action=terminate requires it to be set" });
    }

    auto family = resolve_vehicle_family (autopilot_type);
    if (!family)
    {
        return params;
    }

    const char *gcs_param = gcs_failsafe_param_name (*family);
    switch (*family)
    {
        case vehicle_family::plane:
            params.push_back ({ gcs_param, [] (float value) { return value == 1.0F || value == 2.0F; },
                                std::string (gcs_param)
                                    + " is not enabled for every mode the FMU flies (1 or 2): the autopilot's own "
                                      "GCS/telemetry failsafe is the backstop the FMU relies on while the MAV link "
                                      "is down, and 3 only fails safe in AUTO" });
            params.push_back ({ "FS_LONG_ACTN", [] (float value) { return value == 1.0F; },
                                "the long-failsafe action is not RTL (1), so a GCS failsafe fires and the aircraft "
                                "carries on flying the mission in AUTO and GUIDED — the modes the FMU flies — "
                                "leaving no backstop" });
            break;
        case vehicle_family::copter:
            params.push_back ({ gcs_param, [] (float value) { return value == 1.0F || value == 3.0F; },
                                std::string (gcs_param)
                                    + " is not set to an action that returns the aircraft (1 = RTL, 3 = SmartRTL "
                                      "falling back to RTL): the autopilot's own GCS/telemetry failsafe is the "
                                      "backstop the FMU relies on while the MAV link is down, and 2 continues the "
                                      "mission" });
            params.push_back ({ "FS_OPTIONS", [] (float value)
                                { return (static_cast<uint32_t> (value) & fs_options_continue_in_auto_bit) == 0; },
                                "FS_OPTIONS bit 1 is set, so a GCS failsafe in AUTO continues the mission instead of "
                                "returning — the same trap as FS_LONG_ACTN=0 on Plane, leaving no backstop" });
            break;
        case vehicle_family::rover:
            params.push_back ({ gcs_param, [] (float value) { return value == 1.0F; },
                                std::string (gcs_param)
                                    + " is not enabled (1): the autopilot's own GCS/telemetry failsafe is the "
                                      "backstop the FMU relies on while the MAV link is down, and 2 continues the "
                                      "mission in Auto" });
            params.push_back ({ "FS_ACTION", [] (float value) { return value == 1.0F || value == 3.0F; },
                                "the failsafe action is not one that returns the vehicle (1 = RTL, 3 = SmartRTL "
                                "falling back to RTL); the default is Hold" });
            break;
    }

    /* Last, because it is the one that decides whether any of the above is
     * pointed at the right heartbeat. Uniform across the families. */
    params.push_back ({ "SYSID_MYGCS", [] (float value) { return value == static_cast<float> (fmu_gcs_sys_id); },
                        "the GCS failsafe is watching a different system id than the FMU heartbeats as (200), so the "
                        "FMU dying is invisible to it — MAVProxy on the same companion computer keeps the heartbeat "
                        "the autopilot is actually watching alive" });
    return params;
}
