#pragma once

#include "fmu-core-types.hpp"
#include "fmu-state-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include <cstdint>
#include <functional>

/* Sends the second-phase command acknowledgement back to FSS once the state
 * machine has resolved the command. Given the resolution, the FSS layer maps it
 * to the wire outcome/reason and sends it on the originating connection. Carried
 * alongside the FSSCommand so the (later, async) resolution can be acked without
 * the event loop needing to know any transport detail. */
using fss_command_ack_responder = std::function<void (const FSSCommandResolution &)>;

using notify_fss_command_cb
    = std::function<void (FSSCommand, const FSSCommandTarget &, const fss_command_ack_responder &)>;
using notify_fss_comms_cb = std::function<void (FSSCommsStatus)>;
using notify_mav_comms_cb = std::function<void (MavCommsStatus)>;
/* The autopilot restarted underneath us while the link stayed up (todo/108).
 * Carries nothing: the only thing the event loop does with it is re-apply the
 * state machine's current state, which it already holds. */
using notify_autopilot_restart_cb = std::function<void ()>;

using notify_position_cb = std::function<void (const PositionData &)>;
using notify_battery_status_cb = std::function<void (const BatteryData &)>;
using notify_reached_cb = std::function<void (int)>;

using notify_smm_settings_cb = std::function<void (const SMMSettings &)>;
