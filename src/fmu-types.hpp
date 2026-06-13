#pragma once

#include "fmu-core-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include <cstdint>
#include <functional>

using notify_fss_command_cb = std::function<void (FSSCommand)>;
using notify_fss_comms_cb = std::function<void (FSSCommsStatus)>;
using notify_mav_comms_cb = std::function<void (MavCommsStatus)>;

using notify_position_cb = std::function<void (const PositionData &)>;
using notify_battery_status_cb = std::function<void (const BatteryData &)>;
using notify_reached_cb = std::function<void (int)>;

using notify_smm_settings_cb = std::function<void (const SMMSettings &)>;
