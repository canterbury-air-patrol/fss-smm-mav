#pragma once

#include "fmu-core-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include <cstdint>

using notify_fss_command_cb = void (*)(FSSCommand cmd);
using notify_fss_comms_cb = void (*)(FSSCommsStatus status);

using notify_position_cb = void (*)(const PositionData &pd);
using notify_battery_status_cb = void (*)(const BatteryData &bd);
using notify_reached_cb = void (*)(int point);

using notify_smm_settings_cb = void (*)(const SMMSettings &settings);
