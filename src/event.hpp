#pragma once
#include "fmu-core-types.hpp"
#include "fmu-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include <variant>

struct ReachedPoint
{
    int point;
};
struct OtherAircraftReport
{
    PositionData pd;
};
struct Nudge
{
};

using event = std::variant<FSSCommand, FSSCommsStatus, MavCommsStatus, SMMSettings, PositionData, BatteryData,
                           ReachedPoint, OtherAircraftReport, Nudge>;
