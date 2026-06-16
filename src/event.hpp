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

/* An FSS command together with the responder that acks its resolution back to
 * FSS. The responder is carried through the event queue so the ack is sent from
 * the same thread that runs the state machine, once the command resolves. */
struct FSSCommandEvent
{
    FSSCommand command{ fss_cmd_unknown };
    fss_command_ack_responder ack;
};

using event = std::variant<FSSCommandEvent, FSSCommsStatus, MavCommsStatus, SMMSettings, PositionData, BatteryData,
                           ReachedPoint, OtherAircraftReport, Nudge>;
