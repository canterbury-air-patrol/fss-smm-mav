#pragma once
#include "fmu-core-types.hpp"
#include "fmu-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include <memory>
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

/* An action produced by the SMM worker thread, fed back through the event queue
 * so it is applied on the event-loop thread. The handlers apply these only while
 * the FMU is still in the searching state, so a stale action that raced a
 * higher-priority command (rtl/terminate/latch) is simply dropped — keeping the
 * single-threaded state machine the sole arbiter of priority (todo/33). */
struct SmmLoadSearch
{
    std::shared_ptr<SMMSearch> search;
};
struct SmmRtl
{
};

/* An FSS command together with the responder that acks its resolution back to
 * FSS. The responder is carried through the event queue so the ack is sent from
 * the same thread that runs the state machine, once the command resolves. */
struct FSSCommandEvent
{
    FSSCommand command{ fss_cmd_unknown };
    /* The goto/altitude target this command carries (default for commands that
     * carry none), delivered with the command so the state machine actions it
     * from the event rather than an FSS side-channel (todo/53). */
    FSSCommandTarget target{};
    fss_command_ack_responder ack;
};

using event = std::variant<FSSCommandEvent, FSSCommsStatus, MavCommsStatus, SMMSettings, PositionData, BatteryData,
                           ReachedPoint, OtherAircraftReport, SmmLoadSearch, SmmRtl, Nudge>;
