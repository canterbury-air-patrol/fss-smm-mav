#pragma once

#include "../fmu-core-types.hpp"
#include "../fmu-state-types.hpp"
#include "../fmu-types.hpp"

/* Thin seam over the outbound reports the event-loop dispatch makes to FSS —
 * not the connection/callback-registration machinery, which stays App's job
 * and is never touched per-event. Lets EventDispatcher be exercised with a
 * mock in tests/fmu_test.cpp without a real FSS/SSL connection (todo/76). */
class IFSSReporter
{
  public:
    IFSSReporter () = default;
    IFSSReporter (const IFSSReporter &) = delete;
    IFSSReporter (IFSSReporter &&) = delete;
    auto operator= (const IFSSReporter &) -> IFSSReporter & = delete;
    auto operator= (IFSSReporter &&) -> IFSSReporter & = delete;
    virtual ~IFSSReporter () = default;

    virtual void reportPosition (PositionData pd) = 0;
    virtual void reachedPoint (int point, int total_points) = 0;
    virtual void reportBatteryStatus (BatteryData bd) = 0;
    /* Enqueue the resolved second-phase command ack for the worker to send. */
    virtual void postAck (const fss_command_ack_responder &ack, const FSSCommandResolution &res) = 0;
};
