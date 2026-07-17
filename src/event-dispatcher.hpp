#pragma once

#include "event.hpp"
#include "fmu.hpp"
#include "fss/ifss.hpp"
#include "logger.hpp"
#include "mav/imav.hpp"
#include "smm/ismm.hpp"
#include "util.hpp"
#include <cstdint>
#include <functional>
#include <map>
#include <string>

/* The event-loop's routing policy: which events reach the state machine,
 * which SMM/MAV outcomes are gated on isSearching()/isWaitingForTasking(),
 * what gets logged, how battery readings are classified. Extracted from
 * App::run()'s std::visit block (todo/76) so it can be driven by the
 * existing FMUStateMachine mocks in tests/fmu_test.cpp without a real
 * FSS/MAV/SMM (sockets, config files). App keeps the queue/thread/signal
 * plumbing and callback registration; this only applies one already-dequeued
 * event.
 *
 * Registers itself as the state machine's state-change callback at
 * construction (folding in what App::run() used to do inline), logging the
 * STATE line on every transition. */
class EventDispatcher
{
  public:
    EventDispatcher (FMUStateMachine &t_state_machine, IMAV &t_mav, ISMM &t_smm, IFSSReporter &t_fss, Logger &t_logger,
                     std::string t_asset_name, int t_lowbat_threshold);

    void dispatch (const event &e);

    /* Test seam (todo/81): inject a fake "now" source so the per-ICAO ADS-B
     * throttle can be exercised without a real 1s sleep_for. */
    void setNowMsFn (std::function<uint64_t ()> fn);

  private:
    FMUStateMachine &state_machine;
    IMAV &mav;
    ISMM &smm;
    IFSSReporter &fss;
    Logger &logger;
    std::string asset_name;
    int lowbat_threshold;
    std::function<uint64_t ()> now_ms_fn{ current_timestamp_ms };
    /* Last-forwarded time per ICAO address (todo/81), throttling ADS-B
     * rebroadcast to at most one per address per second (the CAP test plan's
     * §4.1 requirement) so a busy receiver near a real airport cannot flood
     * ArduPilot with ADSB_VEHICLE updates at dump1090's raw rate. Grows by one
     * entry per distinct aircraft ever seen this flight; unbounded is fine at
     * realistic air-traffic density, so no pruning. */
    std::map<uint32_t, uint64_t> adsb_last_forwarded_ms{};
};
