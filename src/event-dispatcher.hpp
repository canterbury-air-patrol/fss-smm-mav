#pragma once

#include "event.hpp"
#include "fmu.hpp"
#include "fss/ifss.hpp"
#include "logger.hpp"
#include "mav/imav.hpp"
#include "smm/ismm.hpp"
#include "util.hpp"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>

/* Minimum interval between ADS-B rebroadcasts of the same ICAO address (the
 * CAP test plan's §4.1 requirement). Also the retention window for the
 * throttle map: once an entry is this old it can no longer suppress a forward,
 * so it is dropped. Namespace scope so the dispatch path, the pruning and the
 * tests all name the same number. */
inline constexpr uint64_t adsb_forward_interval_ms = 1000;

/* The event-loop's routing policy: which events reach the state machine,
 * which SMM/MAV outcomes are gated on isSearching()/isWaitingForTasking(),
 * what gets logged, how battery readings are classified. Extracted from
 * App::run()'s std::visit block so it can be driven by the existing
 * FMUStateMachine mocks in tests/fmu_test.cpp without a real FSS/MAV/SMM
 * (sockets, config files). App keeps the queue/thread/signal plumbing and
 * callback registration; this only applies one already-dequeued event.
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

    /* Test seam: inject a fake "now" source so the per-ICAO ADS-B throttle
     * can be exercised without a real 1s sleep_for. */
    void setNowMsFn (std::function<uint64_t ()> fn);

    /* Test seam: how many ICAO addresses the throttle map is currently holding.
     * Production code never calls this; it exists so a test can assert the map
     * stays bounded, which is otherwise only observable as memory growth. */
    [[nodiscard]] auto adsbThrottleEntries () const -> std::size_t;

  private:
    FMUStateMachine &state_machine;
    IMAV &mav;
    ISMM &smm;
    IFSSReporter &fss;
    Logger &logger;
    std::string asset_name;
    int lowbat_threshold;
    std::function<uint64_t ()> now_ms_fn{ current_timestamp_ms };
    /* Last-forwarded time per ICAO address, throttling ADS-B rebroadcast to at
     * most one per address per second (the CAP test plan's §4.1 requirement) so
     * a busy receiver near a real airport cannot flood ArduPilot with
     * ADSB_VEHICLE updates at dump1090's raw rate.
     *
     * Pruned by pruneAdsbThrottle() to just the addresses forwarded within the
     * last interval, so it is sized by current traffic rather than by history.
     * This used to be justified as "one entry per distinct aircraft ever seen
     * this flight", which was not what happened: a peer reporting no ICAO
     * address gets a synthetic one from known_aircraft, which evicts an
     * aircraft after five quiet minutes and issues a *fresh* synthetic address
     * when it returns --- so one aircraft drifting in and out of range added an
     * entry per reappearance rather than reusing one. */
    std::map<uint32_t, uint64_t> adsb_last_forwarded_ms{};
    /* Drop throttle entries that can no longer suppress anything. An entry
     * older than the forward interval already permits the next forward, so
     * removing it is exactly equivalent to keeping it --- this bounds the map
     * without changing a single forwarding decision. Swept on every report,
     * mirroring known_aircraft::evictStaleLocked(); the surviving set is only
     * the aircraft forwarded in the last second, so the scan stays small even
     * under airport-density traffic. */
    void pruneAdsbThrottle (uint64_t now);
};
