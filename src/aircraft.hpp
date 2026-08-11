#pragma once
#include "fmu-types.hpp"
#include "ilogger.hpp"
#include "util.hpp"
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

/* Aircraft that report no ICAO address are handed a synthetic one from the
 * "Unallocated" range starting at 0x1000. */
inline constexpr uint32_t first_icao_address_for_unknown_aircraft = 0x1000;
/* Top of the 24-bit ICAO address space. Synthetic allocation wraps back to the
 * start of the range here rather than running past 24 bits — the on-the-wire
 * ICAO field is only 24 bits, so a larger value could not be represented. */
inline constexpr uint32_t last_icao_address_for_unknown_aircraft = 0xFFFFFF;

/* Next synthetic ICAO address after `current`, wrapping to the start of the
 * range at the 24-bit ceiling. A single mission never approaches 16 million
 * distinct callsigns, so the wrap is a defensive guard against an unbounded
 * counter rather than an expected path. Pure and free of the map/mutex state so
 * the wrap can be unit tested directly. */
inline auto
next_synthetic_icao (uint32_t current) -> uint32_t
{
    if (current >= last_icao_address_for_unknown_aircraft)
    {
        return first_icao_address_for_unknown_aircraft;
    }
    return current + 1;
}

/* Map key identifying one tracked aircraft.
 *
 * A real ICAO address is the aircraft's own globally unique identity, so it is
 * the key whenever one is reported; the callsign is only a fallback for a peer
 * that reports no address (which is then handed a synthetic one). Keying purely
 * on callsign merged every contact that shared one -- and a blank callsign is
 * common in ADS-B feeds, so several distinct aircraft could collapse into a
 * single aircraft_details and with it a single 1 s debounce
 * (acceptableUpdate()): roughly one report per second survived for the whole
 * group, and the rest never reached the autopilot's collision avoidance at all.
 *
 * Two aircraft that both report no address and share a callsign still collide;
 * nothing in the report distinguishes them. The prefixes keep the two key
 * spaces from ever meeting (a callsign of "icao:1234" is not an address). */
inline auto
aircraft_key (const std::string &t_call_sign, uint32_t t_icao_address) -> std::string
{
    if (t_icao_address != 0)
    {
        return "icao:" + std::to_string (t_icao_address);
    }
    return "cs:" + t_call_sign;
}

class aircraft_details
{
  private:
    std::chrono::milliseconds ts{ 0 };
    uint32_t icao_address;
    static constexpr std::chrono::milliseconds ts_1sec_interval{ 1000 };

  public:
    explicit aircraft_details (uint32_t t_icao_address) : icao_address (t_icao_address) {};
    auto
    getICAOAddress () -> uint32_t
    {
        return this->icao_address;
    };
    /* Debounce to at most one accepted update per second, keyed off the
     * caller's local "now" -- deliberately NOT the reporting peer's own
     * self-reported timestamp, which is untrusted, unvalidated wire data (a
     * peer with a wrong clock, no NTP, or a glitching ADS-B decoder can
     * send anything). A bad peer timestamp used to be able to jam `ts` far
     * into the future, after which no genuine subsequent report from that
     * same aircraft would ever satisfy `ts + 1s <= new_ts` again --
     * permanently silencing it for the life of the process. */
    auto
    acceptableUpdate (uint64_t now_ms) -> bool
    {
        const std::chrono::milliseconds now{ now_ms };
        if (this->ts + this->ts_1sec_interval <= now)
        {
            this->ts = now;
            return true;
        }
        return false;
    }
    auto
    getLastUpdate () -> std::chrono::milliseconds
    {
        return this->ts;
    }
};

/* Drop an aircraft that has not produced an acceptable update within this
 * window. It is far larger than the ~1 s report cadence, so an actively
 * reporting aircraft is never evicted; an aircraft that has left the operating
 * area is reclaimed so the map cannot grow without bound. If such an aircraft
 * later returns it is simply re-added (with a fresh synthetic ICAO). */
inline constexpr std::chrono::milliseconds aircraft_eviction_age{ 5 * 60 * 1000 };

class known_aircraft
{
  private:
    std::mutex lock{};
    uint32_t lastAllocatedICAO{ first_icao_address_for_unknown_aircraft };
    /* Keyed by aircraft_key(): the reported ICAO address where there is one,
     * the callsign otherwise. See that function for why the callsign alone is
     * not enough. */
    std::map<std::string, std::shared_ptr<aircraft_details>> aircraft{};
    ILogger &logger;
    /* Local "now" source (milliseconds since some fixed epoch), driving both
     * aircraft_details::acceptableUpdate()'s debounce and evictStaleLocked()'s
     * sweep below. Defaults to the FMU's own clock; overridable for tests
     * (mirrors EventDispatcher::now_ms_fn). Deliberately NOT the reporting
     * peer's own PositionData::getTimeStamp() -- trusting a peer's
     * self-reported clock let one bad/skewed report either permanently
     * silence that aircraft's own future updates (see acceptableUpdate) or
     * evict every other tracked aircraft in the same evictStaleLocked() call.
     */
    std::function<uint64_t ()> now_ms_fn{ current_timestamp_ms };
    auto
    findAircraft (const std::string &t_call_sign, uint32_t t_icao_address) -> std::shared_ptr<aircraft_details>
    {
        const std::string key = aircraft_key (t_call_sign, t_icao_address);
        auto result = this->aircraft.find (key);
        if (result != this->aircraft.end ())
        {
            return result->second;
        }
        if (t_icao_address == 0)
        {
            this->lastAllocatedICAO = next_synthetic_icao (this->lastAllocatedICAO);
            t_icao_address = this->lastAllocatedICAO;
        }
        this->logger.log (LogLevel::info, "Aircraft: Creating new aircraft with callsign " + t_call_sign
                                              + " ICAO: " + std::to_string (t_icao_address));
        auto ad = std::make_shared<aircraft_details> (t_icao_address);
        this->aircraft.insert (std::pair<std::string, std::shared_ptr<aircraft_details>> (key, ad));
        return ad;
    }

    /* Reclaim aircraft that have gone quiet for longer than the eviction window
     * so the map stays bounded over a long-running mission. Must be called with
     * `lock` held. `now` is the local "now" (now_ms_fn()) driving the sweep, in
     * the same clock as aircraft_details::ts. */
    void
    evictStaleLocked (std::chrono::milliseconds now)
    {
        for (auto it = this->aircraft.begin (); it != this->aircraft.end ();)
        {
            if (it->second->getLastUpdate () + aircraft_eviction_age < now)
            {
                it = this->aircraft.erase (it);
            }
            else
            {
                ++it;
            }
        }
    }

  public:
    explicit known_aircraft (ILogger &t_logger) : logger (t_logger) {}

    /* Test seam: inject a fake "now" source, mirroring
     * EventDispatcher::setNowMsFn. Production code never calls this; the
     * default (current_timestamp_ms) is used unless overridden. */
    void
    setNowMsFn (std::function<uint64_t ()> fn)
    {
        this->now_ms_fn = std::move (fn);
    }

    auto
    newPositionReport (PositionData pd) -> bool
    {
        std::lock_guard<std::mutex> lk (this->lock);
        auto ad = this->findAircraft (pd.getCallSign (), pd.getICAOAddress ());
        const uint64_t now_ms = this->now_ms_fn ();
        const bool accepted = ad->acceptableUpdate (now_ms);
        /* Sweep stale aircraft on every report so the map stays bounded. The
         * entry just touched above carries the current local time, so it is
         * never evicted by its own report. */
        this->evictStaleLocked (std::chrono::milliseconds{ now_ms });
        return accepted;
    }
    auto
    getAircraftICAOAddress (const std::string &t_call_sign) -> uint32_t
    {
        std::lock_guard<std::mutex> lk (this->lock);
        auto ad = this->findAircraft (t_call_sign, 0);
        return ad->getICAOAddress ();
    }
};