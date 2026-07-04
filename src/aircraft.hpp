#pragma once
#include "fmu-types.hpp"
#include "ilogger.hpp"
#include <chrono>
#include <map>
#include <memory>
#include <mutex>

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
    auto
    acceptableUpdate (uint64_t t_new_timestamp) -> bool
    {
        const std::chrono::milliseconds new_ts{ t_new_timestamp };
        if (this->ts + this->ts_1sec_interval <= new_ts)
        {
            this->ts = new_ts;
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
    std::map<std::string, std::shared_ptr<aircraft_details>> aircraft{};
    ILogger &logger;
    auto
    findAircraft (const std::string &t_call_sign, uint32_t t_icao_address) -> std::shared_ptr<aircraft_details>
    {
        auto result = this->aircraft.find (t_call_sign);
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
        this->aircraft.insert (std::pair<std::string, std::shared_ptr<aircraft_details>> (t_call_sign, ad));
        return ad;
    }

    /* Reclaim aircraft that have gone quiet for longer than the eviction window
     * so the map stays bounded over a long-running mission. Must be called with
     * `lock` held. `now` is the timestamp of the report driving the sweep, in
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
    auto
    newPositionReport (PositionData pd) -> bool
    {
        std::lock_guard<std::mutex> lk (this->lock);
        auto ad = this->findAircraft (pd.getCallSign (), pd.getICAOAddress ());
        const bool accepted = ad->acceptableUpdate (pd.getTimeStamp ());
        /* Sweep stale aircraft on every report so the map stays bounded. The
         * entry just touched above carries the current timestamp, so it is
         * never evicted by its own report. */
        this->evictStaleLocked (std::chrono::milliseconds{ pd.getTimeStamp () });
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