#pragma once
#include "fmu-types.hpp"
#include <chrono>
#include <iostream>
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
};

class known_aircraft
{
  private:
    std::mutex lock{};
    uint32_t lastAllocatedICAO{ first_icao_address_for_unknown_aircraft };
    std::map<std::string, std::shared_ptr<aircraft_details>> aircraft{};
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
        std::cout << "Aircraft: Creating new aircraft with callsign " << t_call_sign << " ICAO: " << t_icao_address
                  << std::endl;
        auto ad = std::make_shared<aircraft_details> (t_icao_address);
        this->aircraft.insert (std::pair<std::string, std::shared_ptr<aircraft_details>> (t_call_sign, ad));
        return ad;
    }

  public:
    known_aircraft () = default;
    auto
    newPositionReport (PositionData pd) -> bool
    {
        std::lock_guard<std::mutex> lk (this->lock);
        auto ad = this->findAircraft (pd.getCallSign (), pd.getICAOAddress ());
        if (ad->acceptableUpdate (pd.getTimeStamp ()))
        {
            return true;
        }
        return false;
    }
    auto
    getAircraftICAOAddress (const std::string &t_call_sign) -> uint32_t
    {
        std::lock_guard<std::mutex> lk (this->lock);
        auto ad = this->findAircraft (t_call_sign, 0);
        return ad->getICAOAddress ();
    }
};