#include <memory>
#include <mutex>
#include <map>
#include <iostream>
#include "fmu-types.hpp"


class aircraft_details {
    private:
        uint64_t ts{0};
        uint32_t icao_address;
        static constexpr uint64_t ts_1sec_interval = 1000;
    public:
        explicit aircraft_details(uint32_t t_icao_address) : icao_address(t_icao_address) {};
        auto getICAOAddress() -> uint32_t { return this->icao_address; };
        auto acceptableUpdate(uint64_t t_new_timestamp) -> bool
        {
            if(this->ts + this->ts_1sec_interval <= t_new_timestamp)
            {
                this->ts = t_new_timestamp;
                return true;
            }
            return false;
        }
};

class known_aircraft {
    private:
        std::mutex lock{};
        /* Warning: This is part of an "Unallocated" range of ICAO callsigns */
        static constexpr uint32_t first_icao_address_for_unknown_aircraft = 0x1000;
        uint32_t lastAllocatedICAO{first_icao_address_for_unknown_aircraft};
        std::map<std::string, std::shared_ptr<aircraft_details>> aircraft{};
        auto findAircraft(const std::string &t_call_sign, uint32_t t_icao_address) -> std::shared_ptr<aircraft_details>
        {
            auto result = this->aircraft.find(t_call_sign);
            if (result != this->aircraft.end())
            {
                return result->second;
            }
            if (t_icao_address == 0)
            {
                t_icao_address = ++this->lastAllocatedICAO;
            }
            std::cout << "Aircraft: Creating new aircraft with callsign " << t_call_sign << " ICAO: " << t_icao_address << std::endl;
            auto ad = std::make_shared<aircraft_details>(t_icao_address);
            this->aircraft.insert(std::pair<std::string, std::shared_ptr<aircraft_details>>(t_call_sign, ad));
            return ad;
        }
    public:
        known_aircraft() = default;
        auto newPositionReport(PositionData pd) -> bool
        {
            std::lock_guard<std::mutex> lk(this->lock);
            auto ad = this->findAircraft(pd.getCallSign(), pd.getICAOAddress());
            if (ad->acceptableUpdate(pd.getTimeStamp()))
            {
                return true;
            }
            return false;
        }
        auto getAircraftICAOAddress(const std::string &t_call_sign) -> uint32_t
        {
            auto ad = this->findAircraft(t_call_sign, 0);
            return ad->getICAOAddress();
        }
};