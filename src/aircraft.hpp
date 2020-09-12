#include <mutex>
#include <map>
#include "fmu-types.hpp"

class aircraft_details {
    private:
        uint64_t ts{0};
        uint32_t icao_address;
    public:
        aircraft_details(uint32_t t_icao_address) : icao_address(t_icao_address) {};
        uint32_t getICAOAddress() { return this->icao_address; };
        bool acceptableUpdate(uint64_t t_new_timestamp)
        {
            if(this->ts + 1000 <= t_new_timestamp)
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
        uint32_t lastAllocatedICAO{0x1000};
        std::map<std::string, aircraft_details *> aircraft{};
        aircraft_details *findAircraft(std::string t_call_sign, uint32_t t_icao_address)
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
            auto ad = new aircraft_details(t_icao_address);
            this->aircraft.insert(std::pair<std::string, aircraft_details *>(t_call_sign, ad));
            return ad;
        }
    public:
        known_aircraft() {};
        ~known_aircraft() {
            for (auto ad : this->aircraft)
            {
                delete ad.second;
            }
            this->aircraft.clear();
        };
        bool newPositionReport(PositionData &pd)
        {
            std::lock_guard<std::mutex> lk(this->lock);
            auto ad = this->findAircraft(pd.getCallSign(), pd.getICAOAddress());
            if (ad->acceptableUpdate(pd.getTimeStamp()))
            {
                pd.setICAOAddress(ad->getICAOAddress());
                return true;
            }
            return false;
        }
};