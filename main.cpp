#include <bits/stdint-uintn.h>
#include <string>
#include <list>

enum flight_mode {
    flight_mode_unknown,
    flight_mode_manual,
    flight_mode_search,
    flight_mode_rtl,
    flight_mode_goto,
    flight_mode_hold,
};

class MAV {
private:
    flight_mode mode{flight_mode_unknown};
    bool armed{false};
public:
    MAV() {};
    flight_mode getFlightMode() { return this->mode; };
    bool getArmed() { return this->armed; };
};

class Point {
private:
    double latitude;
    double longitude;
public:
    Point(double lat, double lng) : latitude(lat), longitude(lng) {};
    double getLatitude() { return latitude; };
    double getLongitude() { return longitude; };
};

class SMMSearch {
private:
    std::list<Point> points{};
    uint64_t search_id{0};
public:
    SMMSearch(uint64_t id) : search_id(id) {};
    void addPoint(Point p) { this->points.push_back(p); };
    uint64_t getSearchId() { return this->search_id; };
    const std::list<Point> getPoints() { return this->points; };
};

enum SMMCommand {
    smm_cmd_none,
    smm_cmd_abandon_search,
    smm_cmd_mission_complete,
};

class SMM {
private:
    SMMSearch *current_search{nullptr};
public:
    SMM() {};
};

enum FSSCommand {
    fss_cmd_unknown,
    fss_cmd_rtl,
    fss_cmd_hold,
    fss_cmd_altitude,
    fss_cmd_goto,
    fss_cmd_continue,
    fss_cmd_disarm,
    fss_cmd_terminate,
};

class FSS {
private:
public:
    FSS();
};

enum FMUState {
    fmu_state_manual,
    fmu_state_searching,
    fmu_state_goto,
    fmu_state_altitude_adjust,
    fmu_state_rtl,
    fmu_state_hold,
    fmu_state_low_battery,
    fmu_state_failsafe,
};

class FMUStateMachine {
private:
    FMUState current_state{fmu_state_manual};
    FSSCommand fss_command{fss_cmd_unknown};
    SMMCommand smm_command{smm_cmd_none};
public:
    FMUStateMachine();
    FMUState getCurrentstate();

    void FSSNewCommand(FSSCommand cmd);
    void SMMNewCommand(SMMCommand cmd);
    void setLowBattery();
};

int main(int argc, char *argv[])
{

}