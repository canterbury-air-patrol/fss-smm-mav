#pragma once
#include "fss/fmu-fss-types.hpp"
#include "fmu-types.hpp"
#include "smm/smm-types.hpp"

enum event_type {
    event_unknown,
    event_fss_command,
    event_fss_comms_status,
    event_smm_settings,
    event_position,
    event_reached,
    event_battery_status,
    event_other_aircraft_report,
    event_nudge,
};

class event {
private:
    event_type et;
    FSSCommand command{};
    FSSCommsStatus comms_status{};
    SMMSettings smm_settings{};
    PositionData pd{};
    BatteryData bd{};
    int reached_point{0};
public:
    explicit event(event_type t_et) : et(t_et) {};
    explicit event(FSSCommand t_command) : et(event_fss_command), command(t_command) {};
    explicit event(FSSCommsStatus t_status) : et(event_fss_comms_status), comms_status(t_status) {};
    explicit event(SMMSettings t_settings) : et(event_smm_settings), smm_settings(std::move(t_settings)) {};
    explicit event(PositionData t_pd) : et(event_position), pd(std::move(t_pd)) {};
    event(event_type t_et, PositionData t_pd) : et(t_et), pd(std::move(t_pd)) {};
    explicit event(int t_reached_point) : et(event_reached), reached_point(t_reached_point) {};
    explicit event(BatteryData t_bd) : et(event_battery_status), bd(t_bd) {};
    auto getType() -> event_type { return this->et; };
    auto getFSSCommand() -> FSSCommand { return this->command; };
    auto getFSSCommsStatus() -> FSSCommsStatus { return this->comms_status; };
    auto getSMMSettings() -> SMMSettings { return this->smm_settings; };
    auto getPositionData() -> PositionData { return this->pd; };
    auto getBatteryData() -> BatteryData { return this->bd; };
    auto getReachedPoint() -> int { return this->reached_point; };
};