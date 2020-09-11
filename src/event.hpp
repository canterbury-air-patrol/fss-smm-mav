#include "fss/fmu-fss-types.hpp"
#include "fmu-types.hpp"

enum event_type {
    event_unknown,
    event_fss_command,
    event_fss_comms_status,
    event_smm_settings,
    event_position,
    event_reached,
    event_battery_status,
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
    event(event_type t_et) : et(t_et) {};
    event(FSSCommand t_command) : et(event_fss_command), command(t_command) {};
    event(FSSCommsStatus t_status) : et(event_fss_comms_status), comms_status(t_status) {};
    event(SMMSettings t_settings) : et(event_smm_settings), smm_settings(t_settings) {};
    event(PositionData t_pd) : et(event_position), pd(t_pd) {};
    event(int t_reached_point) : et(event_reached), reached_point(t_reached_point) {};
    event(BatteryData t_bd) : et(event_battery_status), bd(t_bd) {};
    event_type getType() { return this->et; };
    FSSCommand getFSSCommand() { return this->command; };
    FSSCommsStatus getFSSCommsStatus() { return this->comms_status; };
    SMMSettings getSMMSettings() { return this->smm_settings; };
    PositionData getPositionData() { return this->pd; };
    BatteryData getBatteryData() { return this->bd; };
    int getReachedPoint() { return this->reached_point; };
};