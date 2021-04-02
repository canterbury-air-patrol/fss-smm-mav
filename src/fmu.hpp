#pragma once
#include <memory>
#include <mutex>
#include <condition_variable>

#include "fss/fmu-fss.hpp"
#include "smm/smm.hpp"
#include "mav/mav.hpp"

enum FMUState {
    fmu_state_manual,
    fmu_state_searching,
    fmu_state_goto,
    fmu_state_altitude_adjust,
    fmu_state_rtl,
    fmu_state_hold,
    fmu_state_low_battery,
    fmu_state_failsafe,
    fmu_state_disarmed,
    fmu_state_terminate,
};

class FMUStateMachine {
private:
    void updateState();
    void actionState(FMUState state);
    FMUState current_state{fmu_state_manual};
    FSSCommand fss_command{fss_cmd_unknown};
    SMMCommand smm_command{smm_cmd_none};
    bool low_battery{false};
    bool fss_comms_lost{false};
    std::shared_ptr<MAV> mav{nullptr};
    std::shared_ptr<SMM> smm{nullptr};
    std::shared_ptr<FSS> fss{nullptr};
    std::mutex lock{};
public:
    FMUStateMachine(std::shared_ptr<MAV> t_mav, std::shared_ptr<SMM> t_smm, std::shared_ptr<FSS> t_fss);
    auto getCurrentstate() -> FMUState;

    void FSSNewCommand(FSSCommand cmd);
    void SMMNewCommand(SMMCommand cmd);
    void setLowBattery();
    void setCommsFailure(bool failed);
};