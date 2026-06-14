#pragma once
#include <functional>
#include <mutex>
#include <optional>

#include "fss/fmu-fss-types.hpp"
#include "fss/ifss.hpp"
#include "mav/imav.hpp"
#include "smm/ismm.hpp"
#include "smm/smm-command.hpp"

enum FMUState
{
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

class FMUStateMachine
{
  private:
    auto updateState () -> std::optional<FMUState>;
    void actionState (FMUState state);
    FMUState current_state{ fmu_state_manual };
    FSSCommand fss_command{ fss_cmd_unknown };
    SMMCommand smm_command{ smm_cmd_none };
    int low_battery_count{ 0 };
    bool low_battery{ false };
    bool fss_comms_lost{ false };
    bool mav_comms_lost{ false };
    IMAV &mav;
    ISMM &smm;
    IFSS &fss;
    std::mutex lock{};
    std::function<void (FMUState)> state_change_cb;

  public:
    /* Number of consecutive low-battery readings that engage the RTL latch. A
     * single noisy/spurious sample must not ground the mission, so the latch
     * only trips once this many low readings arrive in a row; one healthy
     * reading in between resets the run. Public so tests stay in step with it. */
    static constexpr int low_battery_latch_count = 4;

    FMUStateMachine (IMAV &t_mav, ISMM &t_smm, IFSS &t_fss);

    void setStateChangeCB (std::function<void (FMUState)> cb);
    void FSSNewCommand (FSSCommand cmd);
    void SMMNewCommand (SMMCommand cmd);
    /* Report the latest battery reading's low/not-low state. Called for *every*
     * reading (not only low ones) so the consecutive-low run can be tracked:
     * low_battery_latch_count lows in a row engage a latched RTL, and any
     * not-low reading resets the run. The latch, once engaged, is not cleared. */
    void setLowBattery (bool low);
    void setCommsFailure (bool failed);
    void setMavCommsFailure (bool failed);
};
