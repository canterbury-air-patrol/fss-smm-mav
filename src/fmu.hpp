#pragma once
#include <functional>
#include <mutex>
#include <optional>

#include "fmu-state-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "fss/ifss.hpp"
#include "mav/imav.hpp"
#include "smm/ismm.hpp"
#include "smm/smm-command.hpp"

class FMUStateMachine
{
  private:
    auto updateState () -> std::optional<FMUState>;
    /* Classify how the most recent FSS command resolved against the priority
     * logic. desired is the state the command alone maps to (ignoring latches);
     * changed is what updateState() actually selected (nullopt when no
     * transition occurred). Must be called with this->lock held. */
    auto resolveFSSCommand (FMUState desired, const std::optional<FMUState> &changed) -> FSSCommandResolution;
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
    /* Apply an FSS command and report how it resolved (actioned vs superseded
     * by a higher-priority latch), so the caller can acknowledge it to FSS. */
    auto FSSNewCommand (FSSCommand cmd) -> FSSCommandResolution;
    void SMMNewCommand (SMMCommand cmd);
    /* Report the latest battery reading's low/not-low state. Called for *every*
     * reading (not only low ones) so the consecutive-low run can be tracked:
     * low_battery_latch_count lows in a row engage a latched RTL, and any
     * not-low reading resets the run. The latch, once engaged, is not cleared. */
    void setLowBattery (bool low);
    void setCommsFailure (bool failed);
    void setMavCommsFailure (bool failed);
};
