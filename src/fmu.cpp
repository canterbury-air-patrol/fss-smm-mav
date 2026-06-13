#include "fmu.hpp"
#include <iostream>
#include <mutex>

auto
map_smm_state (SMMCommand cmd) -> FMUState
{
    FMUState new_state = fmu_state_rtl;

    switch (cmd)
    {
        case smm_cmd_abandon_search:
        case smm_cmd_none:
            new_state = fmu_state_searching;
            break;
        case smm_cmd_mission_complete:
        default:
            new_state = fmu_state_rtl;
            break;
    }
    return new_state;
}

auto
map_fss_state (FSSCommand cmd) -> FMUState
{
    FMUState new_state = fmu_state_searching;
    switch (cmd)
    {
        case fss_cmd_rtl:
            new_state = fmu_state_rtl;
            break;
        case fss_cmd_goto:
            new_state = fmu_state_goto;
            break;
        case fss_cmd_hold:
            new_state = fmu_state_hold;
            break;
        case fss_cmd_altitude:
            new_state = fmu_state_altitude_adjust;
            break;
        case fss_cmd_disarm:
            new_state = fmu_state_disarmed;
            break;
        case fss_cmd_manual:
            new_state = fmu_state_manual;
            break;
        case fss_cmd_terminate:
            new_state = fmu_state_terminate;
            break;
        case fss_cmd_continue:
        case fss_cmd_unknown:
            new_state = fmu_state_searching;
            break;
    }
    return new_state;
}

auto
FMUStateMachine::updateState () -> std::optional<FMUState>
{
    FMUState new_state;

    /* terminate is the highest-priority command: it overrides low_battery and
     * comms_failure so the ground station can always halt the aircraft. */
    if (this->fss_command == fss_cmd_terminate)
    {
        new_state = fmu_state_terminate;
    }
    else if (this->low_battery)
    {
        new_state = fmu_state_low_battery;
    }
    else if (this->fss_comms_lost || this->mav_comms_lost)
    {
        new_state = fmu_state_failsafe;
    }
    else
    {
        new_state = map_fss_state (this->fss_command);
        if (new_state == fmu_state_searching)
        {
            new_state = map_smm_state (this->smm_command);
        }
    }

    if (new_state != this->current_state)
    {
        this->current_state = new_state;
        return new_state;
    }
    return std::nullopt;
}

void
FMUStateMachine::actionState (FMUState state)
{
    if (state != fmu_state_searching)
    {
        this->smm.cancelSearch ();
    }
    switch (state)
    {
        case fmu_state_manual:
            /* Tell MAV to exit auto mode */
            this->mav.setMode (flight_mode_manual);
            break;
        case fmu_state_searching:
            /* Tell SMM to implement the search */
            this->smm.search (this->mav.getCurrentPosition ());
            break;
        case fmu_state_rtl:
        case fmu_state_failsafe:
        case fmu_state_low_battery:
            /* Tell MAV to RTL */
            this->mav.setMode (flight_mode_rtl);
            break;
        case fmu_state_goto:
            /* Tell MAV to Goto the fss position */
            this->mav.gotoPosition (this->fss.getGoto ());
            this->mav.setMode (flight_mode_goto);
            break;
        case fmu_state_hold:
            /* Tell MAV to Circle/Hold Position */
            this->mav.setMode (flight_mode_hold);
            break;
        case fmu_state_altitude_adjust:
            /* Tell MAV to adjust the altitude */
            this->mav.setAltitude (this->fss.getAltitude ());
            break;
        case fmu_state_disarmed:
            /* Tell MAV to disarm the aircraft */
            this->mav.disarm ();
            break;
        case fmu_state_terminate:
            /* Tell MAV to terminate the flight */
            this->mav.terminate ();
            break;
    }

    if (this->state_change_cb)
    {
        this->state_change_cb (state);
    }
}

void
FMUStateMachine::FSSNewCommand (FSSCommand cmd)
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->fss_command = cmd;
        changed_to = this->updateState ();
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

void
FMUStateMachine::SMMNewCommand (SMMCommand cmd)
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->smm_command = cmd;
        changed_to = this->updateState ();
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

void
FMUStateMachine::setLowBattery ()
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->low_battery = true;
        changed_to = this->updateState ();
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

void
FMUStateMachine::setCommsFailure (bool failed)
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->fss_comms_lost = failed;
        changed_to = this->updateState ();
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

void
FMUStateMachine::setMavCommsFailure (bool failed)
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->mav_comms_lost = failed;
        changed_to = this->updateState ();
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

FMUStateMachine::FMUStateMachine (IMAV &t_mav, ISMM &t_smm, IFSS &t_fss)
    : mav (t_mav), smm (t_smm), fss (t_fss), state_change_cb{}
{
}

void
FMUStateMachine::setStateChangeCB (std::function<void (FMUState)> cb)
{
    std::lock_guard<std::mutex> lk (this->lock);
    this->state_change_cb = std::move (cb);
}