#pragma once

#include "event.hpp"
#include "fmu.hpp"
#include "fss/ifss.hpp"
#include "logger.hpp"
#include "mav/imav.hpp"
#include "smm/ismm.hpp"
#include <string>

/* The event-loop's routing policy: which events reach the state machine,
 * which SMM/MAV outcomes are gated on isSearching(), what gets logged, how
 * battery readings are classified. Extracted from App::run()'s std::visit
 * block (todo/76) so it can be driven by the existing FMUStateMachine
 * mocks in tests/fmu_test.cpp without a real FSS/MAV/SMM (sockets, config
 * files). App keeps the queue/thread/signal plumbing and callback
 * registration; this only applies one already-dequeued event.
 *
 * Registers itself as the state machine's state-change callback at
 * construction (folding in what App::run() used to do inline), since
 * clearing smm_rtl_replay_pending on leaving fmu_state_searching is part of
 * the same dispatch policy this class owns. */
class EventDispatcher
{
  public:
    EventDispatcher (FMUStateMachine &t_state_machine, IMAV &t_mav, ISMM &t_smm, IFSSReporter &t_fss, Logger &t_logger,
                     std::string t_asset_name, int t_lowbat_threshold);

    void dispatch (const event &e);

  private:
    FMUStateMachine &state_machine;
    IMAV &mav;
    ISMM &smm;
    IFSSReporter &fss;
    Logger &logger;
    std::string asset_name;
    int lowbat_threshold;
    /* Set when an SmmRtl-driven mav.setMode(flight_mode_rtl) fails to send
     * (MAV link down); retried when MavCommsStatus reports the link okay
     * again, mirroring FMUStateMachine's pending_replay_state (todo/46) for
     * this one action that bypasses the state machine (todo/71/todo/70).
     * Cleared whenever the FMU leaves fmu_state_searching so a stale RTL
     * intent cannot fire after a later, unrelated command took over. */
    bool smm_rtl_replay_pending{ false };
};
