#pragma once
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "fmu-state-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "fss/ifss.hpp"
#include "mav/imav.hpp"
#include "smm/ismm.hpp"
#include "smm/smm-command.hpp"

/* Single-event-loop-thread invariant
 * -----------------------------------
 * Every state-mutating method below (FSSNewCommand, SMMNewCommand,
 * setLowBattery, setCommsFailure, setMavCommsFailure) and the work they drive
 * (updateState / actionState, which touch mav, smm and state_change_cb) run on
 * the ONE event-loop thread. All external inputs — FSS/SMM/MAV callbacks — are
 * funnelled through the App event queue and applied here on that thread, which
 * is what makes the priority arbitration and the state fields safe to reason
 * about without locking beyond the small guarded region that publishes to worker
 * threads (pending_replay_state). Calling any of these methods from another
 * thread (e.g. directly from a MAV/SMM/FSS callback instead of enqueuing an
 * event) would introduce a silent data race on the state fields.
 * assert_event_loop_thread() defends this in debug builds (todo/52). */
class FMUStateMachine
{
  private:
    auto updateState () -> std::optional<FMUState>;
    /* The state the current FSS/SMM commands alone map to, ignoring the
     * priority latches (terminate, low battery, comms failsafe). This is the
     * "desired" state: what the operator asked for. updateState() uses it for
     * the comms-okay branch and FSSNewCommand() uses it to classify a command
     * as actioned vs superseded — both must agree, so the mapping (including
     * the searching -> SMM-command special case) lives here once. Must be
     * called with this->lock held. */
    auto commandedState () const -> FMUState;
    /* Classify how the most recent FSS command resolved against the priority
     * logic. desired is the state the command alone maps to (ignoring latches);
     * changed is what updateState() actually selected (nullopt when no
     * transition occurred). Must be called with this->lock held. */
    auto resolveFSSCommand (FMUState desired, const std::optional<FMUState> &changed) -> FSSCommandResolution;
    /* Carry out the side effects of entering `state` (command the MAV, cancel the
     * search, fire the state-change callback). Returns whether the MAV command was
     * transmitted; for a safety-critical state a false result is recorded in
     * pending_replay_state so it is re-sent once the MAV link recovers (todo/46).
     * Must be called WITHOUT this->lock held (it locks internally). */
    auto actionState (FMUState state) -> bool;
    /* Debug-only guard for the single-event-loop-thread invariant documented
     * above the class: assert this call runs on the event-loop thread that owns
     * the state machine (the first caller establishes it). Compiled to a no-op
     * under NDEBUG so flight builds carry zero overhead; the point is to trip
     * tests / CI / TSan the instant a state-machine method is called off the
     * event-loop thread. */
    void assert_event_loop_thread ();
#ifndef NDEBUG
    std::optional<std::thread::id> event_loop_thread_id{};
#endif
    FMUState current_state{ fmu_state_manual };
    /* A safety-critical state (RTL / failsafe / low-battery / terminate) whose MAV
     * command could not be transmitted (link down). Replayed when MAV comms
     * recover; empty once delivered or superseded. Guarded by this->lock. */
    std::optional<FMUState> pending_replay_state{};
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
    /* True while the FMU is in the searching state. The event loop uses this to
     * gate SMM worker outcomes (load-search / RTL-fallback): an outcome that
     * raced a higher-priority transition out of searching is dropped rather than
     * commanding the autopilot. Reads current_state under this->lock; since every
     * transition runs on the event loop too, the guard sees a consistent value
     * (todo/33). */
    auto isSearching () -> bool;
};
