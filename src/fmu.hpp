#pragma once
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "fmu-state-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "mav/imav.hpp"
#include "smm/ismm.hpp"
#include "smm/smm-command.hpp"

/* Single-event-loop-thread invariant (see docs/threading.md for the full
 * cross-thread model this is one piece of)
 * -----------------------------------
 * Every state-mutating method below (FSSNewCommand, SMMNewCommand,
 * setLowBattery, setCommsFailure, setMavCommsFailure, setCurrentAltitude) and
 * the work they drive (updateState / actionState, which touch mav, smm and
 * state_change_cb) run on the ONE event-loop thread. All external inputs —
 * FSS/SMM/MAV callbacks — are funnelled through the App event queue and applied
 * here on that thread, which is what makes the priority arbitration and the
 * state fields safe to reason about without a lock held across the whole
 * update. this->lock still guards those fields (docs/threading.md's "Data
 * ownership and locks" table is the inventory), but nothing below publishes
 * them to a worker thread, and nothing ever did: the replay bookkeeping this
 * paragraph used to cite as the one such region was itself only ever touched
 * from the event loop. Every acquisition of the lock is therefore made from
 * that one thread, so what it buys is not exclusion against a concurrent
 * reader but a scope short enough to close before actionState() runs — which
 * is what keeps the MAV/SMM calls actionState() makes out of a critical
 * section (docs/threading.md, "Lock acquisition order"). Its own unlocked
 * reads of fss_command_target and state_change_cb rest on the same invariant:
 * the thread that wrote them is the thread reading them. Calling any of these
 * methods from another thread (e.g. directly from a MAV/SMM/FSS callback
 * instead of enqueuing an event) would introduce a silent data race on the
 * state fields. assert_event_loop_thread() defends this in debug builds. */
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
     * transmitted; a false result is not recorded anywhere, because the recovery
     * rule no longer depends on it — every MAV-link recovery re-applies the
     * current state whether or not the earlier send got through, generalising an
     * earlier, narrower replay-on-failure rule. Must be called WITHOUT this->lock
     * held — it no longer takes the lock itself (it did, to record the replay
     * bookkeeping that has since gone), so the requirement is not deadlock
     * avoidance: it is that the IMAV/ISMM calls below must not be made from
     * inside a critical section (docs/threading.md, "Lock acquisition order"),
     * and that the fields it reads unlocked (fss_command_target,
     * state_change_cb) are only safe to read that way on the event-loop
     * thread. */
    auto actionState (FMUState state) -> bool;
    /* Debug-only guard for the single-event-loop-thread invariant documented
     * above the class: assert this call runs on the event-loop thread that owns
     * the state machine. Compiled to a no-op under NDEBUG so flight builds carry
     * zero overhead; the point is to trip tests / CI / TSan the instant a
     * state-machine method is called off the event-loop thread. */
    void assert_event_loop_thread ();
#ifndef NDEBUG
    /* The owning event-loop thread, captured at construction (the state machine
     * is always constructed on that thread, before it is published to any other).
     * Write-once here and read-only in assert_event_loop_thread(), so the guard
     * cannot itself race — a lazy first-write would be a data race under the very
     * cross-thread misuse it is meant to detect. */
    const std::thread::id event_loop_thread_id{ std::this_thread::get_id () };
#endif
    /* The state fields below are all accessed under this->lock (declared with
     * the rest of the members further down). docs/threading.md's "Data
     * ownership and locks" table is the canonical inventory of what that lock
     * guards: adding a field here means adding it there too, or the document
     * that contributors are told to reason from before running TSan quietly
     * stops being the complete map it claims to be — which is exactly how
     * `terminated` came to be missing from it. */
    FMUState current_state{ fmu_state_manual };
    FSSCommand fss_command{ fss_cmd_unknown };
    /* Target of the most recent goto/altitude FSS command, retained so
     * actionState can command the MAV whenever the goto/altitude state is
     * (re-)entered — including a later transition (e.g. a comms latch clearing)
     * that re-applies the stored fss_command. Set alongside fss_command in
     * FSSNewCommand; the command carries its own target through the event queue
     * rather than the state machine reading it back from FSS. Only the field
     * matching fss_command is meaningful. */
    FSSCommandTarget fss_command_target{};
    SMMCommand smm_command{ smm_cmd_none };
    int low_battery_count{ 0 };
    int low_battery_latch_count;
    bool low_battery{ false };
    int altitude_breach_count{ 0 };
    int altitude_clear_count{ 0 };
    int altitude_breach_latch_count;
    uint16_t altitude_cap_m;
    /* Unlike low_battery/terminated, this latch is self-clearing: a
     * sustained run of under-cap readings clears it again, returning control
     * to whatever FSS/SMM command is current. Modelled on the comms-failsafe
     * precedent (fss_comms_lost/mav_comms_lost below), not the restart-only
     * latches: a breach RTL should not be an un-overridable latch. */
    bool altitude_breach{ false };
    /* Latches true the first time fss_command is seen as fss_cmd_terminate and
     * is never cleared: the flight-termination action (motor cut / parachute /
     * force-disarm) is physically irreversible, so a later FSS command
     * silently moving the FMU's own state back out of terminate would be
     * misleading (FSS telemetry would claim e.g. "searching" for an aircraft
     * that already terminated) even though it cannot undo the airframe action.
     * Recovery requires an FMU restart, matching the low_battery latch. */
    bool terminated{ false };
    /* Starts ENGAGED, unlike mav_comms_lost: "we have never heard from FSS" is
     * the same thing as "we have lost FSS", so the comms-loss failsafe is armed
     * from construction and only a real fss_comms_okay report clears it. The
     * optimistic init this replaces meant a cold-started FMU that had never
     * reached a server still resolved FSS/SMM commands normally — and, with
     * fss_cmd_unknown mapping to searching (see map_fss_state), self-tasked into
     * a search having never been told to by anyone.
     *
     * The flag alone is not enough: nothing evaluates it until some event calls
     * updateState(), so fss_client_ssl::registerCommsStatusCB() also emits an
     * initial fss_comms_failure at startup. That report is what actually drives
     * the failsafe RTL — it matters for an FMU restarted in flight with FSS
     * unreachable, where the MAV link is healthy and no other event would ever
     * evaluate the state. The two together are the FSS equivalent of the MAV
     * side's mav_comms_ok{true} initial-report forcing. */
    bool fss_comms_lost{ true };
    bool mav_comms_lost{ false };
    IMAV &mav;
    ISMM &smm;
    std::mutex lock{};
    std::function<void (FMUState)> state_change_cb;

  public:
    /* Default number of consecutive low-battery readings that engage the RTL
     * latch, used unless the constructor is given a different value (from
     * FmuConfig::low_battery_latch_count). A single noisy/spurious sample must
     * not ground the mission, so the latch only trips once this many low
     * readings arrive in a row; one healthy reading in between resets the run.
     * Public so tests stay in step with it. */
    static constexpr int default_low_battery_latch_count = 4;
    /* Default consecutive over-/under-cap AGL readings to trip, or clear, the
     * altitude-cap breach latch. At the default 200ms (5Hz)
     * GLOBAL_POSITION_INT stream that's ~1s -- tighter than
     * default_low_battery_latch_count's ~4s (at the default 1000ms battery
     * stream), since a ceiling violation is more time-critical than a
     * battery reading, but still enough to reject single-sample EKF jitter.
     * The same count debounces both directions (symmetric hysteresis)
     * rather than adding a second knob. Public so tests stay in step. */
    static constexpr int default_altitude_breach_latch_count = 5;
    /* Matches FmuConfig::altitude_cap_m's default. */
    static constexpr uint16_t default_altitude_cap_m = 122;

    FMUStateMachine (IMAV &t_mav, ISMM &t_smm, int t_low_battery_latch_count = default_low_battery_latch_count,
                     uint16_t t_altitude_cap_m = default_altitude_cap_m,
                     int t_altitude_breach_latch_count = default_altitude_breach_latch_count);

    void setStateChangeCB (std::function<void (FMUState)> cb);
    /* Apply an FSS command (carrying its own goto/altitude target, default for
     * commands with none) and report how it resolved (actioned vs superseded by a
     * higher-priority latch), so the caller can acknowledge it to FSS. */
    auto FSSNewCommand (FSSCommand cmd, const FSSCommandTarget &target = {}) -> FSSCommandResolution;
    void SMMNewCommand (SMMCommand cmd);
    /* Report the latest battery reading's low/not-low state. Called for *every*
     * reading (not only low ones) so the consecutive-low run can be tracked:
     * low_battery_latch_count lows in a row engage a latched RTL, and any
     * not-low reading resets the run. The latch, once engaged, is not cleared. */
    void setLowBattery (bool low);
    void setCommsFailure (bool failed);
    /* Report the MAV link's health. On a genuine down->up edge the current state
     * is re-commanded to the autopilot unconditionally -- whether or not the
     * earlier send succeeded, and whether or not the recovery produced a state
     * transition. A link gap is not distinguishable from an autopilot reboot
     * from here, so the safe reading is that the autopilot came back with no
     * memory: a redundant SET_MODE costs nothing, a low-battery RTL that is
     * never re-sent costs the aircraft. */
    void setMavCommsFailure (bool failed);
    /* Re-apply the current state to the autopilot with no state change: the
     * autopilot restarted underneath us and has forgotten its mode and its
     * mission, but nothing about the FMU's own state is stale. Runs the full
     * side effects of entering that state -- including the SMM search/cancel
     * calls and the state-change log -- because the restart invalidated all of
     * them; SMM::doSearch()'s held-search resume path makes the searching case
     * idempotent. */
    void reassertState ();
    /* Report the latest own-aircraft AGL altitude reading and whether it is
     * backed by a valid fix. Called for *every* own-ship position report
     * (not just over-cap ones), mirroring setLowBattery, so the consecutive
     * over-/under-cap runs stay accurate. A fix_valid == false reading, or a
     * non-finite altitude_agl_m (NaN/Inf), is ignored entirely -- it neither
     * trips nor clears the latch, and does not disturb an in-progress
     * debounce run -- since a GPS gap or a garbled sample must not
     * false-trigger a breach, and must not silently clear a real one either.
     * A negative altitude_agl_m is NOT rejected: AGL can be legitimately
     * negative just after takeoff or on sloped terrain. */
    void setCurrentAltitude (bool fix_valid, double altitude_agl_m);
    /* True while the FMU is in the searching state. The event loop uses this to
     * gate SMM worker outcomes (load-search / RTL-fallback): an outcome that
     * raced a higher-priority transition out of searching is dropped rather than
     * commanding the autopilot. Reads current_state under this->lock; since every
     * transition runs on the event loop too, the guard sees a consistent value.
     */
    auto isSearching () -> bool;
    /* True while the FMU is waiting for tasking: SMM reported it has nothing
     * to search right now (a completed search with none queued behind it, or
     * a failed acquire attempt). The aircraft is flying the same RTL flight
     * mode as fmu_state_rtl, but -- unlike a real RTL -- the SMM searching
     * role was deliberately left granted, so SMM's own background
     * acquire-retry loop keeps running. The event loop uses this to resume
     * searching (rather than re-uploading directly) when SMM reports a
     * freshly (re)acquired search. Reads current_state under this->lock. */
    auto isWaitingForTasking () -> bool;
};
