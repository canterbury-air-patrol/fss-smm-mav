#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "aircraft.hpp"
#include "altitude-units.hpp"
#include "fmu-config.hpp"
#include "fmu.hpp"
#include "fss/command-ack-group.hpp"
#include "fss/command-ack.hpp"
#include "mav/battery-voltage.hpp"
#include "mav/internal.hpp"
#include "mav/mav-comms.hpp"
#include "mav/mission-plan.hpp"
#include "mav/mode-resolve.hpp"
#include "smm/connection-state.hpp"
#include "smm/search-acquire.hpp"
#include "smm/search-altitude.hpp"

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>

class MockMAV : public IMAV
{
  public:
    flight_mode last_mode{ flight_mode_unknown };
    int set_mode_calls{ 0 };
    bool disarmed{ false };
    int disarm_calls{ 0 };
    bool terminated{ false };
    int terminate_calls{ 0 };
    uint16_t last_altitude{ 0 };
    int set_altitude_calls{ 0 };
    /* Controls the transmission result the action methods report. Set false to
     * simulate a send that did not reach the autopilot (MAV link down) so the
     * replay-on-recovery path (todo/46) can be exercised. */
    bool send_succeeds{ true };

    MockMAV () = default;
    MockMAV (const MockMAV &) = delete;
    MockMAV (MockMAV &&) = delete;
    auto operator= (const MockMAV &) -> MockMAV & = delete;
    auto operator= (MockMAV &&) -> MockMAV & = delete;
    ~MockMAV () override = default;

    auto
    setMode (flight_mode fm) -> bool override
    {
        last_mode = fm;
        set_mode_calls++;
        return send_succeeds;
    }
    auto
    disarm () -> bool override
    {
        disarmed = true;
        disarm_calls++;
        return send_succeeds;
    }
    auto
    terminate () -> bool override
    {
        terminated = true;
        terminate_calls++;
        return send_succeeds;
    }
    void
    gotoPosition (Point) override
    {
    }
    auto
    setAltitude (uint16_t alt) -> bool override
    {
        last_altitude = alt;
        set_altitude_calls++;
        return send_succeeds;
    }
    auto
    getCurrentPosition () -> Point override
    {
        return Point{};
    }
    void
    registerMavCommsStatusCB (notify_mav_comms_cb) override
    {
    }
};

class MockSMM : public ISMM
{
  public:
    int search_calls{ 0 };
    int cancel_calls{ 0 };

    MockSMM () = default;
    MockSMM (const MockSMM &) = delete;
    MockSMM (MockSMM &&) = delete;
    auto operator= (const MockSMM &) -> MockSMM & = delete;
    auto operator= (MockSMM &&) -> MockSMM & = delete;
    ~MockSMM () override = default;

    void
    search (Point) override
    {
        search_calls++;
    }
    void
    cancelSearch () override
    {
        cancel_calls++;
    }
};

class MockFSS : public IFSS
{
  public:
    Point goto_point{};
    uint16_t altitude_val{ 0 };

    MockFSS () = default;
    MockFSS (const MockFSS &) = delete;
    MockFSS (MockFSS &&) = delete;
    auto operator= (const MockFSS &) -> MockFSS & = delete;
    auto operator= (MockFSS &&) -> MockFSS & = delete;
    ~MockFSS () override = default;

    auto
    getGoto () -> Point override
    {
        return goto_point;
    }
    auto
    getAltitude () -> uint16_t override
    {
        return altitude_val;
    }
};

using SM = std::tuple<std::shared_ptr<MockMAV>, std::shared_ptr<MockSMM>, std::shared_ptr<MockFSS>,
                      std::shared_ptr<FMUStateMachine>>;

static auto
make_sm () -> SM
{
    auto mav = std::make_shared<MockMAV> ();
    auto smm = std::make_shared<MockSMM> ();
    auto fss = std::make_shared<MockFSS> ();
    auto sm = std::make_shared<FMUStateMachine> (*mav, *smm, *fss);
    return { mav, smm, fss, sm };
}

/* The state machine debounces low-battery readings: the RTL latch only engages
 * after low_battery_latch_count consecutive low samples. Drive exactly that many
 * so tests that assume a latched low battery stay correct if the count changes. */
static void
latch_low_battery (const std::shared_ptr<FMUStateMachine> &sm)
{
    for (int i = 0; i < FMUStateMachine::low_battery_latch_count; i++)
    {
        sm->setLowBattery (true);
    }
}

TEST_CASE ("low battery latches RTL regardless of subsequent FSS commands", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_goto);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_manual);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("comms failure latches failsafe until comms restored", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->setCommsFailure (false);
    REQUIRE (mav->last_mode == flight_mode_hold);
}

TEST_CASE ("fss_cmd_continue with smm_cmd_none leads to searching", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();
    int calls_before = smm->search_calls;

    sm->FSSNewCommand (fss_cmd_continue);

    REQUIRE (smm->search_calls > calls_before);
}

TEST_CASE ("fss_cmd_continue with smm_cmd_abandon_search leads to searching", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->SMMNewCommand (smm_cmd_abandon_search);
    sm->FSSNewCommand (fss_cmd_continue);

    REQUIRE (smm->search_calls > 0);
}

/* The search is paused (not abandoned) by an interrupting command, so returning
 * to searching via `continue` must re-invoke SMM::search to resume it. This pins
 * the state-machine half of todo/50; the SMM half (re-issuing the mission upload)
 * is covered in mav_io_test. */
TEST_CASE ("continue after a hold re-invokes the search so it can resume", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    /* hold interrupts the search... */
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    /* ...and continue must drive another search() call to resume it. */
    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 2);
}

/* Override matrix: the searching state defers to the SMM command, and any higher
 * priority input (an explicit FSS command, low battery, or comms failure) takes
 * over an active search. These cover the transitions into/out of searching; the
 * pause/resume behaviour of the search itself is tracked separately (todo/50). */
TEST_CASE ("smm_cmd_mission_complete overrides an active search with RTL", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    /* SMM reports the search done: the FMU leaves searching for RTL. */
    sm->SMMNewCommand (smm_cmd_mission_complete);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("an explicit FSS command overrides an active SMM search", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    /* hold maps directly (it never consults the SMM command), so it takes over
     * the search and the aircraft holds. */
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);
    REQUIRE (res.outcome == fss_command_actioned);

    /* rtl likewise overrides a search. */
    sm->FSSNewCommand (fss_cmd_continue);
    sm->FSSNewCommand (fss_cmd_rtl);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("low battery forces RTL out of an active search", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("comms failure forces RTL out of an active search", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* Design decision: terminate has the highest priority and overrides both
 * low_battery and comms_failure.  The ground station must always be able to
 * halt the aircraft, even during an emergency RTL. */
TEST_CASE ("terminate overrides low battery", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (!mav->terminated);

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
}

TEST_CASE ("terminate overrides comms failure", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (!mav->terminated);

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
}

/* Design decision: low_battery outranks comms_failure.  Both drive RTL, but
 * low_battery latches, so clearing comms (or sending FSS commands) must not
 * release the aircraft from RTL while the battery is still low. */
TEST_CASE ("low battery outranks comms failure", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->setCommsFailure (false);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* Design decision: low_battery BLOCKS manual and disarm.  Allowing manual
 * override or a mid-air disarm when the battery is critically low risks loss
 * of the aircraft; RTL is the safe action. */
TEST_CASE ("manual blocked when low battery is set", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    sm->FSSNewCommand (fss_cmd_manual);

    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("manual allowed when battery OK", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    /* The machine starts in fmu_state_manual, so move away first to make the
     * transition back to manual observable. */
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->FSSNewCommand (fss_cmd_manual);
    REQUIRE (mav->last_mode == flight_mode_manual);
}

TEST_CASE ("disarm allowed when battery OK", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_disarm);

    REQUIRE (mav->disarmed);
}

TEST_CASE ("disarm blocked when low battery is set", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    sm->FSSNewCommand (fss_cmd_disarm);

    REQUIRE (!mav->disarmed);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("same non-searching state does not re-action", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    int calls = mav->set_mode_calls;

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->set_mode_calls == calls);
}

TEST_CASE ("searching does not re-action on repeated state update", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    int calls = smm->search_calls;

    sm->SMMNewCommand (smm_cmd_none);
    REQUIRE (smm->search_calls == calls);
}

TEST_CASE ("goto does not re-action on repeated FSS goto", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_goto);
    int calls = mav->set_mode_calls;

    sm->FSSNewCommand (fss_cmd_goto);
    REQUIRE (mav->set_mode_calls == calls);
}

TEST_CASE ("altitude adjust command calls mav setAltitude", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    fss->altitude_val = 150;
    sm->FSSNewCommand (fss_cmd_altitude);

    REQUIRE (mav->last_altitude == 150);
}

TEST_CASE ("altitude adjust does not re-action on repeated FSS altitude", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    fss->altitude_val = 150;
    sm->FSSNewCommand (fss_cmd_altitude);
    int calls = mav->set_altitude_calls;

    sm->FSSNewCommand (fss_cmd_altitude);
    REQUIRE (mav->set_altitude_calls == calls);
}

/* Design decision: the low-battery RTL latch is debounced. A single noisy or
 * spurious low reading must not ground the mission; the latch only engages
 * after low_battery_latch_count consecutive low samples. */
TEST_CASE ("low battery does not latch before the debounce count", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    /* One short of the count is not enough to latch. */
    for (int i = 0; i < FMUStateMachine::low_battery_latch_count - 1; i++)
    {
        sm->setLowBattery (true);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);

    /* The final consecutive reading trips the latch. */
    sm->setLowBattery (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("a healthy battery reading resets the low battery debounce", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);

    /* Almost enough to latch... */
    for (int i = 0; i < FMUStateMachine::low_battery_latch_count - 1; i++)
    {
        sm->setLowBattery (true);
    }
    /* ...but a healthy reading clears the run, so the count restarts from zero. */
    sm->setLowBattery (false);
    for (int i = 0; i < FMUStateMachine::low_battery_latch_count - 1; i++)
    {
        sm->setLowBattery (true);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setLowBattery (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* Once the latch has engaged, a later optimistic reading must NOT release it:
 * recovery from a critically low battery requires a restart. */
TEST_CASE ("low battery latch is not cleared by a healthy reading", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->setLowBattery (false);
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* The consecutive-low counter saturates at low_battery_latch_count so a long
 * flight with a sustained low battery cannot overflow it. The latch must engage
 * exactly once and stay engaged no matter how many more low readings arrive. */
TEST_CASE ("low battery latch saturates and stays engaged over a long run", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    /* Far more readings than the latch count, to exercise counter saturation. */
    for (int i = 0; i < 1000; i++)
    {
        sm->setLowBattery (true);
    }
    REQUIRE (mav->last_mode == flight_mode_rtl);
    /* The machine started in manual, so the single RTL transition is the only
     * setMode call; saturated readings past the latch point do not re-action. */
    REQUIRE (mav->set_mode_calls == 1);

    /* Still latched: a subsequent FSS command cannot release it. */
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (mav->set_mode_calls == 1);
}

TEST_CASE ("mav comms failure triggers failsafe RTL", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setMavCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* The failsafe must be idempotent: once a comms failure has driven the RTL,
 * further missed heartbeats (repeated setMavCommsFailure(true)) must not
 * re-action it. Asserts the internal FMUState via the state-change callback,
 * not just the mav side effect. */
TEST_CASE ("repeated mav comms failures do not re-action the failsafe", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    FMUState last_state = fmu_state_manual;
    sm->setStateChangeCB ([&] (FMUState s) { last_state = s; });

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (last_state == fmu_state_hold);

    sm->setMavCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (last_state == fmu_state_failsafe);
    int mode_calls = mav->set_mode_calls;

    /* Subsequent missed heartbeats while already failed change nothing. */
    sm->setMavCommsFailure (true);
    sm->setMavCommsFailure (true);
    REQUIRE (mav->set_mode_calls == mode_calls);
    REQUIRE (last_state == fmu_state_failsafe);
}

TEST_CASE ("mav comms failure clears and restores prior FSS command", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setMavCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->setMavCommsFailure (false);
    REQUIRE (mav->last_mode == flight_mode_hold);
}

TEST_CASE ("terminate overrides mav comms failure", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->setMavCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (!mav->terminated);

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
}

TEST_CASE ("mav comms failure does not displace an active terminate", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
    int mode_calls = mav->set_mode_calls;

    /* A subsequent comms failure must not pull the aircraft out of terminate
     * into RTL — terminate is the highest priority. */
    sm->setMavCommsFailure (true);
    REQUIRE (mav->set_mode_calls == mode_calls);
}

TEST_CASE ("disarm command is actioned when no emergency", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_disarm);
    REQUIRE (mav->disarmed);
}

TEST_CASE ("state_change_cb fires once per state change, not on no-op transitions", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    int transitions = 0;
    FMUState last_seen = fmu_state_manual;
    sm->setStateChangeCB (
        [&] (FMUState s)
        {
            transitions++;
            last_seen = s;
        });

    sm->FSSNewCommand (fss_cmd_hold);
    sm->FSSNewCommand (fss_cmd_goto);
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (transitions == 3);
    REQUIRE (last_seen == fmu_state_hold);

    /* A repeated command is not a transition, so the callback must not fire. */
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (transitions == 3);
}

/* The resolution returned by FSSNewCommand drives the command acknowledgement
 * sent back to FSS: a command that takes effect is actioned, one blocked by a
 * higher-priority latch is superseded (and names the blocking state). */
TEST_CASE ("FSS command that transitions resolves as actioned", "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_actioned);
    REQUIRE (res.transitioned);
}

TEST_CASE ("FSS command already in the target state resolves as actioned no-op", "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_actioned);
    /* The aircraft is in the commanded state, but this command did not move it. */
    REQUIRE (!res.transitioned);
}

TEST_CASE ("FSS command superseded by low battery names the latch", "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_superseded);
    REQUIRE (res.superseding_state == fmu_state_low_battery);
}

TEST_CASE ("FSS command superseded by comms failsafe names the latch", "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->setCommsFailure (true);
    auto res = sm->FSSNewCommand (fss_cmd_goto);
    REQUIRE (res.outcome == fss_command_superseded);
    REQUIRE (res.superseding_state == fmu_state_failsafe);
}

/* todo/43 decision: an operator RTL whose effect matches an active latch (which
 * also flies RTL) is still reported "superseded", not "actioned" — the latch,
 * not the command, is in control, and the GS surfaces it as the latch's RTL in
 * effect. These tests pin that decision so it cannot silently regress. */
TEST_CASE ("an explicit RTL during the low-battery latch is reported superseded", "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    /* The operator asks for RTL and the aircraft is in fact flying RTL, yet the
     * latch (not the command) is in control, so the ack names the latch. */
    auto res = sm->FSSNewCommand (fss_cmd_rtl);
    REQUIRE (res.outcome == fss_command_superseded);
    REQUIRE (res.superseding_state == fmu_state_low_battery);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("an RTL superseded by a recoverable comms failure still applies once comms clears",
           "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    /* Baseline so "the commanded RTL applied on recovery" is observable: without
     * the RTL below, recovery would restore this hold. */
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    /* A (recoverable) comms failure engages the failsafe RTL latch. */
    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    /* The operator commands RTL during the failure: the latch is in control, so
     * the ack is superseded (decision above) ... */
    auto res = sm->FSSNewCommand (fss_cmd_rtl);
    REQUIRE (res.outcome == fss_command_superseded);
    REQUIRE (res.superseding_state == fmu_state_failsafe);

    /* ... but the command is retained, so when the recoverable comms failure
     * clears the aircraft holds RTL rather than reverting to the earlier hold.
     * The operator's intent is honoured once the latch releases. */
    sm->setCommsFailure (false);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* terminate is an FSS command occupying the same input slot as every other FSS
 * command, so a later FSS command replaces it rather than being superseded by
 * it. (The terminate priority in updateState() guards against the *concurrent*
 * latches — low battery and comms failure — not against a newer FSS command.) */
TEST_CASE ("FSS command after terminate replaces it and resolves as actioned", "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_terminate);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_actioned);
    REQUIRE (res.transitioned);
}

TEST_CASE ("terminate command itself resolves as actioned", "[state_machine][command_ack]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    auto res = sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (res.outcome == fss_command_actioned);
    REQUIRE (res.transitioned);
}

/* The resolution maps onto the transport-domain ack fields FSS receives: a real
 * transition is actioned, an already-in-state command is noop (not a phantom
 * transition), and a superseded command carries the dedicated latch reason that
 * keeps low-battery and comms-loss RTL distinguishable. */
namespace fsst = flight_safety_system::transport;

TEST_CASE ("ack mapping: a transitioning command is actioned with no reason", "[command_ack][mapping]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_actioned);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_none);
}

TEST_CASE ("ack mapping: an already-in-state command is noop", "[command_ack][mapping]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_noop);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_none);
}

TEST_CASE ("ack mapping: low-battery supersede carries the low-battery reason", "[command_ack][mapping]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    latch_low_battery (sm);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_superseded);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_low_battery);
}

TEST_CASE ("ack mapping: comms-loss supersede carries the comms-loss reason", "[command_ack][mapping]")
{
    auto [mav, smm, fss, sm] = make_sm ();

    sm->setCommsFailure (true);
    auto res = sm->FSSNewCommand (fss_cmd_goto);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_superseded);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_comms_loss);
}

/* The phase-2 ack responder built in handleCommandFrom is invoked later, from
 * the event-loop thread, after the originating fss_server may have been
 * destroyed (comms-loss teardown / updateServers). The fix carries the
 * connection across that boundary as a weak_ptr instead of a raw fss_server*,
 * so a freed connection is observed as an expired weak_ptr (a silent no-op)
 * rather than dereferenced. fss_client_ssl and the SSL transport are not linked
 * into this test target, so we exercise the exact capture/lock idiom directly:
 * a moved-from connection must lock() to nullptr and the responder must skip the
 * send without touching freed memory. */
TEST_CASE ("ack responder over an expired connection is a silent no-op", "[command_ack][lifetime]")
{
    auto conn = std::make_shared<int> (0); // stand-in for the fss_connection
    std::weak_ptr<int> weak_conn = conn;

    int sends = 0;
    /* Mirrors the lambda in handleCommandFrom: capture the connection weakly,
     * lock() at invocation time, and only send when it is still alive. */
    auto responder = [weak_conn, &sends] ()
    {
        auto locked = weak_conn.lock ();
        if (locked == nullptr)
        {
            return; // connection torn down: stay silent, do not dereference
        }
        sends++;
    };

    responder ();
    REQUIRE (sends == 1); // connection alive: ack would be sent

    conn.reset (); // originating server/connection destroyed before resolution
    REQUIRE (weak_conn.expired ());
    responder (); // must not crash or send
    REQUIRE (sends == 1);
}

/* CommandAckGroup groups the redundant per-server deliveries of one logical
 * command so every copy is acked with the same terminal outcome (the FMU is
 * connected to all FSS servers and the web frontend pushes the command to each).
 * These tests use a lightweight integer target standing in for the per-copy ack
 * target (weak_ptr<fss_connection> + acked_id) so the bookkeeping is exercised
 * without the SSL transport. The command-identity value is an arbitrary int. */
namespace
{
constexpr uint64_t group_tolerance_ms = 60000;

auto
actioned () -> FSSCommandResolution
{
    FSSCommandResolution res;
    res.outcome = fss_command_actioned;
    res.transitioned = true;
    return res;
}
} // namespace

TEST_CASE ("same command on two connections actions once, both acked the same", "[command_ack][group]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_hold = 5;

    /* First server's copy: a new command, so the FMU actuates it once. */
    auto first = group.onDelivery (cmd_hold, 1000, /*copy=*/1);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);
    REQUIRE (first.superseded.empty ());

    /* Second server's copy of the SAME command (equal timestamp): must NOT
     * actuate again, just join the group pending the outcome. */
    auto second = group.onDelivery (cmd_hold, 1000, /*copy=*/2);
    REQUIRE (second.disposition == CommandAckGroup<int>::Disposition::pending);

    /* The state machine resolves once; both copies are returned to be acked with
     * that single outcome — neither is left unacked. */
    auto to_ack = group.resolve (first.epoch, actioned ());
    REQUIRE (to_ack.size () == 2);
    REQUIRE (to_ack[0] == 1);
    REQUIRE (to_ack[1] == 2);
}

TEST_CASE ("a duplicate arriving after resolution is acked immediately from the cache", "[command_ack][group]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_goto = 3;

    auto first = group.onDelivery (cmd_goto, 1000, 1);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);

    auto resolved = group.resolve (first.epoch, actioned ());
    REQUIRE (resolved.size () == 1);

    /* A slow server delivers its copy only after the command already resolved:
     * the cached outcome is replayed to it straight away, not left pending. */
    auto late = group.onDelivery (cmd_goto, 1000, 2);
    REQUIRE (late.disposition == CommandAckGroup<int>::Disposition::already_resolved);
    REQUIRE (late.resolution.has_value ());
    REQUIRE (late.resolution->outcome == fss_command_actioned);
}

TEST_CASE ("an older different command within the window is acked superseded, not actuated", "[command_ack][group]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_hold = 5;
    constexpr int cmd_rtl = 1;

    auto current = group.onDelivery (cmd_hold, 10000, 1);
    REQUIRE (current.disposition == CommandAckGroup<int>::Disposition::actuate);

    /* A different, strictly-older command from a slow server (within tolerance):
     * the newer command already won, so do NOT actuate — but it must still be
     * acked (superseded) rather than dropped silently. */
    auto stale = group.onDelivery (cmd_rtl, 9000, 2);
    REQUIRE (stale.disposition == CommandAckGroup<int>::Disposition::stale_superseded);

    /* handleCommandFrom acks that stale delivery with the dedicated newer-command
     * reason (a later operator command replaced it), not supersede_none and not a
     * safety-latch reason. These shared constants are exactly what it sends. */
    REQUIRE (stale_command_ack_outcome == fsst::command_ack_superseded);
    REQUIRE (stale_command_ack_reason == fsst::supersede_newer_command);
    REQUIRE (stale_command_ack_reason != fsst::supersede_none);
    REQUIRE (stale_command_ack_reason != fsst::supersede_low_battery);
    REQUIRE (stale_command_ack_reason != fsst::supersede_comms_loss);
}

TEST_CASE ("a new command supersedes an unresolved group and its copies are handed back", "[command_ack][group]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_hold = 5;
    constexpr int cmd_rtl = 1;

    /* Two copies of the first command arrive but it never resolves... */
    auto first = group.onDelivery (cmd_hold, 1000, 1);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);
    auto first_dup = group.onDelivery (cmd_hold, 1000, 2);
    REQUIRE (first_dup.disposition == CommandAckGroup<int>::Disposition::pending);

    /* ...before a genuinely new command arrives. The new command actuates and
     * hands back the old group's still-pending copies so the caller acks them as
     * superseded; none is left unacked. */
    auto second = group.onDelivery (cmd_rtl, 2000, 3);
    REQUIRE (second.disposition == CommandAckGroup<int>::Disposition::actuate);
    REQUIRE (second.superseded.size () == 2);
    REQUIRE (second.superseded[0] == 1);
    REQUIRE (second.superseded[1] == 2);

    /* This is the same situation as the stale_superseded path — an older command
     * replaced by a newer operator command — differing only in arrival timing.
     * handleCommandFrom must therefore ack these displaced copies with the SAME
     * shared outcome+reason it uses there, so the operator-facing label is
     * 'superseded by newer command' regardless of timing, never supersede_none. */
    REQUIRE (stale_command_ack_outcome == fsst::command_ack_superseded);
    REQUIRE (stale_command_ack_reason == fsst::supersede_newer_command);
    REQUIRE (stale_command_ack_reason != fsst::supersede_none);

    /* The first command's late resolution is dropped (its epoch is stale): those
     * copies were already acked as superseded, so it must not double-ack them. */
    auto stale_resolution = group.resolve (first.epoch, actioned ());
    REQUIRE (stale_resolution.empty ());

    /* The new command resolves normally for its own copy. */
    auto live_resolution = group.resolve (second.epoch, actioned ());
    REQUIRE (live_resolution.size () == 1);
    REQUIRE (live_resolution[0] == 3);
}

TEST_CASE ("same goto payload from two connections actions once and acks both", "[command_ack][group]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_goto = 3;
    const auto target = CommandPayload::forPosition (-43.5, 172.6);

    /* Two servers deliver the SAME goto (same target) within the window: actuate
     * once, the second joins the group pending the single outcome. */
    auto first = group.onDelivery (cmd_goto, 1000, 1, target);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);
    auto second = group.onDelivery (cmd_goto, 1000, 2, target);
    REQUIRE (second.disposition == CommandAckGroup<int>::Disposition::pending);

    auto to_ack = group.resolve (first.epoch, actioned ());
    REQUIRE (to_ack.size () == 2);
}

TEST_CASE ("a goto with a different target within the window actuates again", "[command_ack][group]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_goto = 3;

    auto first = group.onDelivery (cmd_goto, 1000, 1, CommandPayload::forPosition (-43.5, 172.6));
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);

    /* A re-targeted goto inside the 60 s tolerance is a NEW logical command, not a
     * duplicate, so it must actuate (and hand back the first copy as superseded)
     * rather than replay the first goto's cached ack. */
    auto retarget = group.onDelivery (cmd_goto, 2000, 2, CommandPayload::forPosition (-43.6, 172.7));
    REQUIRE (retarget.disposition == CommandAckGroup<int>::Disposition::actuate);
    REQUIRE (retarget.superseded.size () == 1);
    REQUIRE (retarget.superseded[0] == 1);
}

TEST_CASE ("an altitude command with a different value within the window actuates again", "[command_ack][group]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_altitude = 7;

    auto first = group.onDelivery (cmd_altitude, 1000, 1, CommandPayload::forAltitude (300));
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);

    /* Same altitude is a duplicate... */
    auto same = group.onDelivery (cmd_altitude, 1000, 2, CommandPayload::forAltitude (300));
    REQUIRE (same.disposition == CommandAckGroup<int>::Disposition::pending);

    /* ...but a different assigned altitude inside the window must actuate again. */
    auto changed = group.onDelivery (cmd_altitude, 2000, 3, CommandPayload::forAltitude (500));
    REQUIRE (changed.disposition == CommandAckGroup<int>::Disposition::actuate);
}

TEST_CASE ("altitude unit helpers convert between metres, feet and MAVLink mm", "[altitude_units]")
{
    /* 1 foot is exactly 0.3048 m, so 100 m is ~328.084 ft. */
    REQUIRE (metres_to_feet (100.0) == Catch::Approx (328.0839895));
    REQUIRE (feet_to_metres (328.0839895) == Catch::Approx (100.0));
    /* Round-trip a whole-foot value back to itself. */
    REQUIRE (metres_to_feet (feet_to_metres (500.0)) == Catch::Approx (500.0));

    /* MAVLink GLOBAL_POSITION_INT.alt / ADSB_VEHICLE.altitude are millimetres. */
    REQUIRE (mav_mm_to_metres (100000) == Catch::Approx (100.0));
    REQUIRE (metres_to_mav_mm (100.0) == 100000);
    /* metres_to_mav_mm rounds to the nearest millimetre. */
    REQUIRE (metres_to_mav_mm (1.2345) == 1235);

    /* A non-finite altitude must not reach std::lround (UB): report 0. */
    REQUIRE (metres_to_mav_mm (std::nan ("")) == 0);
    REQUIRE (metres_to_mav_mm (std::numeric_limits<double>::infinity ()) == 0);
    /* An out-of-range magnitude clamps to the int32_t bounds rather than wrapping. */
    REQUIRE (metres_to_mav_mm (1e12) == std::numeric_limits<int32_t>::max ());
    REQUIRE (metres_to_mav_mm (-1e12) == std::numeric_limits<int32_t>::min ());
}

TEST_CASE ("PositionData altitude is metres and converts correctly at each protocol boundary", "[altitude_units]")
{
    /* A position at exactly 100 m drives the three outbound boundaries. */
    PositionData pd (0.0, 0.0, 100.0, 0, 0, 0);
    REQUIRE (pd.getAltitudeMetres () == Catch::Approx (100.0));

    /* SMM is sent metres directly (this is the bug fix: previously feet were
     * forwarded mislabelled as metres). */
    REQUIRE (std::lround (pd.getAltitudeMetres ()) == 100);

    /* FSS is sent feet: 100 m -> 328 ft (rounded), matching the FSS web UI's
     * "Altitude (ft)" display. */
    REQUIRE (static_cast<int16_t> (std::lround (metres_to_feet (pd.getAltitudeMetres ()))) == 328);

    /* ADS-B (MAVLink ADSB_VEHICLE.altitude) is sent millimetres: 100 m -> 100000 mm. */
    REQUIRE (metres_to_mav_mm (pd.getAltitudeMetres ()) == 100000);
}

TEST_CASE ("battery_pack_voltage_v converts millivolts and maps the unknown sentinel", "[battery]")
{
    /* A normal cell-0 pack voltage converts millivolts to volts. */
    REQUIRE (battery_pack_voltage_v (22400) == Catch::Approx (22.4));
    REQUIRE (battery_pack_voltage_v (12600) == Catch::Approx (12.6));

    /* UINT16_MAX is MAVLink's "unknown" sentinel; it must not surface as
     * 65.535 V but as the FSS "unknown" convention (0.0). */
    REQUIRE (battery_pack_voltage_v (UINT16_MAX) == Catch::Approx (battery_voltage_unknown));
    REQUIRE (battery_voltage_unknown == Catch::Approx (0.0));

    /* An empty 0 mV slot already coincides with the unknown sentinel. */
    REQUIRE (battery_pack_voltage_v (0) == Catch::Approx (0.0));
}

TEST_CASE ("raw_search_altitude derives height from sweep width and FoV", "[altitude]")
{
    /* At a 90deg total FoV, tan(45deg) == 1, so altitude == sweep_width / 2. */
    REQUIRE (raw_search_altitude (200.0, 90.0) == Catch::Approx (100.0));
    REQUIRE (raw_search_altitude (100.0, 90.0) == Catch::Approx (50.0));

    /* A narrower FoV needs more altitude for the same sweep width. */
    REQUIRE (raw_search_altitude (100.0, 60.0) > raw_search_altitude (100.0, 90.0));
    /* h = width / (2 * tan(30deg)) = 100 / (2 * 0.57735) ~= 86.60 */
    REQUIRE (raw_search_altitude (100.0, 60.0) == Catch::Approx (86.6025).margin (0.01));

    /* Zero sweep width derives zero altitude (the floor clamp handles this). */
    REQUIRE (raw_search_altitude (0.0, 90.0) == Catch::Approx (0.0));
}

TEST_CASE ("raw_search_altitude returns 0 for an out-of-range FoV", "[altitude]")
{
    /* A FoV outside (0, 180) cannot yield a valid height; the helper returns 0
     * so the floor clamp takes over rather than producing a garbage value. */
    REQUIRE (raw_search_altitude (100.0, 0.0) == Catch::Approx (0.0));
    REQUIRE (raw_search_altitude (100.0, 180.0) == Catch::Approx (0.0));
    REQUIRE (raw_search_altitude (100.0, 200.0) == Catch::Approx (0.0));
    REQUIRE (raw_search_altitude (100.0, -10.0) == Catch::Approx (0.0));

    /* The floor clamp then pins it to the safe minimum. */
    REQUIRE (clamp_search_altitude (raw_search_altitude (100.0, 0.0), 10, 122) == 10);
}

TEST_CASE ("clamp_search_altitude holds the derived altitude within [floor, cap]", "[altitude]")
{
    /* Within range passes through (truncated to whole metres). */
    REQUIRE (clamp_search_altitude (100.0, 10, 122) == 100);
    REQUIRE (clamp_search_altitude (100.9, 10, 122) == 100);

    /* Above the cap clamps to the regulatory ceiling. */
    REQUIRE (clamp_search_altitude (500.0, 10, 122) == 122);

    /* Below the floor clamps up: a tiny or zero sweep width cannot put the
     * search at ground level. */
    REQUIRE (clamp_search_altitude (5.0, 10, 122) == 10);
    REQUIRE (clamp_search_altitude (0.0, 10, 122) == 10);

    /* Boundaries are inclusive. */
    REQUIRE (clamp_search_altitude (122.0, 10, 122) == 122);
    REQUIRE (clamp_search_altitude (10.0, 10, 122) == 10);

    /* When floor == cap the altitude is pinned to that single value. */
    REQUIRE (clamp_search_altitude (50.0, 122, 122) == 122);
    REQUIRE (clamp_search_altitude (200.0, 122, 122) == 122);
}

TEST_CASE ("search altitude derivation and clamp compose for realistic searches", "[altitude]")
{
    constexpr uint16_t floor = 10;
    constexpr uint16_t cap = 122;

    /* 200 m sweep at 90deg -> 100 m, within range. */
    REQUIRE (clamp_search_altitude (raw_search_altitude (200.0, 90.0), floor, cap) == 100);
    /* A very wide sweep would exceed the regulatory ceiling -> clamped to cap. */
    REQUIRE (clamp_search_altitude (raw_search_altitude (1000.0, 90.0), floor, cap) == cap);
    /* A tiny sweep would sit at ground level -> clamped up to the floor. */
    REQUIRE (clamp_search_altitude (raw_search_altitude (2.0, 90.0), floor, cap) == floor);
}

TEST_CASE ("clamp_command_altitude converts feet to metres and clamps to [floor, cap]", "[altitude]")
{
    /* Defaults from the README: 122 m cap, 10 m floor. */
    constexpr uint16_t floor = 10;
    constexpr uint16_t cap = 122;

    /* 165 ft == 50.29 m, comfortably within range -> truncated to whole metres. */
    REQUIRE (clamp_command_altitude (165, floor, cap) == 50);
    /* 33 ft == 10.06 m, just above the floor -> kept (truncated to 10). */
    REQUIRE (clamp_command_altitude (33, floor, cap) == 10);

    /* An altitude command above the regulatory ceiling is pinned to the cap:
     * 500 ft == 152.4 m > 122 m. This is the safety gap the clamp closes -- a
     * direct operator altitude command must not exceed the ceiling that the
     * search and goto altitudes already honour. */
    REQUIRE (clamp_command_altitude (500, floor, cap) == cap);
    /* 1500 ft == 457 m, far above the cap. */
    REQUIRE (clamp_command_altitude (1500, floor, cap) == cap);

    /* A command below the floor is pinned up to it: 10 ft == 3.05 m < 10 m. */
    REQUIRE (clamp_command_altitude (10, floor, cap) == floor);
    /* Zero feet -> floor, never ground level. */
    REQUIRE (clamp_command_altitude (0, floor, cap) == floor);

    /* Exactly at the cap in feet: 400 ft == 121.92 m, within (122) -> 121. */
    REQUIRE (clamp_command_altitude (400, floor, cap) == 121);
}

TEST_CASE ("resolve_mav_mode returns no mode until the autopilot type is known", "[mav]")
{
    REQUIRE (!resolve_mav_mode (0, MavModeCommand::rtl).has_value ());
    REQUIRE (!resolve_mav_mode (0, MavModeCommand::manual).has_value ());
    REQUIRE (!resolve_mav_mode (0, MavModeCommand::hold).has_value ());
    REQUIRE (!resolve_mav_mode (0, MavModeCommand::auto_mode).has_value ());
}

TEST_CASE ("resolve_mav_mode treats zero-valued manual modes as valid", "[mav]")
{
    REQUIRE (resolve_mav_mode (MAV_TYPE_QUADROTOR, MavModeCommand::manual) == COPTER_MODE_STABILIZE);
    REQUIRE (resolve_mav_mode (MAV_TYPE_GROUND_ROVER, MavModeCommand::manual) == ROVER_MODE_MANUAL);
}

TEST_CASE ("resolve_mav_mode maps supported airframes to command modes", "[mav]")
{
    REQUIRE (resolve_mav_mode (MAV_TYPE_FIXED_WING, MavModeCommand::rtl) == PLANE_MODE_RTL);
    REQUIRE (resolve_mav_mode (MAV_TYPE_QUADROTOR, MavModeCommand::hold) == COPTER_MODE_POSHOLD);
    REQUIRE (resolve_mav_mode (MAV_TYPE_GROUND_ROVER, MavModeCommand::auto_mode) == ROVER_MODE_AUTO);
}

TEST_CASE ("mission_item_for lays out a goto mission", "[mission]")
{
    /* num_points is irrelevant in goto mode. */
    constexpr std::size_t any_points = 7;

    /* seq 0 and 1 are the goto target waypoint (sent twice). */
    REQUIRE (mission_item_for (0, any_points, MissionPlanMode::go_to).kind == MissionItemKind::goto_point);
    REQUIRE (mission_item_for (1, any_points, MissionPlanMode::go_to).kind == MissionItemKind::goto_point);
    /* Everything past it returns home. */
    REQUIRE (mission_item_for (2, any_points, MissionPlanMode::go_to).kind == MissionItemKind::rtl);
    REQUIRE (mission_item_for (99, any_points, MissionPlanMode::go_to).kind == MissionItemKind::rtl);
}

TEST_CASE ("mission_item_for lays out a search mission with the two-item offset", "[mission]")
{
    /* A 3-point search: setup items at seq 0/1, points at seq 2,3,4, RTL after. */
    constexpr std::size_t num_points = 3;

    REQUIRE (mission_item_for (0, num_points, MissionPlanMode::search).kind == MissionItemKind::takeoff);
    REQUIRE (mission_item_for (1, num_points, MissionPlanMode::search).kind == MissionItemKind::takeoff);

    /* seq 2..4 map to point indices 0..2 (the seq - 2 offset). */
    auto first = mission_item_for (2, num_points, MissionPlanMode::search);
    REQUIRE (first.kind == MissionItemKind::search_point);
    REQUIRE (first.point_index == 0);

    auto last = mission_item_for (4, num_points, MissionPlanMode::search);
    REQUIRE (last.kind == MissionItemKind::search_point);
    REQUIRE (last.point_index == num_points - 1);

    /* seq num_points + 1 (== 4) is still the last point; the first seq beyond it
     * terminates the mission with an RTL. */
    REQUIRE (mission_item_for (5, num_points, MissionPlanMode::search).kind == MissionItemKind::rtl);
    REQUIRE (mission_item_for (100, num_points, MissionPlanMode::search).kind == MissionItemKind::rtl);
}

TEST_CASE ("search_point_mission_seq offsets a search point index to its mission sequence", "[mission]")
{
    /* A resume must jump the autopilot to the mission sequence, which is offset
     * past the two setup/takeoff items: point 0 -> seq 2, point N -> seq N + 2. */
    REQUIRE (search_point_mission_seq (0) == 2);
    REQUIRE (search_point_mission_seq (1) == 3);
    REQUIRE (search_point_mission_seq (7) == 9);

    /* Round-trip invariant: the sequence for point i maps back to point index i
     * via mission_item_for (the inverse offset), so a resume lands on exactly the
     * intended search point rather than a setup item or the wrong waypoint. */
    constexpr std::size_t num_points = 10;
    for (int i = 0; i < static_cast<int> (num_points); i++)
    {
        auto item = mission_item_for (search_point_mission_seq (i), num_points, MissionPlanMode::search);
        REQUIRE (item.kind == MissionItemKind::search_point);
        REQUIRE (item.point_index == static_cast<std::size_t> (i));
    }

    /* A goto resume is unaffected: it always sets current to seq 0, which is the
     * goto waypoint (the search offset never applies to a goto upload). */
    REQUIRE (mission_item_for (0, num_points, MissionPlanMode::go_to).kind == MissionItemKind::goto_point);
}

TEST_CASE ("mission_item_for handles an empty search (no points)", "[mission]")
{
    /* With zero points, only the two takeoff items exist; seq 2 onward is RTL. */
    REQUIRE (mission_item_for (0, 0, MissionPlanMode::search).kind == MissionItemKind::takeoff);
    REQUIRE (mission_item_for (1, 0, MissionPlanMode::search).kind == MissionItemKind::takeoff);
    REQUIRE (mission_item_for (2, 0, MissionPlanMode::search).kind == MissionItemKind::rtl);
}

TEST_CASE ("mission_count_for advertises the RTL terminator slot", "[mission]")
{
    /* A goto mission is always 3 items regardless of the (irrelevant) point
     * count: seq 0/1 goto waypoint + seq 2 RTL. */
    REQUIRE (mission_count_for (0, MissionPlanMode::go_to) == 3);
    REQUIRE (mission_count_for (7, MissionPlanMode::go_to) == 3);

    /* A search of N points needs 2 setup items + N points + 1 RTL. */
    REQUIRE (mission_count_for (0, MissionPlanMode::search) == 3);
    REQUIRE (mission_count_for (3, MissionPlanMode::search) == 6);
}

TEST_CASE ("mission_count_for and mission_item_for agree on the RTL terminator", "[mission]")
{
    /* The count is what is sent in MISSION_COUNT; the FC then requests seq
     * 0 .. count-1. The last requested seq must resolve to the RTL item, and
     * the one before it must not, so the search always ends with exactly one
     * return-home item. This is the invariant todo/23 was about. */
    for (auto mode : { MissionPlanMode::search, MissionPlanMode::go_to })
    {
        for (std::size_t num_points : { std::size_t{ 0 }, std::size_t{ 1 }, std::size_t{ 3 }, std::size_t{ 50 } })
        {
            std::size_t count = mission_count_for (num_points, mode);
            uint16_t last_seq = static_cast<uint16_t> (count - 1);
            REQUIRE (mission_item_for (last_seq, num_points, mode).kind == MissionItemKind::rtl);
            REQUIRE (mission_item_for (static_cast<uint16_t> (last_seq - 1), num_points, mode).kind
                     != MissionItemKind::rtl);
        }
    }
}

TEST_CASE ("mav_systems creates a system once and finds it again", "[mav_sys]")
{
    mav_systems systems;

    /* The command paths must not conjure a system: before the recv thread has
     * seen a heartbeat for an id, the non-mutating lookup reports it absent. */
    REQUIRE (systems.findExistingSystem (1) == nullptr);

    /* findSystem is the recv-thread find-or-create; the first call makes it. */
    auto first = systems.findSystem (1);
    REQUIRE (first != nullptr);
    REQUIRE (first->getSysId () == 1);

    /* A second find-or-create for the same id returns the same object, not a
     * duplicate, so metadata written via one handle is visible through any. */
    REQUIRE (systems.findSystem (1) == first);

    /* And the command-path lookup now resolves to that same system. */
    REQUIRE (systems.findExistingSystem (1) == first);

    /* Distinct ids are distinct systems. */
    auto second = systems.findSystem (2);
    REQUIRE (second != first);
    REQUIRE (systems.findExistingSystem (2) == second);
}

TEST_CASE ("mav_sys metadata defaults to unknown and reflects updates", "[mav_sys]")
{
    /* A freshly created system has no heartbeat yet: the autopilot type reads
     * back as 0 (unknown), which is what makes the command paths fail safely
     * and no-op rather than command a wrong mode while the type is unknown. */
    mav_sys sys (7);
    REQUIRE (sys.getAutoPilotType () == 0);
    REQUIRE (sys.getFlightMode () == 0);
    REQUIRE_FALSE (sys.isSetup ());

    /* Once the heartbeat populates the metadata, the getters reflect it. */
    sys.setAutoPilotMode (MAV_TYPE_FIXED_WING);
    sys.setFlightMode (PLANE_MODE_RTL);
    sys.setupComplete ();
    REQUIRE (sys.getAutoPilotType () == MAV_TYPE_FIXED_WING);
    REQUIRE (sys.getFlightMode () == PLANE_MODE_RTL);
    REQUIRE (sys.isSetup ());
}

TEST_CASE ("mav_systems metadata access is race-free across recv and command threads", "[mav_sys][concurrency]")
{
    /* Regression guard for todo/25: the recv thread grows the systems list and
     * writes per-system metadata while the command threads read it. This drives
     * those paths concurrently so a reintroduced unsynchronised list mutation
     * or non-atomic scalar shows up as a crash here, and as a reported race
     * under -fsanitize=thread. */
    mav_systems systems;
    constexpr int iterations = 2000;
    std::atomic<bool> go{ false };

    /* Recv-thread role: find-or-create across the full id space (growing the
     * list) and keep rewriting system 1's metadata. */
    std::thread writer (
        [&] ()
        {
            while (!go.load ())
            {
            }
            for (int i = 0; i < iterations; i++)
            {
                systems.findSystem (static_cast<uint8_t> (i % 250 + 1));
                auto sys = systems.findSystem (1);
                sys->setAutoPilotMode (MAV_TYPE_FIXED_WING);
                sys->setFlightMode (PLANE_MODE_RTL);
            }
        });

    /* Command-thread role: non-mutating lookups plus metadata reads, exactly as
     * commandRTL/Hold/Manual/Auto do. */
    auto reader = [&] ()
    {
        while (!go.load ())
        {
        }
        for (int i = 0; i < iterations; i++)
        {
            auto sys = systems.findExistingSystem (1);
            if (sys != nullptr)
            {
                (void)sys->getAutoPilotType ();
                (void)sys->getFlightMode ();
            }
            (void)systems.findExistingSystem (static_cast<uint8_t> (i % 250 + 1));
        }
    };
    std::thread reader_one (reader);
    std::thread reader_two (reader);

    go.store (true);
    writer.join ();
    reader_one.join ();
    reader_two.join ();

    /* The writer always ends having created system 1 with a known type. */
    auto sys = systems.findExistingSystem (1);
    REQUIRE (sys != nullptr);
    REQUIRE (sys->getAutoPilotType () == MAV_TYPE_FIXED_WING);
}

TEST_CASE ("search_acquire_action never fetches without an asset", "[smm][acquire]")
{
    constexpr uint64_t now = 1000;

    /* While the retry timer is in the future, back off regardless of asset
     * state (the timer check comes first). */
    REQUIRE (search_acquire_action (true, now + 1, now) == SearchAcquireAction::backoff);
    REQUIRE (search_acquire_action (false, now + 1, now) == SearchAcquireAction::backoff);

    /* Timer elapsed but no asset (disconnected / discovery failed): RTL and
     * retry later, never call into the SMM C library. */
    REQUIRE (search_acquire_action (false, now, now) == SearchAcquireAction::disconnected_rtl);
    REQUIRE (search_acquire_action (false, 0, now) == SearchAcquireAction::disconnected_rtl);

    /* Timer elapsed and asset present: safe to fetch. */
    REQUIRE (search_acquire_action (true, now, now) == SearchAcquireAction::fetch);
    REQUIRE (search_acquire_action (true, 0, now) == SearchAcquireAction::fetch);
}

TEST_CASE ("smm_connection_is_connected rejects null before asking the C library for state", "[smm]")
{
    REQUIRE (!smm_connection_is_connected (nullptr));
}

TEST_CASE ("mav_comms_is_up requires an open socket and a recent heartbeat", "[mav][comms]")
{
    constexpr uint64_t timeout = 5000;
    constexpr uint64_t now = 100000;

    /* A closed socket is always down — even if a heartbeat timestamp looks
     * recent. This is the cold-start case: never connected, so no heartbeat can
     * time out and the link must read down rather than be assumed healthy. */
    REQUIRE_FALSE (mav_comms_is_up (false, now, now, timeout));
    REQUIRE_FALSE (mav_comms_is_up (false, now, 0, timeout));

    /* Open socket, heartbeat within the window (including the exact boundary). */
    REQUIRE (mav_comms_is_up (true, now, now, timeout));           /* age 0 */
    REQUIRE (mav_comms_is_up (true, now, now - 2000, timeout));    /* age 2000 */
    REQUIRE (mav_comms_is_up (true, now, now - timeout, timeout)); /* age == timeout */

    /* Open socket, heartbeat too old -> down. */
    REQUIRE_FALSE (mav_comms_is_up (true, now, now - (timeout + 1), timeout));

    /* Open socket, no heartbeat yet (ts 0, far in the past) -> down. A freshly
     * connected socket stays down until the autopilot is actually heard from. */
    REQUIRE_FALSE (mav_comms_is_up (true, now, 0, timeout));

    /* A clock anomaly (last heartbeat timestamped after now) must not wrap the
     * unsigned subtraction into a huge age and spuriously fail; treat it as up. */
    REQUIRE (mav_comms_is_up (true, now, now + 1000, timeout));
}

TEST_CASE ("known_aircraft assigns and retrieves consistent ICAO address", "[aircraft]")
{
    known_aircraft ka;
    std::string callsign = "TEST123";

    uint32_t icao1 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 >= first_icao_address_for_unknown_aircraft);

    uint32_t icao2 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 == icao2);

    PositionData pd;
    pd = PositionData (0, 0, 0, 0, 0, 0, callsign, 0, 0, 1000, 0, 0, 0);
    ka.newPositionReport (pd);

    uint32_t icao3 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 == icao3);
}

TEST_CASE ("known_aircraft evicts aircraft that go quiet past the eviction window", "[aircraft]")
{
    known_aircraft ka;
    const uint64_t window_ms = static_cast<uint64_t> (aircraft_eviction_age.count ());

    /* Establish aircraft A at t=1000 and capture its synthetic ICAO. */
    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "A", 0, 0, 1000, 0, 0, 0));
    const uint32_t icao_a_first = ka.getAircraftICAOAddress ("A");

    /* A different aircraft reports well past the eviction window. That report
     * drives the sweep, which must reclaim the now-stale entry for A. */
    const uint64_t late = 1000 + window_ms + 1000;
    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "B", 0, 0, late, 0, 0, 0));

    /* Re-introducing the previously-seen callsign A must allocate a fresh
     * synthetic ICAO, proving the old entry was evicted rather than retained. */
    const uint32_t icao_a_second = ka.getAircraftICAOAddress ("A");
    REQUIRE (icao_a_second != icao_a_first);
}

TEST_CASE ("known_aircraft keeps actively-reporting aircraft across the window", "[aircraft]")
{
    known_aircraft ka;
    const uint64_t window_ms = static_cast<uint64_t> (aircraft_eviction_age.count ());

    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "C", 0, 0, 1000, 0, 0, 0));
    const uint32_t icao_c_first = ka.getAircraftICAOAddress ("C");

    /* C keeps reporting; each report refreshes its timestamp, so it must never
     * be evicted and must retain its original synthetic ICAO. */
    for (uint64_t t = 1000 + window_ms; t <= 1000 + (3 * window_ms); t += window_ms)
    {
        ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "C", 0, 0, t, 0, 0, 0));
    }

    REQUIRE (ka.getAircraftICAOAddress ("C") == icao_c_first);
}

TEST_CASE ("next_synthetic_icao advances and wraps at the 24-bit ceiling", "[aircraft]")
{
    /* Normal case: advance by one within the range. */
    REQUIRE (next_synthetic_icao (first_icao_address_for_unknown_aircraft)
             == first_icao_address_for_unknown_aircraft + 1);
    REQUIRE (next_synthetic_icao (0x1234) == 0x1235);

    /* At (or above) the 24-bit ceiling, wrap back to the start of the range
     * rather than emitting a value that does not fit the 24-bit wire field. */
    REQUIRE (next_synthetic_icao (last_icao_address_for_unknown_aircraft) == first_icao_address_for_unknown_aircraft);
    REQUIRE (next_synthetic_icao (last_icao_address_for_unknown_aircraft - 1)
             == last_icao_address_for_unknown_aircraft);

    /* Every produced address stays inside the valid 24-bit range. */
    REQUIRE (next_synthetic_icao (last_icao_address_for_unknown_aircraft) <= last_icao_address_for_unknown_aircraft);
}

/* loadFmuConfig reads a file path, so the tests write JSON to a temp file. This
 * RAII wrapper writes it on construction and unlinks on destruction, so a failed
 * REQUIRE mid-test cannot leave a stray file behind. The directory comes from
 * std::filesystem rather than a hard-coded /tmp; mkstemp keeps the name unique
 * without the linker warning std::tmpnam draws. */
namespace
{
class TempConfigFile
{
  public:
    explicit TempConfigFile (const std::string &contents) : path{}
    {
        std::string tmpl = (std::filesystem::temp_directory_path () / "cap-fmu-config-test-XXXXXX").string ();
        std::vector<char> buf (tmpl.begin (), tmpl.end ());
        buf.push_back ('\0');
        int fd = mkstemp (buf.data ());
        REQUIRE (fd >= 0);
        path = buf.data ();
        ssize_t written = write (fd, contents.data (), contents.size ());
        close (fd);
        REQUIRE (written == static_cast<ssize_t> (contents.size ()));
    }
    TempConfigFile (const TempConfigFile &) = delete;
    TempConfigFile (TempConfigFile &&) = delete;
    auto operator= (const TempConfigFile &) -> TempConfigFile & = delete;
    auto operator= (TempConfigFile &&) -> TempConfigFile & = delete;
    ~TempConfigFile ()
    {
        std::error_code ec;
        std::filesystem::remove (path, ec);
    }
    auto
    str () const -> const std::string &
    {
        return path;
    }

  private:
    std::string path;
};

auto
load_config (const std::string &contents) -> FmuConfig
{
    TempConfigFile tf (contents);
    return loadFmuConfig (tf.str ());
}

/* A default-constructed config, shared by the tests that assert values fell
 * back to their defaults. */
const FmuConfig def{};
} // namespace

TEST_CASE ("loadFmuConfig returns defaults when the fmu block is absent", "[config]")
{
    FmuConfig cfg = load_config (R"({ "name": "test" })");
    REQUIRE (cfg.altitude_cap_m == def.altitude_cap_m);
    REQUIRE (cfg.altitude_floor_m == def.altitude_floor_m);
    REQUIRE (cfg.goto_altitude_m == def.goto_altitude_m);
    REQUIRE (cfg.camera_fov_deg == Catch::Approx (def.camera_fov_deg));
    REQUIRE (cfg.lowbat_threshold == def.lowbat_threshold);
    REQUIRE (cfg.reconnect_interval_s == def.reconnect_interval_s);
    REQUIRE (cfg.log_level == def.log_level);
    REQUIRE (cfg.log_dir == def.log_dir);
}

TEST_CASE ("loadFmuConfig reads valid fmu values", "[config]")
{
    FmuConfig cfg = load_config (R"({
        "fmu": {
            "altitude_cap_m": 100,
            "altitude_floor_m": 20,
            "goto_altitude_m": 60,
            "camera_fov_deg": 60.0,
            "lowbat_threshold": 25,
            "reconnect_interval_s": 30,
            "log_level": "debug"
        }
    })");
    REQUIRE (cfg.altitude_cap_m == 100);
    REQUIRE (cfg.altitude_floor_m == 20);
    REQUIRE (cfg.goto_altitude_m == 60);
    REQUIRE (cfg.camera_fov_deg == Catch::Approx (60.0));
    REQUIRE (cfg.lowbat_threshold == 25);
    REQUIRE (cfg.reconnect_interval_s == 30);
    REQUIRE (cfg.log_level == LogLevel::debug);
}

TEST_CASE ("loadFmuConfig reads a custom log_dir and rejects bad ones", "[config]")
{
    FmuConfig cfg = load_config (R"({ "fmu": { "log_dir": "/tmp/cap-fmu-logs" } })");
    REQUIRE (cfg.log_dir == "/tmp/cap-fmu-logs");

    /* An empty or non-string log_dir keeps the default. */
    FmuConfig empty = load_config (R"({ "fmu": { "log_dir": "" } })");
    REQUIRE (empty.log_dir == def.log_dir);
    FmuConfig wrong_type = load_config (R"({ "fmu": { "log_dir": 42 } })");
    REQUIRE (wrong_type.log_dir == def.log_dir);
}

TEST_CASE ("loadFmuConfig clamps the goto altitude into [floor, cap]", "[config]")
{
    /* Above the cap clamps down to the cap. */
    FmuConfig high = load_config (R"({ "fmu": { "altitude_cap_m": 100, "goto_altitude_m": 250 } })");
    REQUIRE (high.goto_altitude_m == 100);

    /* Below the floor clamps up to the floor. */
    FmuConfig low = load_config (R"({ "fmu": { "altitude_floor_m": 40, "goto_altitude_m": 10 } })");
    REQUIRE (low.goto_altitude_m == 40);

    /* Feet are converted like the other altitudes: 200 ft -> 61 m, within the
     * default [10, 122] range, so it is kept as-is. */
    FmuConfig ft = load_config (R"({ "fmu": { "goto_altitude_ft": 200 } })");
    REQUIRE (ft.goto_altitude_m == 61);
}

TEST_CASE ("loadFmuConfig converts feet to metres and prefers feet over metres", "[config]")
{
    /* 400 ft -> lround(400 * 0.3048) == 122 m. */
    FmuConfig ft = load_config (R"({ "fmu": { "altitude_cap_ft": 400 } })");
    REQUIRE (ft.altitude_cap_m == 122);

    /* When both _m and _ft are given, feet wins. */
    FmuConfig both = load_config (R"({ "fmu": { "altitude_cap_m": 80, "altitude_cap_ft": 400 } })");
    REQUIRE (both.altitude_cap_m == 122);
}

TEST_CASE ("loadFmuConfig clamps an altitude floor above the cap down to the cap", "[config]")
{
    FmuConfig cfg = load_config (R"({ "fmu": { "altitude_cap_m": 50, "altitude_floor_m": 80 } })");
    REQUIRE (cfg.altitude_cap_m == 50);
    REQUIRE (cfg.altitude_floor_m == 50);
}

TEST_CASE ("loadFmuConfig rejects out-of-range values and keeps defaults", "[config]")
{
    /* FoV outside (0, 180), battery outside [0, 100], interval outside
     * [1, 3600], and an unknown log level each fall back to their default. */
    FmuConfig cfg = load_config (R"({
        "fmu": {
            "camera_fov_deg": 200.0,
            "lowbat_threshold": 150,
            "reconnect_interval_s": 0,
            "log_level": "verbose"
        }
    })");
    REQUIRE (cfg.camera_fov_deg == Catch::Approx (def.camera_fov_deg));
    REQUIRE (cfg.lowbat_threshold == def.lowbat_threshold);
    REQUIRE (cfg.reconnect_interval_s == def.reconnect_interval_s);
    REQUIRE (cfg.log_level == def.log_level);
}

TEST_CASE ("loadFmuConfig falls back to defaults on malformed JSON", "[config]")
{
    FmuConfig cfg = load_config ("{ this is not valid json ");
    REQUIRE (cfg.altitude_cap_m == def.altitude_cap_m);
    REQUIRE (cfg.lowbat_threshold == def.lowbat_threshold);
}

TEST_CASE ("loadFmuConfig falls back to defaults when the file is missing", "[config]")
{
    FmuConfig cfg = loadFmuConfig ("/nonexistent/cap-fmu-no-such-config.json");
    REQUIRE (cfg.altitude_cap_m == def.altitude_cap_m);
    REQUIRE (cfg.log_level == def.log_level);
}
