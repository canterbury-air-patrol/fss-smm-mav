#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "aircraft.hpp"
#include "fmu-config.hpp"
#include "fmu.hpp"
#include "fss/command-ack-group.hpp"
#include "fss/command-ack.hpp"
#include "mav/mission-plan.hpp"
#include "smm/search-altitude.hpp"

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <tuple>
#include <unistd.h>
#include <vector>

class MockMAV : public IMAV
{
  public:
    flight_mode last_mode{ flight_mode_unknown };
    int set_mode_calls{ 0 };
    bool disarmed{ false };
    bool terminated{ false };
    uint16_t last_altitude{ 0 };
    int set_altitude_calls{ 0 };

    MockMAV () = default;
    MockMAV (const MockMAV &) = delete;
    MockMAV (MockMAV &&) = delete;
    auto operator= (const MockMAV &) -> MockMAV & = delete;
    auto operator= (MockMAV &&) -> MockMAV & = delete;
    ~MockMAV () override = default;

    void
    setMode (flight_mode fm) override
    {
        last_mode = fm;
        set_mode_calls++;
    }
    void
    disarm () override
    {
        disarmed = true;
    }
    void
    terminate () override
    {
        terminated = true;
    }
    void
    gotoPosition (Point) override
    {
    }
    void
    setAltitude (uint16_t alt) override
    {
        last_altitude = alt;
        set_altitude_calls++;
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

TEST_CASE ("mission_item_for lays out a goto mission", "[mission]")
{
    /* num_points is irrelevant in goto mode. */
    constexpr std::size_t any_points = 7;

    /* seq 0 and 1 are the goto target waypoint (sent twice). */
    REQUIRE (mission_item_for (0, any_points, true).kind == MissionItemKind::goto_point);
    REQUIRE (mission_item_for (1, any_points, true).kind == MissionItemKind::goto_point);
    /* Everything past it returns home. */
    REQUIRE (mission_item_for (2, any_points, true).kind == MissionItemKind::rtl);
    REQUIRE (mission_item_for (99, any_points, true).kind == MissionItemKind::rtl);
}

TEST_CASE ("mission_item_for lays out a search mission with the two-item offset", "[mission]")
{
    /* A 3-point search: setup items at seq 0/1, points at seq 2,3,4, RTL after. */
    constexpr std::size_t num_points = 3;

    REQUIRE (mission_item_for (0, num_points, false).kind == MissionItemKind::takeoff);
    REQUIRE (mission_item_for (1, num_points, false).kind == MissionItemKind::takeoff);

    /* seq 2..4 map to point indices 0..2 (the seq - 2 offset). */
    auto first = mission_item_for (2, num_points, false);
    REQUIRE (first.kind == MissionItemKind::search_point);
    REQUIRE (first.point_index == 0);

    auto last = mission_item_for (4, num_points, false);
    REQUIRE (last.kind == MissionItemKind::search_point);
    REQUIRE (last.point_index == num_points - 1);

    /* seq num_points + 1 (== 4) is still the last point; the first seq beyond it
     * terminates the mission with an RTL. */
    REQUIRE (mission_item_for (5, num_points, false).kind == MissionItemKind::rtl);
    REQUIRE (mission_item_for (100, num_points, false).kind == MissionItemKind::rtl);
}

TEST_CASE ("mission_item_for handles an empty search (no points)", "[mission]")
{
    /* With zero points, only the two takeoff items exist; seq 2 onward is RTL. */
    REQUIRE (mission_item_for (0, 0, false).kind == MissionItemKind::takeoff);
    REQUIRE (mission_item_for (1, 0, false).kind == MissionItemKind::takeoff);
    REQUIRE (mission_item_for (2, 0, false).kind == MissionItemKind::rtl);
}

TEST_CASE ("known_aircraft assigns and retrieves consistent ICAO address", "[aircraft]")
{
    known_aircraft ka;
    std::string callsign = "TEST123";

    uint32_t icao1 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 >= known_aircraft::first_icao_address_for_unknown_aircraft);

    uint32_t icao2 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 == icao2);

    PositionData pd;
    pd = PositionData (0, 0, 0, 0, 0, 0, callsign, 0, 0, 1000, 0, 0, 0);
    ka.newPositionReport (pd);

    uint32_t icao3 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 == icao3);
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
    const FmuConfig def{};
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
    const FmuConfig def{};
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
    const FmuConfig def{};
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
    const FmuConfig def{};
    FmuConfig cfg = load_config ("{ this is not valid json ");
    REQUIRE (cfg.altitude_cap_m == def.altitude_cap_m);
    REQUIRE (cfg.lowbat_threshold == def.lowbat_threshold);
}

TEST_CASE ("loadFmuConfig falls back to defaults when the file is missing", "[config]")
{
    const FmuConfig def{};
    FmuConfig cfg = loadFmuConfig ("/nonexistent/cap-fmu-no-such-config.json");
    REQUIRE (cfg.altitude_cap_m == def.altitude_cap_m);
    REQUIRE (cfg.log_level == def.log_level);
}
