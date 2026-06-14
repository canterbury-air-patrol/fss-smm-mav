#include <catch2/catch_test_macros.hpp>

#include "aircraft.hpp"
#include "fmu.hpp"

#include <memory>
#include <tuple>

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

TEST_CASE ("state_change_cb fires on every state transition", "[state_machine]")
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
