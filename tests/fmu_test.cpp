#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "aircraft.hpp"
#include "altitude-cap.hpp"
#include "altitude-units.hpp"
#include "event-dispatcher.hpp"
#include "fmu-config.hpp"
#include "fmu.hpp"
#include "fss/command-ack-group.hpp"
#include "fss/command-ack.hpp"
#include "fss/ifss.hpp"
#include "log-rotation.hpp"
#include "logger.hpp"
#include "mav/battery-voltage.hpp"
#include "mav/failsafe-params.hpp"
#include "mav/internal.hpp"
#include "mav/latlon-encoding.hpp"
#include "mav/mav-comms.hpp"
#include "mav/mission-plan.hpp"
#include "mav/mode-resolve.hpp"
#include "mav/velocity.hpp"
#include "smm/connection-state.hpp"
#include "smm/search-acquire.hpp"
#include "smm/search-altitude.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>

/* Discards everything; used where a test needs an ILogger but is not
 * itself testing logging output (todo/59). */
class NullLogger : public ILogger
{
  public:
    void
    log (LogLevel, std::string_view) override
    {
    }
};

class MockMAV : public IMAV
{
  public:
    flight_mode last_mode{ flight_mode_unknown };
    int set_mode_calls{ 0 };
    bool disarmed{ false };
    int disarm_calls{ 0 };
    bool terminated{ false };
    int terminate_calls{ 0 };
    uint32_t last_altitude{ 0 };
    int set_altitude_calls{ 0 };
    Point last_goto{};
    int goto_calls{ 0 };
    /* Controls the transmission result the action methods report. Set false to
     * simulate a send that did not reach the autopilot (MAV link down) so the
     * re-apply-on-recovery path (todo/46, todo/108) can be exercised. */
    bool send_succeeds{ true };
    std::shared_ptr<SMMSearch> last_loaded_search{ nullptr };
    int load_search_calls{ 0 };
    PositionData last_adsb{};
    int send_adsb_calls{ 0 };

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
    gotoPosition (Point p) override
    {
        last_goto = p;
        goto_calls++;
    }
    auto
    setAltitude (uint32_t alt) -> bool override
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
    void
    registerAutopilotRestartCB (notify_autopilot_restart_cb) override
    {
    }
    void
    loadSearch (const std::shared_ptr<SMMSearch> &search) override
    {
        last_loaded_search = search;
        load_search_calls++;
    }
    void
    sendADSB (PositionData pd) override
    {
        last_adsb = pd;
        send_adsb_calls++;
    }
};

class MockSMM : public ISMM
{
  public:
    int search_calls{ 0 };
    int cancel_calls{ 0 };
    int connect_calls{ 0 };
    std::string last_connect_host{};
    std::string last_connect_asset_name{};
    PositionData last_reported_position{};
    int report_position_calls{ 0 };
    int last_reached_point{ 0 };
    int reached_point_calls{ 0 };
    int current_search_points{ 0 };

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
    void
    connect (const std::string &host, const flight_safety_system::secure_string &,
             const flight_safety_system::secure_string &, const std::string &asset_name) override
    {
        last_connect_host = host;
        last_connect_asset_name = asset_name;
        connect_calls++;
    }
    void
    reportPosition (PositionData pd) override
    {
        last_reported_position = pd;
        report_position_calls++;
    }
    void
    reachedPoint (int point) override
    {
        last_reached_point = point;
        reached_point_calls++;
    }
    auto
    currentSearchPoints () -> int override
    {
        return current_search_points;
    }
};

class MockFSSReporter : public IFSSReporter
{
  public:
    PositionData last_reported_position{};
    int report_position_calls{ 0 };
    int last_reached_point{ 0 };
    int last_reached_total{ 0 };
    int reached_point_calls{ 0 };
    BatteryData last_battery_status{};
    int report_battery_status_calls{ 0 };
    int post_ack_calls{ 0 };

    MockFSSReporter () = default;
    MockFSSReporter (const MockFSSReporter &) = delete;
    MockFSSReporter (MockFSSReporter &&) = delete;
    auto operator= (const MockFSSReporter &) -> MockFSSReporter & = delete;
    auto operator= (MockFSSReporter &&) -> MockFSSReporter & = delete;
    ~MockFSSReporter () override = default;

    void
    reportPosition (PositionData pd) override
    {
        last_reported_position = pd;
        report_position_calls++;
    }
    void
    reachedPoint (int point, int total_points) override
    {
        last_reached_point = point;
        last_reached_total = total_points;
        reached_point_calls++;
    }
    void
    reportBatteryStatus (BatteryData bd) override
    {
        last_battery_status = bd;
        report_battery_status_calls++;
    }
    void
    postAck (const fss_command_ack_responder &, const FSSCommandResolution &) override
    {
        post_ack_calls++;
    }
};

using SM = std::tuple<std::shared_ptr<MockMAV>, std::shared_ptr<MockSMM>, std::shared_ptr<FMUStateMachine>>;

static auto
make_sm (int low_battery_latch_count = FMUStateMachine::default_low_battery_latch_count,
         uint16_t altitude_cap_m = FMUStateMachine::default_altitude_cap_m,
         int altitude_breach_latch_count = FMUStateMachine::default_altitude_breach_latch_count) -> SM
{
    auto mav = std::make_shared<MockMAV> ();
    auto smm = std::make_shared<MockSMM> ();
    auto sm = std::make_shared<FMUStateMachine> (*mav, *smm, low_battery_latch_count, altitude_cap_m,
                                                 altitude_breach_latch_count);
    return { mav, smm, sm };
}

/* The state machine debounces low-battery readings: the RTL latch only engages
 * after low_battery_latch_count consecutive low samples. Drive exactly that many
 * so tests that assume a latched low battery stay correct if the count changes. */
static void
latch_low_battery (const std::shared_ptr<FMUStateMachine> &sm)
{
    for (int i = 0; i < FMUStateMachine::default_low_battery_latch_count; i++)
    {
        sm->setLowBattery (true);
    }
}

/* Mirrors latch_low_battery: drives exactly default_altitude_breach_latch_count
 * over-cap, valid-fix readings so tests that assume an engaged altitude
 * breach stay correct if the count changes. 200m is comfortably over the
 * default 122m cap. */
static void
latch_altitude_breach (const std::shared_ptr<FMUStateMachine> &sm, double over_cap_altitude_agl_m = 200.0)
{
    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count; i++)
    {
        sm->setCurrentAltitude (true, over_cap_altitude_agl_m);
    }
}

TEST_CASE ("low battery latches RTL regardless of subsequent FSS commands", "[state_machine][TC-FS-011][TC-FS-012]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_goto);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_manual);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("comms failure latches failsafe until comms restored", "[state_machine][TC-MAV-004][TC-MAV-005][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->setCommsFailure (false);
    REQUIRE (mav->last_mode == flight_mode_hold);
}

TEST_CASE ("a low-battery RTL that failed to send is replayed when MAV comms recover (todo/46)",
           "[state_machine][replay][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

    /* MAV link is down: action sends do not reach the autopilot. */
    mav->send_succeeds = false;
    sm->setMavCommsFailure (true);
    latch_low_battery (sm);
    /* The low-battery latch (higher priority than comms) holds the state at RTL,
     * but the command never made it onto the wire. */
    REQUIRE (mav->last_mode == flight_mode_rtl);
    const int calls_before = mav->set_mode_calls;

    /* The link recovers. The latch keeps current_state unchanged, so there is no
     * transition to re-drive the action — without replay the RTL would never be
     * re-sent. It must be replayed now. */
    mav->send_succeeds = true;
    sm->setMavCommsFailure (false);
    REQUIRE (mav->set_mode_calls == calls_before + 1);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("a waiting_for_tasking RTL that failed to send is replayed once MAV comms report okay again (todo/46/70)",
           "[state_machine][replay]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    mav->send_succeeds = false;
    sm->SMMNewCommand (smm_cmd_mission_complete);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    const int calls_before = mav->set_mode_calls;

    /* Unlike low_battery/terminate, waiting_for_tasking is not a latch, so a
     * genuine comms down->up cycle re-drives the send via the ordinary
     * transition path (out to failsafe and back) rather than the no-transition
     * re-apply. Either way the RTL that never reached the autopilot must be on
     * the wire once the link is back. */
    mav->send_succeeds = true;
    sm->setMavCommsFailure (true);
    sm->setMavCommsFailure (false);
    REQUIRE (mav->set_mode_calls > calls_before);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("a terminate that failed to send is replayed when MAV comms recover (todo/46)",
           "[state_machine][replay][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

    mav->send_succeeds = false;
    sm->setMavCommsFailure (true);
    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
    const int terminate_before = mav->terminate_calls;

    /* terminate stays the current state on recovery (it is the top priority and
     * still commanded), so updateState reports no transition; the failed send must
     * still be re-issued. Policy: re-send terminate on recovery. */
    mav->send_succeeds = true;
    sm->setMavCommsFailure (false);
    REQUIRE (mav->terminate_calls == terminate_before + 1);
}

TEST_CASE ("a safety action that was transmitted is re-applied on a genuine MAV recovery (todo/108)",
           "[state_machine][replay][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

    /* send_succeeds stays true: the RTL reaches the autopilot first time. */
    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    const int calls_before = mav->set_mode_calls;

    /* The link goes away and comes back. The low-battery latch outranks the
     * comms failsafe, so current_state never moves and there is no transition to
     * re-drive -- and under todo/46's old rule nothing was re-sent, because the
     * original send had succeeded. But this end cannot tell a link drop from an
     * autopilot reboot, and a rebooted autopilot has forgotten the RTL entirely.
     * So the current state is re-commanded regardless (todo/108). */
    sm->setMavCommsFailure (true);
    sm->setMavCommsFailure (false);
    REQUIRE (mav->set_mode_calls == calls_before + 1);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("a redundant comms-okay report with no preceding loss re-sends nothing (todo/108)",
           "[state_machine][replay]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    const int calls_before = mav->set_mode_calls;

    /* Only a genuine down->up edge means the autopilot may have restarted. The
     * heartbeat loop only ever reports edges, so this cannot happen in
     * production -- but the re-apply must be gated on the edge, not on
     * !failed, or any repeated health report would re-command the autopilot. */
    sm->setMavCommsFailure (false);
    REQUIRE (mav->set_mode_calls == calls_before);
}

TEST_CASE ("fss_cmd_continue with smm_cmd_none leads to searching", "[state_machine]")
{
    auto [mav, smm, sm] = make_sm ();
    int calls_before = smm->search_calls;

    sm->FSSNewCommand (fss_cmd_continue);

    REQUIRE (smm->search_calls > calls_before);
}

TEST_CASE ("fss_cmd_continue with smm_cmd_abandon_search leads to searching", "[state_machine]")
{
    auto [mav, smm, sm] = make_sm ();

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
    auto [mav, smm, sm] = make_sm ();

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
TEST_CASE ("smm_cmd_mission_complete overrides an active search with RTL, without cancelling the search role (todo/70)",
           "[state_machine]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    /* SMM reports the search done: the FMU leaves searching for
     * waiting_for_tasking, the same RTL flight mode as a real RTL. */
    sm->SMMNewCommand (smm_cmd_mission_complete);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    /* Unlike a real RTL, this must NOT cancel the SMM searching role: the
     * acquire-retry loop must keep running so a freshly reacquired search
     * auto-engages with no operator action (todo/77). Contrast with "low
     * battery forces RTL out of an active search" below. */
    REQUIRE (smm->cancel_calls == 0);
}

TEST_CASE ("an explicit FSS command overrides an active SMM search", "[state_machine]")
{
    auto [mav, smm, sm] = make_sm ();

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
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    /* Contrast with "smm_cmd_mission_complete ... without cancelling the
     * search role" above: a genuine latch-driven RTL DOES cancel the SMM
     * searching role. */
    REQUIRE (smm->cancel_calls >= 1);
}

TEST_CASE ("comms failure forces RTL out of an active search", "[state_machine][TC-MAV-004][TC-MAV-005]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* Design decision: terminate has the highest priority and overrides both
 * low_battery and comms_failure.  The ground station must always be able to
 * halt the aircraft, even during an emergency RTL. */
TEST_CASE ("terminate overrides low battery", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (!mav->terminated);

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
}

TEST_CASE ("terminate overrides comms failure", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (!mav->terminated);

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
}

/* Design decision: low_battery outranks comms_failure.  Both drive RTL, but
 * low_battery latches, so clearing comms (or sending FSS commands) must not
 * release the aircraft from RTL while the battery is still low. */
TEST_CASE ("low battery outranks comms failure", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

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
TEST_CASE ("manual blocked when low battery is set", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    sm->FSSNewCommand (fss_cmd_manual);

    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("manual allowed when battery OK", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

    /* The machine starts in fmu_state_manual, so move away first to make the
     * transition back to manual observable. */
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->FSSNewCommand (fss_cmd_manual);
    REQUIRE (mav->last_mode == flight_mode_manual);
}

TEST_CASE ("disarm allowed when battery OK", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_disarm);

    REQUIRE (mav->disarmed);
}

TEST_CASE ("disarm blocked when low battery is set", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    sm->FSSNewCommand (fss_cmd_disarm);

    REQUIRE (!mav->disarmed);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("same non-searching state does not re-action", "[state_machine][TC-MAV-013]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    int calls = mav->set_mode_calls;

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->set_mode_calls == calls);
}

TEST_CASE ("searching does not re-action on repeated state update", "[state_machine][TC-MAV-013]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    int calls = smm->search_calls;

    sm->SMMNewCommand (smm_cmd_none);
    REQUIRE (smm->search_calls == calls);
}

TEST_CASE ("goto does not re-action on repeated FSS goto", "[state_machine][TC-MAV-013]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_goto);
    int calls = mav->set_mode_calls;

    sm->FSSNewCommand (fss_cmd_goto);
    REQUIRE (mav->set_mode_calls == calls);
}

TEST_CASE ("goto command passes its carried target to mav gotoPosition", "[state_machine][TC-MAV-018]")
{
    auto [mav, smm, sm] = make_sm ();

    /* The target travels inside the command (todo/53), not via an FSS
     * side-channel; actionState must goto exactly the point the command carried. */
    sm->FSSNewCommand (fss_cmd_goto, FSSCommandTarget{ Point{ -43.5, 172.6 }, 0 });

    REQUIRE (mav->goto_calls == 1);
    REQUIRE (mav->last_goto == Point{ -43.5, 172.6 });
    REQUIRE (mav->last_mode == flight_mode_goto);
}

/* The carried goto target is retained across a higher-priority latch: a comms
 * failsafe takes over goto, and when comms recover the goto is re-applied with
 * the SAME target the original command carried (fss_command and its target are
 * written together and only a new FSS command overwrites them). */
TEST_CASE ("goto target survives a comms-loss latch and is re-applied on recovery",
           "[state_machine][TC-MAV-018][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_goto, FSSCommandTarget{ Point{ -43.5, 172.6 }, 0 });
    REQUIRE (mav->last_goto == Point{ -43.5, 172.6 });

    /* Comms drop: the failsafe RTL supersedes the goto. */
    sm->setCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    /* Comms restored: goto is re-entered and the original target flown again,
     * even though no new goto command arrived to re-supply it. */
    sm->setCommsFailure (false);
    REQUIRE (mav->last_mode == flight_mode_goto);
    REQUIRE (mav->goto_calls == 2);
    REQUIRE (mav->last_goto == Point{ -43.5, 172.6 });
}

TEST_CASE ("altitude adjust command calls mav setAltitude", "[state_machine]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_altitude, FSSCommandTarget{ Point{}, 150 });

    REQUIRE (mav->last_altitude == 150);
}

TEST_CASE ("altitude adjust carries a full-width altitude through to setAltitude", "[state_machine][TC-MAV-018]")
{
    auto [mav, smm, sm] = make_sm ();

    /* An altitude that does not fit in 16 bits must reach the MAV layer intact so
     * the [floor, cap] clamp there is the only thing that narrows it. If the
     * target were still uint16_t this would wrap (70000 & 0xFFFF == 4464) and the
     * state machine would command a low altitude instead (todo/55). */
    constexpr uint32_t oversized_ft = 70000;
    sm->FSSNewCommand (fss_cmd_altitude, FSSCommandTarget{ Point{}, oversized_ft });

    REQUIRE (mav->last_altitude == oversized_ft);
}

TEST_CASE ("altitude adjust does not re-action on repeated FSS altitude", "[state_machine][TC-MAV-013]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_altitude, FSSCommandTarget{ Point{}, 150 });
    int calls = mav->set_altitude_calls;

    sm->FSSNewCommand (fss_cmd_altitude, FSSCommandTarget{ Point{}, 150 });
    REQUIRE (mav->set_altitude_calls == calls);
}

/* Design decision: the low-battery RTL latch is debounced. A single noisy or
 * spurious low reading must not ground the mission; the latch only engages
 * after low_battery_latch_count consecutive low samples. */
TEST_CASE ("low battery does not latch before the debounce count", "[state_machine][TC-FS-011][TC-FS-012]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    /* One short of the count is not enough to latch. */
    for (int i = 0; i < FMUStateMachine::default_low_battery_latch_count - 1; i++)
    {
        sm->setLowBattery (true);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);

    /* The final consecutive reading trips the latch. */
    sm->setLowBattery (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* low_battery_latch_count is configurable (todo/54): a non-default count must
 * actually change the debounce, not just be accepted and ignored. */
TEST_CASE ("a configured low_battery_latch_count changes the debounce", "[state_machine][TC-FS-011][TC-FS-012]")
{
    auto [mav, smm, sm] = make_sm (2);

    sm->FSSNewCommand (fss_cmd_hold);
    sm->setLowBattery (true);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setLowBattery (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("a healthy battery reading resets the low battery debounce", "[state_machine][TC-FS-011][TC-FS-012]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);

    /* Almost enough to latch... */
    for (int i = 0; i < FMUStateMachine::default_low_battery_latch_count - 1; i++)
    {
        sm->setLowBattery (true);
    }
    /* ...but a healthy reading clears the run, so the count restarts from zero. */
    sm->setLowBattery (false);
    for (int i = 0; i < FMUStateMachine::default_low_battery_latch_count - 1; i++)
    {
        sm->setLowBattery (true);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setLowBattery (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* Once the latch has engaged, a later optimistic reading must NOT release it:
 * recovery from a critically low battery requires a restart. */
TEST_CASE ("low battery latch is not cleared by a healthy reading", "[state_machine][TC-FS-011][TC-FS-012]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->setLowBattery (false);
    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* The consecutive-low counter saturates at low_battery_latch_count so a long
 * flight with a sustained low battery cannot overflow it. The latch must engage
 * exactly once and stay engaged no matter how many more low readings arrive. */
TEST_CASE ("low battery latch saturates and stays engaged over a long run", "[state_machine][TC-FS-011][TC-FS-012]")
{
    auto [mav, smm, sm] = make_sm ();

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

/* Design decision: the altitude-cap breach latch is debounced exactly like
 * low-battery (todo/92). A single noisy over-cap EKF sample must not ground
 * the mission. */
TEST_CASE ("altitude breach does not latch before the debounce count", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count - 1; i++)
    {
        sm->setCurrentAltitude (true, 200.0);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setCurrentAltitude (true, 200.0);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("an under-cap reading resets the altitude breach trip-side debounce", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);

    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count - 1; i++)
    {
        sm->setCurrentAltitude (true, 200.0);
    }
    /* An under-cap reading clears the run, so the trip-side count restarts. */
    sm->setCurrentAltitude (true, 50.0);
    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count - 1; i++)
    {
        sm->setCurrentAltitude (true, 200.0);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setCurrentAltitude (true, 200.0);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* altitude_breach_latch_count is configurable, matching low_battery_latch_count
 * (todo/54): a non-default count must actually change the debounce. */
TEST_CASE ("a configured altitude_breach_latch_count changes the debounce", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm]
        = make_sm (FMUStateMachine::default_low_battery_latch_count, FMUStateMachine::default_altitude_cap_m, 2);

    sm->FSSNewCommand (fss_cmd_hold);
    sm->setCurrentAltitude (true, 200.0);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setCurrentAltitude (true, 200.0);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

/* Unlike low_battery, the altitude breach latch is self-clearing (todo/92):
 * once altitude drops back under the cap for the debounce window, control
 * returns to whatever FSS/SMM command is current -- here, a resumed search. */
TEST_CASE ("altitude breach self-clears once altitude drops back under the cap", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    FMUState last_state = fmu_state_manual;
    sm->setStateChangeCB ([&] (FMUState s) { last_state = s; });

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (last_state == fmu_state_searching);

    latch_altitude_breach (sm);
    REQUIRE (last_state == fmu_state_rtl);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count; i++)
    {
        sm->setCurrentAltitude (true, 50.0);
    }
    REQUIRE (last_state == fmu_state_searching);
}

TEST_CASE ("an invalid fix never trips the altitude breach latch", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();
    sm->FSSNewCommand (fss_cmd_hold);

    for (int i = 0; i < 1000; i++)
    {
        sm->setCurrentAltitude (false, 200.0);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);
}

/* A garbled (NaN/Inf) reading must not trip the latch (over_altitude_cap's
 * strict '>' comparison against NaN is always false, so an unguarded caller
 * would silently treat it as under-cap) nor, symmetrically, clear one that is
 * already engaged. Negative readings are deliberately NOT covered here --
 * they are legitimate AGL data, not garbage, and are exercised as ordinary
 * under-cap readings elsewhere (e.g. the self-clearing test uses 50.0, but
 * see mav_io_test.cpp for the real-world negative-AGL note). */
TEST_CASE ("a non-finite altitude reading is ignored like an invalid fix", "[state_machine][altitude_cap]")
{
    double nan = std::numeric_limits<double>::quiet_NaN ();
    double inf = std::numeric_limits<double>::infinity ();

    {
        auto [mav, smm, sm] = make_sm ();
        sm->FSSNewCommand (fss_cmd_hold);
        for (int i = 0; i < 1000; i++)
        {
            sm->setCurrentAltitude (true, nan);
        }
        REQUIRE (mav->last_mode == flight_mode_hold);
    }
    {
        auto [mav, smm, sm] = make_sm ();
        sm->FSSNewCommand (fss_cmd_hold);
        for (int i = 0; i < 1000; i++)
        {
            sm->setCurrentAltitude (true, inf);
        }
        REQUIRE (mav->last_mode == flight_mode_hold);
    }
    {
        /* Does not clear an already-engaged latch either. */
        auto [mav, smm, sm] = make_sm ();
        latch_altitude_breach (sm);
        REQUIRE (mav->last_mode == flight_mode_rtl);
        for (int i = 0; i < 1000; i++)
        {
            sm->setCurrentAltitude (true, nan);
        }
        REQUIRE (mav->last_mode == flight_mode_rtl);
    }
}

TEST_CASE ("an invalid fix never clears an engaged altitude breach latch", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();
    sm->FSSNewCommand (fss_cmd_hold);
    latch_altitude_breach (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    for (int i = 0; i < 1000; i++)
    {
        sm->setCurrentAltitude (false, 50.0);
    }
    REQUIRE (mav->last_mode == flight_mode_rtl);

    /* A subsequent valid-fix under-cap run does clear it. */
    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count; i++)
    {
        sm->setCurrentAltitude (true, 50.0);
    }
    REQUIRE (mav->last_mode == flight_mode_hold);
}

TEST_CASE ("an invalid fix reading mid-run does not corrupt the altitude breach debounce",
           "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();
    sm->FSSNewCommand (fss_cmd_hold);

    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count - 1; i++)
    {
        sm->setCurrentAltitude (true, 200.0);
    }
    /* Skipped entirely, not treated as a reset. */
    sm->setCurrentAltitude (false, 200.0);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setCurrentAltitude (true, 200.0);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("altitude breach latch saturates and stays engaged over a long run", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    for (int i = 0; i < 1000; i++)
    {
        sm->setCurrentAltitude (true, 200.0);
    }
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (mav->set_mode_calls == 1);

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (mav->set_mode_calls == 1);
}

TEST_CASE ("terminate overrides an active altitude breach", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_altitude_breach (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (!mav->terminated);

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
}

TEST_CASE ("low battery overrides an active altitude breach", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_altitude_breach (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    FMUState last_state = fmu_state_manual;
    sm->setStateChangeCB ([&] (FMUState s) { last_state = s; });
    latch_low_battery (sm);
    REQUIRE (last_state == fmu_state_low_battery);
}

TEST_CASE ("mav comms failure overrides an active altitude breach", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_altitude_breach (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    FMUState last_state = fmu_state_manual;
    sm->setStateChangeCB ([&] (FMUState s) { last_state = s; });
    sm->setMavCommsFailure (true);
    REQUIRE (last_state == fmu_state_failsafe);
}

/* An altitude breach that engages internally while comms are already lost
 * must not force an extra reaction (comms failsafe already has it in RTL);
 * once comms recover, the still-engaged breach (never cleared -- no
 * under-cap readings arrived) keeps it in RTL via its own priority slot,
 * not the comms-failsafe one. */
TEST_CASE ("an altitude breach persists across a mav comms outage", "[state_machine][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    FMUState last_state = fmu_state_manual;
    sm->setStateChangeCB ([&] (FMUState s) { last_state = s; });

    sm->setMavCommsFailure (true);
    REQUIRE (last_state == fmu_state_failsafe);

    latch_altitude_breach (sm);
    REQUIRE (last_state == fmu_state_failsafe);

    sm->setMavCommsFailure (false);
    REQUIRE (last_state == fmu_state_rtl);
}

TEST_CASE ("an explicit FSS command during an active altitude breach resolves superseded",
           "[state_machine][altitude_cap][command_ack]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_altitude_breach (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_superseded);
    REQUIRE (res.superseding_state == fmu_state_rtl);
    REQUIRE (mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("mav comms failure triggers failsafe RTL", "[state_machine][TC-MAV-004][TC-MAV-005]")
{
    auto [mav, smm, sm] = make_sm ();

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
    auto [mav, smm, sm] = make_sm ();

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

TEST_CASE ("mav comms failure clears and restores prior FSS command",
           "[state_machine][TC-MAV-004][TC-MAV-005][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);

    sm->setMavCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    sm->setMavCommsFailure (false);
    REQUIRE (mav->last_mode == flight_mode_hold);
}

TEST_CASE ("terminate overrides mav comms failure", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->setMavCommsFailure (true);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    REQUIRE (!mav->terminated);

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);
}

TEST_CASE ("mav comms failure does not displace an active terminate", "[state_machine][TC-MAV-016]")
{
    auto [mav, smm, sm] = make_sm ();

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
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_disarm);
    REQUIRE (mav->disarmed);
}

TEST_CASE ("state_change_cb fires once per state change, not on no-op transitions", "[state_machine]")
{
    auto [mav, smm, sm] = make_sm ();

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
    auto [mav, smm, sm] = make_sm ();

    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_actioned);
    REQUIRE (res.transitioned);
}

TEST_CASE ("FSS command already in the target state resolves as actioned no-op", "[state_machine][command_ack]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_actioned);
    /* The aircraft is in the commanded state, but this command did not move it. */
    REQUIRE (!res.transitioned);
}

TEST_CASE ("FSS command superseded by low battery names the latch", "[state_machine][command_ack]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_superseded);
    REQUIRE (res.superseding_state == fmu_state_low_battery);
}

TEST_CASE ("FSS command superseded by comms failsafe names the latch", "[state_machine][command_ack]")
{
    auto [mav, smm, sm] = make_sm ();

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
    auto [mav, smm, sm] = make_sm ();

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
           "[state_machine][command_ack][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

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

/* terminate latches (todo/63): the flight-termination action (motor cut /
 * parachute / force-disarm) is physically irreversible, so a later FSS
 * command must not silently move the FMU's own state back out of terminate —
 * it is superseded by the latch, the same as low-battery/comms-failsafe. */
TEST_CASE ("FSS command after terminate is superseded, not actioned", "[state_machine][command_ack]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_terminate);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (res.outcome == fss_command_superseded);
    REQUIRE (res.superseding_state == fmu_state_terminate);
    REQUIRE_FALSE (res.transitioned);
    /* hold's own MAV command must never have been sent: the latch, not the
     * command, is in control. */
    REQUIRE (mav->set_mode_calls == 0);
}

TEST_CASE ("terminate latch survives further commands and cannot be un-terminated", "[state_machine]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminated);

    sm->FSSNewCommand (fss_cmd_continue);
    sm->FSSNewCommand (fss_cmd_hold);
    sm->FSSNewCommand (fss_cmd_manual);

    /* None of the later commands may have re-armed/moved the aircraft: the
     * only MAV call ever made is the original terminate. */
    REQUIRE (mav->set_mode_calls == 0);
    REQUIRE (mav->disarm_calls == 0);
    REQUIRE (mav->terminate_calls == 1);
}

TEST_CASE ("terminate command itself resolves as actioned", "[state_machine][command_ack]")
{
    auto [mav, smm, sm] = make_sm ();

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
    auto [mav, smm, sm] = make_sm ();

    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_actioned);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_none);
}

TEST_CASE ("ack mapping: an already-in-state command is noop", "[command_ack][mapping]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_noop);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_none);
}

TEST_CASE ("ack mapping: low-battery supersede carries the low-battery reason", "[command_ack][mapping]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_superseded);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_low_battery);
}

TEST_CASE ("ack mapping: comms-loss supersede carries the comms-loss reason", "[command_ack][mapping]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->setCommsFailure (true);
    auto res = sm->FSSNewCommand (fss_cmd_goto);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_superseded);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_comms_loss);
}

/* The transport enum has no dedicated reason for an altitude-cap breach
 * (todo/92) -- adding one is a cross-repo protocol change, out of scope
 * here. It falls back to supersede_none, the same fallback as terminate. Pin
 * this current (imperfect) behavior so a future protocol addition is a
 * deliberate, visible diff rather than a silent behavior change. */
TEST_CASE ("ack mapping: altitude-cap breach supersede falls back to no reason", "[command_ack][mapping][altitude_cap]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_altitude_breach (sm);
    auto res = sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (fss_command_ack_outcome_for (res) == fsst::command_ack_superseded);
    REQUIRE (fss_command_ack_reason_for (res) == fsst::supersede_none);
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

TEST_CASE ("same command on two connections actions once, both acked the same", "[command_ack][group][TC-MAV-006]")
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

TEST_CASE ("a duplicate arriving after resolution is acked immediately from the cache",
           "[command_ack][group][TC-MAV-006]")
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

TEST_CASE ("an older different command within the window is acked superseded, not actuated",
           "[command_ack][group][TC-MAV-006]")
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

TEST_CASE ("a different older command is never actuated as new no matter how far outside the window "
           "(todo/86)",
           "[command_ack][group][TC-MAV-006]")
{
    constexpr int cmd_hold = 5;
    constexpr int cmd_rtl = 1;
    constexpr int cmd_terminate = 9;

    /* Boundary: older by exactly the tolerance window. The legacy dedup treated
     * this as "outside the window" and let it fall through to actuate; it must
     * still be rejected as stale — staleness has no upper bound. */
    {
        CommandAckGroup<int> group{ group_tolerance_ms };
        auto current = group.onDelivery (cmd_hold, 100000, 1);
        REQUIRE (current.disposition == CommandAckGroup<int>::Disposition::actuate);
        auto stale = group.onDelivery (cmd_rtl, 100000 - group_tolerance_ms, 2);
        REQUIRE (stale.disposition == CommandAckGroup<int>::Disposition::stale_superseded);
    }

    /* Older by one millisecond more than the window: definitely rejected. */
    {
        CommandAckGroup<int> group{ group_tolerance_ms };
        auto current = group.onDelivery (cmd_hold, 100000, 1);
        REQUIRE (current.disposition == CommandAckGroup<int>::Disposition::actuate);
        auto stale = group.onDelivery (cmd_rtl, 100000 - group_tolerance_ms - 1, 2);
        REQUIRE (stale.disposition == CommandAckGroup<int>::Disposition::stale_superseded);
    }

    /* Older by several hours: a delayed/reconnect-replayed server delivery of an
     * old command must not be actioned just because it is long outside the
     * dedup window. Use terminate as the command: this is the worst case named
     * in the todo, since terminate latches permanently once actuated. */
    {
        constexpr uint64_t five_hours_ms = 5ULL * 60 * 60 * 1000;
        CommandAckGroup<int> group{ group_tolerance_ms };
        auto current = group.onDelivery (cmd_hold, five_hours_ms + 100000, 1);
        REQUIRE (current.disposition == CommandAckGroup<int>::Disposition::actuate);
        auto stale = group.onDelivery (cmd_terminate, 100000, 2);
        REQUIRE (stale.disposition == CommandAckGroup<int>::Disposition::stale_superseded);
    }

    /* Equal timestamp, different command: not "older", so it is a genuinely new
     * command and must actuate rather than being mistaken for stale. */
    {
        CommandAckGroup<int> group{ group_tolerance_ms };
        auto current = group.onDelivery (cmd_hold, 100000, 1);
        REQUIRE (current.disposition == CommandAckGroup<int>::Disposition::actuate);
        auto simultaneous = group.onDelivery (cmd_rtl, 100000, 2);
        REQUIRE (simultaneous.disposition == CommandAckGroup<int>::Disposition::actuate);
    }

    /* Newer, different command, far outside the window: also a genuinely new
     * command and must actuate. */
    {
        constexpr uint64_t five_hours_ms = 5ULL * 60 * 60 * 1000;
        CommandAckGroup<int> group{ group_tolerance_ms };
        auto current = group.onDelivery (cmd_hold, 100000, 1);
        REQUIRE (current.disposition == CommandAckGroup<int>::Disposition::actuate);
        auto later = group.onDelivery (cmd_rtl, 100000 + five_hours_ms, 2);
        REQUIRE (later.disposition == CommandAckGroup<int>::Disposition::actuate);
    }
}

TEST_CASE ("a new command supersedes an unresolved group and its copies are handed back",
           "[command_ack][group][TC-MAV-006]")
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

/* todo/86 (retry-suppression half): a server_command_id lets a connection's
 * redelivery of the SAME operator action be told apart from that connection
 * delivering a genuinely NEW action with identical command/payload — the
 * upstream (flight-safety-system todo/49) contract cap-fmu was blocked on. */

TEST_CASE ("a same-command retry with a fresh server command id re-actuates a resolved command",
           "[command_ack][group][TC-MAV-006]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_goto = 3;
    constexpr uint64_t connection_a = 1;
    const auto target = CommandPayload::forPosition (-43.5, 172.6);

    auto first = group.onDelivery (cmd_goto, 1000, 1, target, /*server_command_id=*/100, connection_a);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);
    group.resolve (first.epoch, actioned ());

    /* Same connection, same command/payload/timestamp window, but the goto's
     * upload never completed and the operator retried: the server assigns a
     * fresh DB row, so a new server_command_id arrives. That must reach the
     * state machine again, not replay the stale cached ack. */
    auto retry = group.onDelivery (cmd_goto, 1000, 2, target, /*server_command_id=*/101, connection_a);
    REQUIRE (retry.disposition == CommandAckGroup<int>::Disposition::actuate);
}

TEST_CASE ("a same-command redelivery with the same server command id still dedups", "[command_ack][group][TC-MAV-006]")
{
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_hold = 5;
    constexpr uint64_t connection_a = 1;

    auto first = group.onDelivery (cmd_hold, 1000, 1, {}, /*server_command_id=*/100, connection_a);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);

    /* Same id from the same connection (resend window, reconnect identify,
     * server bounce): still the same operator action, not a retry. */
    auto redelivery = group.onDelivery (cmd_hold, 1000, 2, {}, /*server_command_id=*/100, connection_a);
    REQUIRE (redelivery.disposition == CommandAckGroup<int>::Disposition::pending);

    group.resolve (first.epoch, actioned ());
    auto late_redelivery = group.onDelivery (cmd_hold, 1000, 3, {}, /*server_command_id=*/100, connection_a);
    REQUIRE (late_redelivery.disposition == CommandAckGroup<int>::Disposition::already_resolved);
}

TEST_CASE ("a second connection's first copy is still a duplicate, not a retry, even with its own id",
           "[command_ack][group][TC-MAV-006]")
{
    /* Ids are only comparable within one connection (upstream contract): a
     * second server's first delivery of the same operator action carries its
     * OWN id (a different DB row on that server), which must not be mistaken
     * for a retry just because it has never been seen before. */
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_hold = 5;
    constexpr uint64_t connection_a = 1;
    constexpr uint64_t connection_b = 2;

    auto first = group.onDelivery (cmd_hold, 1000, 1, {}, /*server_command_id=*/100, connection_a);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);

    auto second = group.onDelivery (cmd_hold, 1000, 2, {}, /*server_command_id=*/555, connection_b);
    REQUIRE (second.disposition == CommandAckGroup<int>::Disposition::pending);
}

TEST_CASE ("a server command id of zero never overrides the timestamp-window heuristic",
           "[command_ack][group][TC-MAV-006]")
{
    /* 0 means "not reported" (legacy peer, or the connection did not negotiate
     * FSS_FEATURE_SERVER_COMMAND_ID): behaviour must stay exactly the
     * pre-todo/86-fix timestamp-window dedup. */
    CommandAckGroup<int> group{ group_tolerance_ms };
    constexpr int cmd_hold = 5;
    constexpr uint64_t connection_a = 1;

    auto first = group.onDelivery (cmd_hold, 1000, 1, {}, /*server_command_id=*/0, connection_a);
    REQUIRE (first.disposition == CommandAckGroup<int>::Disposition::actuate);
    group.resolve (first.epoch, actioned ());

    auto redelivery = group.onDelivery (cmd_hold, 1000, 2, {}, /*server_command_id=*/0, connection_a);
    REQUIRE (redelivery.disposition == CommandAckGroup<int>::Disposition::already_resolved);
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

TEST_CASE ("clamp_command_altitude converts feet to metres and clamps to [floor, cap]", "[altitude][TC-MAV-018]")
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

    /* Regression (todo/55): an altitude that does not fit in 16 bits must pin to
     * the cap, never wrap. 65536 ft (0x10000) truncates to 0 as a uint16_t, which
     * would have clamped up to the floor -- i.e. a "far too high" command would
     * have become "as low as allowed". Taken full-width it is ~19974 m -> cap. */
    REQUIRE (clamp_command_altitude (65536, floor, cap) == cap);
    /* 65536 + 33: low 16 bits == 33 ft (~10 m, the floor). Full-width -> cap. */
    REQUIRE (clamp_command_altitude (65569, floor, cap) == cap);
    /* The maximum wire value stays pinned to the cap. */
    REQUIRE (clamp_command_altitude (UINT32_MAX, floor, cap) == cap);
}

TEST_CASE ("over_altitude_cap compares AGL altitude against the cap", "[altitude_cap]")
{
    constexpr uint16_t cap = 122;

    /* Just under the cap: not a breach. */
    REQUIRE_FALSE (over_altitude_cap (121.9, cap));
    /* Exactly at the cap: strict '>', so "at the cap" is not itself a breach
     * -- a search/goto/direct-altitude command is legitimately clamped to
     * fly exactly there. */
    REQUIRE_FALSE (over_altitude_cap (122.0, cap));
    /* Just over: a breach. */
    REQUIRE (over_altitude_cap (122.1, cap));

    /* Degenerate zero cap: any positive altitude is a breach, zero is not. */
    REQUIRE_FALSE (over_altitude_cap (0.0, 0));
    REQUIRE (over_altitude_cap (0.1, 0));
}

TEST_CASE ("horizontal_velocity is a Pythagorean magnitude with no int overflow", "[mav][velocity]")
{
    /* 3-4-5 triangle, and sign-independence. */
    REQUIRE (horizontal_velocity (300, 400) == 500);
    REQUIRE (horizontal_velocity (-300, -400) == 500);
    REQUIRE (horizontal_velocity (0, 0) == 0);
    REQUIRE (horizontal_velocity (0, 250) == 250);

    /* Regression (todo/57): the extreme inputs. sqrt((vx*vx)+(vy*vy)) does the
     * squares in int and overflows signed int at INT16_MIN (UB / UBSan trip);
     * hypot in double does not. hypot(32768,32768) ~= 46340.95 -> 46340, and the
     * magnitude of two int16_t components always fits in uint16_t. */
    REQUIRE (horizontal_velocity (INT16_MIN, INT16_MIN) == 46340);
    REQUIRE (horizontal_velocity (INT16_MAX, INT16_MAX) == 46339);
    REQUIRE (horizontal_velocity (INT16_MIN, 0) == 32768);
}

/* todo/80: lat/lon degrees<->degE7 encoding at the numeric extremes (TC-MAV-018
 * boundary share). The geofence *policy* half of TC-MAV-018 (southern/western
 * coordinates, longitude wrap, coordinate-system confusion) is out of scope for
 * cap-fmu -- no fence logic exists here at all (todo/17, deferred). This is only
 * the numeric encoding used by commandGoto/send_waypoint/sendADSB and decoded
 * from GLOBAL_POSITION_INT: confirms the round-trip is exact and does not
 * silently wrap or overflow at the latitude/longitude/antimeridian extremes. */
TEST_CASE ("degrees_to_degE7 round-trips exactly at the latitude/longitude extremes", "[mav][latlon]")
{
    /* Latitude extremes. */
    REQUIRE (degrees_to_degE7 (-90.0) == -900000000);
    REQUIRE (degrees_to_degE7 (90.0) == 900000000);
    REQUIRE (degrees_to_degE7 (-90.0000001) == -900000001);
    REQUIRE (degrees_to_degE7 (90.0000001) == 900000001);

    /* Longitude extremes, including just past the antimeridian: no wrap, the
     * commanded position is not silently corrupted to a wildly different one. */
    REQUIRE (degrees_to_degE7 (-180.0) == -1800000000);
    REQUIRE (degrees_to_degE7 (180.0) == 1800000000);
    REQUIRE (degrees_to_degE7 (-180.0000001) == -1800000001);
    REQUIRE (degrees_to_degE7 (180.0000001) == 1800000001);

    /* Round-trip (degrees -> degE7 -> degrees) is exact at every extreme above,
     * plus a couple of ordinary values for good measure. */
    for (double degrees :
         { -90.0, 90.0, -90.0000001, 90.0000001, -180.0, 180.0, -180.0000001, 180.0000001, 0.0, -43.5, 172.6 })
    {
        REQUIRE (degE7_to_degrees (degrees_to_degE7 (degrees)) == Catch::Approx (degrees));
    }
}

/* todo/85: the shared coordinate validator every external source of a
 * commanded/reported position must pass before its coordinates reach
 * degrees_to_degE7() (whose float-to-int32 cast is undefined behavior on a
 * non-finite or out-of-range input). */
TEST_CASE ("Point::isValid accepts the geographic range and rejects outside it", "[latlon][validate]")
{
    /* Boundary values remain accepted, unchanged from before validation
     * existed (todo/80's exact round-trip test above covers their encoding). */
    REQUIRE (Point (-90.0, -180.0).isValid ());
    REQUIRE (Point (90.0, 180.0).isValid ());
    REQUIRE (Point (0.0, 0.0).isValid ());
    REQUIRE (Point (-43.5, 172.6).isValid ());

    /* Just outside the geographic range. */
    REQUIRE_FALSE (Point (-90.0000001, 0.0).isValid ());
    REQUIRE_FALSE (Point (90.0000001, 0.0).isValid ());
    REQUIRE_FALSE (Point (0.0, -180.0000001).isValid ());
    REQUIRE_FALSE (Point (0.0, 180.0000001).isValid ());

    /* Non-finite. */
    double nan = std::numeric_limits<double>::quiet_NaN ();
    double inf = std::numeric_limits<double>::infinity ();
    REQUIRE_FALSE (Point (nan, 0.0).isValid ());
    REQUIRE_FALSE (Point (0.0, nan).isValid ());
    REQUIRE_FALSE (Point (inf, 0.0).isValid ());
    REQUIRE_FALSE (Point (-inf, 0.0).isValid ());
    REQUIRE_FALSE (Point (0.0, inf).isValid ());
    REQUIRE_FALSE (Point (0.0, -inf).isValid ());

    /* Finite but wildly out of range. */
    REQUIRE_FALSE (Point (1e300, 0.0).isValid ());
    REQUIRE_FALSE (Point (0.0, -1e300).isValid ());
}

/* todo/85: degrees_to_degE7() is a last-resort backstop against UB for a
 * caller that skips Point::isValid() -- confirm it clamps to a deterministic
 * value instead of invoking undefined behavior, without disturbing the
 * representable-input behavior the round-trip test above pins. */
TEST_CASE ("degrees_to_degE7 clamps non-finite and out-of-range input instead of invoking UB", "[latlon][validate]")
{
    double nan = std::numeric_limits<double>::quiet_NaN ();
    double inf = std::numeric_limits<double>::infinity ();
    REQUIRE (degrees_to_degE7 (nan) == 0);
    REQUIRE (degrees_to_degE7 (inf) == 0);
    REQUIRE (degrees_to_degE7 (-inf) == 0);
    REQUIRE (degrees_to_degE7 (1e300) == INT32_MAX);
    REQUIRE (degrees_to_degE7 (-1e300) == INT32_MIN);
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

TEST_CASE ("resolve_gcs_failsafe_param_name maps supported airframes, and returns nullopt for unknown ones (todo/91)",
           "[mav]")
{
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_FIXED_WING) == std::optional<const char *> ("FS_GCS_ENABL"));
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_QUADROTOR) == std::optional<const char *> ("FS_GCS_ENABLE"));
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_COAXIAL) == std::optional<const char *> ("FS_GCS_ENABLE"));
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_HELICOPTER) == std::optional<const char *> ("FS_GCS_ENABLE"));
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_HEXAROTOR) == std::optional<const char *> ("FS_GCS_ENABLE"));
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_OCTOROTOR) == std::optional<const char *> ("FS_GCS_ENABLE"));
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_TRICOPTER) == std::optional<const char *> ("FS_GCS_ENABLE"));
    REQUIRE (resolve_gcs_failsafe_param_name (MAV_TYPE_GROUND_ROVER) == std::optional<const char *> ("FS_GCS_ENABLE"));
    REQUIRE (!resolve_gcs_failsafe_param_name (0).has_value ());
    REQUIRE (!resolve_gcs_failsafe_param_name (MAV_TYPE_GCS).has_value ());
}

TEST_CASE ("expected_failsafe_params only requests AFS_ENABLE/AFS_TERM_ACTION for terminate_action::terminate "
           "(todo/91)",
           "[mav]")
{
    auto has_param = [] (const std::vector<FailsafeParamCheck> &params, const std::string &name)
    { return std::any_of (params.begin (), params.end (), [&] (const auto &p) { return name == p.name; }); };

    auto none_params = expected_failsafe_params (MAV_TYPE_QUADROTOR, terminate_action::none);
    REQUIRE (!has_param (none_params, "AFS_ENABLE"));
    REQUIRE (!has_param (none_params, "AFS_TERM_ACTION"));
    REQUIRE (has_param (none_params, "FS_GCS_ENABLE"));

    auto disarm_params = expected_failsafe_params (MAV_TYPE_QUADROTOR, terminate_action::disarm);
    REQUIRE (!has_param (disarm_params, "AFS_ENABLE"));
    REQUIRE (!has_param (disarm_params, "AFS_TERM_ACTION"));
    REQUIRE (has_param (disarm_params, "FS_GCS_ENABLE"));

    auto terminate_params = expected_failsafe_params (MAV_TYPE_QUADROTOR, terminate_action::terminate);
    REQUIRE (has_param (terminate_params, "AFS_ENABLE"));
    REQUIRE (has_param (terminate_params, "AFS_TERM_ACTION"));
    REQUIRE (has_param (terminate_params, "FS_GCS_ENABLE"));
}

TEST_CASE ("expected_failsafe_params omits the GCS-failsafe entry for an unrecognised airframe (todo/91)", "[mav]")
{
    auto params = expected_failsafe_params (0, terminate_action::terminate);
    REQUIRE (std::any_of (params.begin (), params.end (),
                          [] (const auto &p) { return std::string (p.name) == "AFS_ENABLE"; }));
    REQUIRE (std::any_of (params.begin (), params.end (),
                          [] (const auto &p) { return std::string (p.name) == "AFS_TERM_ACTION"; }));
    REQUIRE (std::none_of (params.begin (), params.end (),
                           [] (const auto &p) { return std::string (p.name) == "FS_GCS_ENABLE"; }));
    /* SYSID_MYGCS goes with the GCS-failsafe entry: it selects whose heartbeat
     * that failsafe watches, so it means nothing when the failsafe itself is
     * not being checked. */
    REQUIRE (std::none_of (params.begin (), params.end (),
                           [] (const auto &p) { return std::string (p.name) == "SYSID_MYGCS"; }));
}

/* todo/104. Every todo/91 check reduced to "this param must read nonzero",
 * which an enable flag satisfies while the failsafe it enables goes on to
 * continue the mission -- so the check passed on an airframe with no backstop.
 * Each entry now judges its own value. */
namespace
{
auto
find_check (const std::vector<FailsafeParamCheck> &params, const std::string &name) -> const FailsafeParamCheck *
{
    auto it = std::find_if (params.begin (), params.end (), [&] (const auto &p) { return name == p.name; });
    return it == params.end () ? nullptr : &*it;
}
} // namespace

TEST_CASE ("expected_failsafe_params rejects a failsafe that fires and then continues the mission (todo/104)", "[mav]")
{
    SECTION ("Plane: FS_LONG_ACTN must be RTL, not the default Continue")
    {
        auto params = expected_failsafe_params (MAV_TYPE_FIXED_WING, terminate_action::none);
        const auto *check = find_check (params, "FS_LONG_ACTN");
        REQUIRE (check != nullptr);
        /* 0 = Continue, the firmware default and the whole point of this item:
         * the failsafe triggers and the aircraft carries on flying. */
        REQUIRE_FALSE (check->accept (0.0F));
        REQUIRE (check->accept (1.0F));
    }

    SECTION ("Plane: FS_GCS_ENABL=3 only fails safe in AUTO, and the FMU also flies GUIDED")
    {
        auto params = expected_failsafe_params (MAV_TYPE_FIXED_WING, terminate_action::none);
        const auto *check = find_check (params, "FS_GCS_ENABL");
        REQUIRE (check != nullptr);
        REQUIRE_FALSE (check->accept (0.0F));
        REQUIRE (check->accept (1.0F));
        REQUIRE (check->accept (2.0F));
        REQUIRE_FALSE (check->accept (3.0F));
    }

    SECTION ("Copter: FS_GCS_ENABLE=2 continues the mission, so nonzero is not enough")
    {
        auto params = expected_failsafe_params (MAV_TYPE_QUADROTOR, terminate_action::none);
        const auto *check = find_check (params, "FS_GCS_ENABLE");
        REQUIRE (check != nullptr);
        REQUIRE_FALSE (check->accept (0.0F));
        REQUIRE (check->accept (1.0F));
        REQUIRE_FALSE (check->accept (2.0F));
        REQUIRE (check->accept (3.0F));
    }

    SECTION ("Copter: FS_OPTIONS bit 1 is the same trap as FS_LONG_ACTN=0")
    {
        auto params = expected_failsafe_params (MAV_TYPE_QUADROTOR, terminate_action::none);
        const auto *check = find_check (params, "FS_OPTIONS");
        REQUIRE (check != nullptr);
        REQUIRE (check->accept (0.0F));
        /* Bit 0 and bit 2 are unrelated options; only bit 1 is the trap. */
        REQUIRE (check->accept (1.0F));
        REQUIRE_FALSE (check->accept (2.0F));
        REQUIRE_FALSE (check->accept (3.0F));
        REQUIRE (check->accept (4.0F));
    }

    SECTION ("Rover: FS_GCS_ENABLE=2 continues in Auto, and FS_ACTION's default is Hold")
    {
        auto params = expected_failsafe_params (MAV_TYPE_GROUND_ROVER, terminate_action::none);
        const auto *enable = find_check (params, "FS_GCS_ENABLE");
        REQUIRE (enable != nullptr);
        REQUIRE (enable->accept (1.0F));
        REQUIRE_FALSE (enable->accept (2.0F));

        const auto *action = find_check (params, "FS_ACTION");
        REQUIRE (action != nullptr);
        REQUIRE_FALSE (action->accept (0.0F));
        REQUIRE (action->accept (1.0F));
        REQUIRE_FALSE (action->accept (2.0F));
        REQUIRE (action->accept (3.0F));
    }
}

TEST_CASE ("expected_failsafe_params requires the GCS failsafe to watch the FMU's own system id (todo/104)", "[mav]")
{
    /* Uniform across the families: the default (255) is MAVProxy's, which runs
     * on the same companion computer, so an FMU process death leaves the
     * autopilot seeing a perfectly healthy GCS. */
    for (uint8_t type : { MAV_TYPE_FIXED_WING, MAV_TYPE_QUADROTOR, MAV_TYPE_GROUND_ROVER })
    {
        auto params = expected_failsafe_params (type, terminate_action::none);
        const auto *check = find_check (params, "SYSID_MYGCS");
        REQUIRE (check != nullptr);
        REQUIRE (check->accept (static_cast<float> (fmu_gcs_sys_id)));
        REQUIRE_FALSE (check->accept (255.0F));
        REQUIRE_FALSE (check->accept (1.0F));
    }
}

TEST_CASE ("the AFS checks still accept any nonzero value (todo/104 did not narrow them)", "[mav]")
{
    auto params = expected_failsafe_params (MAV_TYPE_QUADROTOR, terminate_action::terminate);
    for (const char *name : { "AFS_ENABLE", "AFS_TERM_ACTION" })
    {
        const auto *check = find_check (params, name);
        REQUIRE (check != nullptr);
        REQUIRE_FALSE (check->accept (0.0F));
        REQUIRE (check->accept (1.0F));
        REQUIRE (check->accept (2.0F));
    }
}

TEST_CASE ("every failsafe check carries a predicate and a warning (todo/104)", "[mav]")
{
    /* A null predicate would be dereferenced by checkFailsafeParamReply, and an
     * empty warning would log a bare param name and value with no explanation. */
    for (uint8_t type :
         { uint8_t (MAV_TYPE_FIXED_WING), uint8_t (MAV_TYPE_QUADROTOR), uint8_t (MAV_TYPE_GROUND_ROVER), uint8_t (0) })
    {
        for (auto action : { terminate_action::none, terminate_action::disarm, terminate_action::terminate })
        {
            for (const auto &check : expected_failsafe_params (type, action))
            {
                REQUIRE (check.name != nullptr);
                REQUIRE (check.accept != nullptr);
                REQUIRE_FALSE (check.warning.empty ());
                /* MAVLink's param_id field is 16 bytes; a longer name would be
                 * silently truncated in the request and never answered. */
                REQUIRE (std::string (check.name).size () <= 16);
            }
        }
    }
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

/* todo/64: next_search_point_after_reached_seq is what report_reached() uses
 * to turn a MISSION_ITEM_REACHED seq into the point SMMSearch resumes from —
 * one past the point index mission_item_for would assign that same seq,
 * since SMMSearch::current_point tracks the *next* point, not the one just
 * completed. Pinned directly (as a pure function) so this relationship stays
 * correct if the setup-item count ever changes. */
TEST_CASE ("next_search_point_after_reached_seq is one past mission_item_for's point index", "[mission]")
{
    constexpr std::size_t num_points = 10;
    for (uint16_t seq = search_first_point_seq; seq < search_first_point_seq + num_points; seq++)
    {
        auto item = mission_item_for (seq, num_points, MissionPlanMode::search);
        REQUIRE (item.kind == MissionItemKind::search_point);
        REQUIRE (next_search_point_after_reached_seq (seq) == static_cast<int> (item.point_index) + 1);
    }

    /* The first real search point (seq == search_first_point_seq) resumes at
     * index 1, not 0 - reaching it means point 0 is done and the next target
     * is point 1. */
    REQUIRE (next_search_point_after_reached_seq (search_first_point_seq) == 1);
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

TEST_CASE ("autopilot_restarted trips only on a real backwards jump in autopilot uptime (todo/108)", "[mav][comms]")
{
    constexpr uint32_t margin = autopilot_restart_margin_ms;

    /* Nothing observed yet: the first sample can never look like a restart, at
     * any value. */
    REQUIRE_FALSE (autopilot_restarted (0, 0, margin));
    REQUIRE_FALSE (autopilot_restarted (0, 900000, margin));

    /* A zero *reading* is MAVLink's "field not populated", not a jump back to
     * the epoch -- otherwise every such message would report a restart. */
    REQUIRE_FALSE (autopilot_restarted (900000, 0, margin));

    /* Uptime moving forward, or standing still, is the healthy case. */
    REQUIRE_FALSE (autopilot_restarted (900000, 900200, margin));
    REQUIRE_FALSE (autopilot_restarted (900000, 900000, margin));

    /* A small backwards step is ordinary skew between the two sources feeding
     * this (GLOBAL_POSITION_INT at 5Hz, SYSTEM_TIME at 1Hz), not a restart --
     * including at the exact margin. */
    REQUIRE_FALSE (autopilot_restarted (900000, 899800, margin));
    REQUIRE_FALSE (autopilot_restarted (900000, 900000 - margin, margin));

    /* Past the margin: a genuine reboot, whose first reported uptime is the few
     * milliseconds between boot and the stream's first message. */
    REQUIRE (autopilot_restarted (900000, 900000 - (margin + 1), margin));
    REQUIRE (autopilot_restarted (900000, 1200, margin));
    REQUIRE (autopilot_restarted (900000, 1, margin));
}

TEST_CASE ("reassertState re-commands the current state without a transition (todo/108)", "[state_machine][replay]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (mav->last_mode == flight_mode_hold);
    const int calls_before = mav->set_mode_calls;

    /* The autopilot rebooted: no FMU state changed, but the autopilot has
     * forgotten the mode, so it must be commanded again. */
    sm->reassertState ();
    REQUIRE (mav->set_mode_calls == calls_before + 1);
    REQUIRE (mav->last_mode == flight_mode_hold);
}

TEST_CASE ("reassertState re-issues a goto with its stored target (todo/108)", "[state_machine][replay]")
{
    auto [mav, smm, sm] = make_sm ();

    FSSCommandTarget target{};
    target.position = Point (-43.5, 172.6);
    sm->FSSNewCommand (fss_cmd_goto, target);
    REQUIRE (mav->goto_calls == 1);
    REQUIRE (mav->last_mode == flight_mode_goto);

    /* The reboot wiped the uploaded mission too, so the re-apply must re-drive
     * the goto from the retained target, not just re-set the mode. */
    mav->last_goto = Point{};
    sm->reassertState ();
    REQUIRE (mav->goto_calls == 2);
    REQUIRE (mav->last_goto.getLatitude () == Catch::Approx (-43.5));
    REQUIRE (mav->last_goto.getLongitude () == Catch::Approx (172.6));
    REQUIRE (mav->last_mode == flight_mode_goto);
}

TEST_CASE ("reassertState re-drives the search while searching (todo/108)", "[state_machine][replay]")
{
    auto [mav, smm, sm] = make_sm ();

    sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (smm->search_calls == 1);

    /* SMM's resume path re-uploads the held search when the mission is no longer
     * loaded on the autopilot -- which is exactly what a restart makes true (the
     * MAV layer drops search_loaded on detecting one). */
    sm->reassertState ();
    REQUIRE (smm->search_calls == 2);
}

TEST_CASE ("reassertState re-commands a latched state that no transition would re-drive (todo/108)",
           "[state_machine][replay][TC-FS-005]")
{
    auto [mav, smm, sm] = make_sm ();

    latch_low_battery (sm);
    REQUIRE (mav->last_mode == flight_mode_rtl);
    const int calls_before = mav->set_mode_calls;

    sm->reassertState ();
    REQUIRE (mav->set_mode_calls == calls_before + 1);
    REQUIRE (mav->last_mode == flight_mode_rtl);

    /* Same for terminate, whose latch equally produces no transition to ride. */
    const int terminate_before = mav->terminate_calls;
    sm->FSSNewCommand (fss_cmd_terminate);
    REQUIRE (mav->terminate_calls == terminate_before + 1);
    sm->reassertState ();
    REQUIRE (mav->terminate_calls == terminate_before + 2);
}

TEST_CASE ("known_aircraft assigns and retrieves consistent ICAO address", "[aircraft]")
{
    NullLogger null_logger;
    known_aircraft ka (null_logger);
    /* Debounce/eviction are driven by known_aircraft's own local clock, not
     * the report's wire timestamp (todo/89); inject a fake one so the test
     * does not depend on real wall-clock timing. */
    uint64_t now = 1000;
    ka.setNowMsFn ([&now] { return now; });
    std::string callsign = "TEST123";

    uint32_t icao1 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 >= first_icao_address_for_unknown_aircraft);

    uint32_t icao2 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 == icao2);

    PositionData pd;
    pd = PositionData (0, 0, 0, 0, 0, 0, callsign, 0, 0, 0, 0, 0, 0);
    ka.newPositionReport (pd);

    uint32_t icao3 = ka.getAircraftICAOAddress (callsign);
    REQUIRE (icao1 == icao3);
}

TEST_CASE ("known_aircraft evicts aircraft that go quiet past the eviction window", "[aircraft]")
{
    NullLogger null_logger;
    known_aircraft ka (null_logger);
    const uint64_t window_ms = static_cast<uint64_t> (aircraft_eviction_age.count ());
    /* todo/89: the sweep clock is known_aircraft's own now_ms_fn, not the
     * report's wire timestamp; a fake clock keeps this deterministic. */
    uint64_t now = 1000;
    ka.setNowMsFn ([&now] { return now; });

    /* Establish aircraft A at t=1000 and capture its synthetic ICAO. */
    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "A", 0, 0, 0, 0, 0, 0));
    const uint32_t icao_a_first = ka.getAircraftICAOAddress ("A");

    /* A different aircraft reports well past the eviction window. That report
     * drives the sweep, which must reclaim the now-stale entry for A. */
    now = 1000 + window_ms + 1000;
    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "B", 0, 0, 0, 0, 0, 0));

    /* Re-introducing the previously-seen callsign A must allocate a fresh
     * synthetic ICAO, proving the old entry was evicted rather than retained. */
    const uint32_t icao_a_second = ka.getAircraftICAOAddress ("A");
    REQUIRE (icao_a_second != icao_a_first);
}

TEST_CASE ("known_aircraft keeps actively-reporting aircraft across the window", "[aircraft]")
{
    NullLogger null_logger;
    known_aircraft ka (null_logger);
    const uint64_t window_ms = static_cast<uint64_t> (aircraft_eviction_age.count ());
    /* todo/89: fake clock, same reasoning as the eviction test above. */
    uint64_t now = 1000;
    ka.setNowMsFn ([&now] { return now; });

    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "C", 0, 0, 0, 0, 0, 0));
    const uint32_t icao_c_first = ka.getAircraftICAOAddress ("C");

    /* C keeps reporting; each report refreshes its timestamp, so it must never
     * be evicted and must retain its original synthetic ICAO. */
    for (now = 1000 + window_ms; now <= 1000 + (3 * window_ms); now += window_ms)
    {
        ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "C", 0, 0, 0, 0, 0, 0));
    }

    REQUIRE (ka.getAircraftICAOAddress ("C") == icao_c_first);
}

/* todo/89 regression: a peer's self-reported wire timestamp must not drive
 * known_aircraft's debounce or eviction. Before the fix, a single report
 * carrying a wildly bogus (e.g. far-future) timestamp would jam that
 * aircraft's own `ts` forward, permanently rejecting every later genuine
 * report from it, and would separately drive the eviction sweep's "now" to
 * that same bogus value, evicting every other tracked aircraft in the same
 * call. */
TEST_CASE ("known_aircraft ignores a peer's self-reported timestamp for debounce/eviction", "[aircraft]")
{
    NullLogger null_logger;
    known_aircraft ka (null_logger);
    uint64_t now = 1000;
    ka.setNowMsFn ([&now] { return now; });

    /* A peer with no NTP, a stuck RTC, or a glitching ADS-B decoder — no
     * malice required — could report this. */
    constexpr uint64_t bogus_future_ts = 1'000'000'000'000ULL;

    /* D's first report, with the bogus wire timestamp. */
    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "D", 0, 0, bogus_future_ts, 0, 0, 0));

    /* E is unrelated and reports on the real (fake) local clock. */
    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "E", 0, 0, 0, 0, 0, 0));
    const uint32_t icao_e_first = ka.getAircraftICAOAddress ("E");

    /* Local time advances normally (1.5s); D reports again, still with the
     * same bogus wire timestamp. If the wire timestamp still drove the
     * debounce, D's `ts` would have jumped to bogus_future_ts on the first
     * call, and this genuine, well-spaced local update would be rejected
     * forever after. */
    now += 1500;
    REQUIRE (ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "D", 0, 0, bogus_future_ts, 0, 0, 0)));

    /* E must survive: if the bogus wire timestamp drove the eviction sweep's
     * "now" (as it did before the fix), it would have evicted every other
     * tracked aircraft, including E, in D's very first call above. */
    REQUIRE (ka.getAircraftICAOAddress ("E") == icao_e_first);
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
    REQUIRE (cfg.low_battery_latch_count == def.low_battery_latch_count);
    REQUIRE (cfg.reconnect_interval_s == def.reconnect_interval_s);
    REQUIRE (cfg.mav_address == def.mav_address);
    REQUIRE (cfg.mav_port == def.mav_port);
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
            "low_battery_latch_count": 6,
            "reconnect_interval_s": 30,
            "position_stream_interval_ms": 100,
            "battery_stream_interval_ms": 2000,
            "smm_position_report_interval_ms": 500,
            "smm_connect_timeout_s": 3,
            "smm_transfer_timeout_s": 8,
            "mav_address": "192.168.1.50",
            "mav_port": 14550,
            "log_level": "debug"
        }
    })");
    REQUIRE (cfg.altitude_cap_m == 100);
    REQUIRE (cfg.altitude_floor_m == 20);
    REQUIRE (cfg.goto_altitude_m == 60);
    REQUIRE (cfg.camera_fov_deg == Catch::Approx (60.0));
    REQUIRE (cfg.lowbat_threshold == 25);
    REQUIRE (cfg.low_battery_latch_count == 6);
    REQUIRE (cfg.reconnect_interval_s == 30);
    REQUIRE (cfg.position_stream_interval_ms == 100);
    REQUIRE (cfg.battery_stream_interval_ms == 2000);
    REQUIRE (cfg.smm_position_report_interval_ms == 500);
    REQUIRE (cfg.smm_connect_timeout_s == 3);
    REQUIRE (cfg.smm_transfer_timeout_s == 8);
    REQUIRE (cfg.mav_address == "192.168.1.50");
    REQUIRE (cfg.mav_port == 14550);
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

/* todo/109: `warning` sits between error and info, so a config asking for it
 * gets the degraded-but-handled lines without the full info stream. An
 * unrecognised level must still fall back to the default rather than being
 * coerced to the nearest match. */
TEST_CASE ("loadFmuConfig reads every log level, including warning", "[config]")
{
    REQUIRE (load_config (R"({ "fmu": { "log_level": "error" } })").log_level == LogLevel::error);
    REQUIRE (load_config (R"({ "fmu": { "log_level": "warning" } })").log_level == LogLevel::warning);
    REQUIRE (load_config (R"({ "fmu": { "log_level": "info" } })").log_level == LogLevel::info);
    REQUIRE (load_config (R"({ "fmu": { "log_level": "debug" } })").log_level == LogLevel::debug);

    REQUIRE (load_config (R"({ "fmu": { "log_level": "warn" } })").log_level == def.log_level);
    REQUIRE (load_config (R"({ "fmu": { "log_level": 3 } })").log_level == def.log_level);
}

TEST_CASE ("loadFmuConfig reads the MAV endpoint and fails hard on bad values", "[config]")
{
    FmuConfig cfg = load_config (R"({ "fmu": { "mav_address": "10.0.0.2", "mav_port": 5762 } })");
    REQUIRE (cfg.mav_address == "10.0.0.2");
    REQUIRE (cfg.mav_port == 5762);

    /* An absent endpoint keeps the (valid) defaults — only a present-but-invalid
     * value is an error. */
    FmuConfig absent = load_config (R"({ "fmu": { "lowbat_threshold": 25 } })");
    REQUIRE (absent.mav_address == def.mav_address);
    REQUIRE (absent.mav_port == def.mav_port);

    /* The MAV endpoint is safety-relevant, so a present-but-invalid value throws
     * (fail hard) rather than silently using the default and connecting to the
     * wrong/no autopilot. */
    REQUIRE_THROWS (load_config (R"({ "fmu": { "mav_address": "" } })"));
    REQUIRE_THROWS (load_config (R"({ "fmu": { "mav_address": "   " } })"));
    REQUIRE_THROWS (load_config (R"({ "fmu": { "mav_address": 42 } })"));
    REQUIRE_THROWS (load_config (R"({ "fmu": { "mav_port": 70000 } })"));
    REQUIRE_THROWS (load_config (R"({ "fmu": { "mav_port": 0 } })"));
    REQUIRE_THROWS (load_config (R"({ "fmu": { "mav_port": "5760" } })"));
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
     * [1, 3600], the stream/report intervals outside their ranges, and an unknown
     * log level each fall back to their default. */
    FmuConfig cfg = load_config (R"({
        "fmu": {
            "camera_fov_deg": 200.0,
            "lowbat_threshold": 150,
            "low_battery_latch_count": 0,
            "reconnect_interval_s": 0,
            "position_stream_interval_ms": 10,
            "battery_stream_interval_ms": 90000,
            "smm_position_report_interval_ms": 0,
            "smm_connect_timeout_s": 0,
            "smm_transfer_timeout_s": 120,
            "log_level": "verbose"
        }
    })");
    REQUIRE (cfg.camera_fov_deg == Catch::Approx (def.camera_fov_deg));
    REQUIRE (cfg.lowbat_threshold == def.lowbat_threshold);
    REQUIRE (cfg.low_battery_latch_count == def.low_battery_latch_count);
    REQUIRE (cfg.reconnect_interval_s == def.reconnect_interval_s);
    REQUIRE (cfg.position_stream_interval_ms == def.position_stream_interval_ms);
    REQUIRE (cfg.battery_stream_interval_ms == def.battery_stream_interval_ms);
    REQUIRE (cfg.smm_position_report_interval_ms == def.smm_position_report_interval_ms);
    REQUIRE (cfg.smm_connect_timeout_s == def.smm_connect_timeout_s);
    REQUIRE (cfg.smm_transfer_timeout_s == def.smm_transfer_timeout_s);
    REQUIRE (cfg.log_level == def.log_level);
}

TEST_CASE ("loadFmuConfig fails hard on malformed JSON", "[config]")
{
    /* The file is load-bearing for FSS regardless (todo/74), so a malformed
     * file throws rather than silently falling back to defaults and then
     * failing fatally moments later on the FSS side. */
    REQUIRE_THROWS_AS (load_config ("{ this is not valid json "), std::runtime_error);
}

TEST_CASE ("loadFmuConfig fails hard when the file is missing", "[config]")
{
    REQUIRE_THROWS_AS (loadFmuConfig ("/nonexistent/cap-fmu-no-such-config.json"), std::runtime_error);
}

/* Logger writes to a real directory, so tests get a unique temp one and clean
 * it up afterwards, mirroring TempConfigFile above. */
namespace
{
class TempLogDir
{
  public:
    TempLogDir () : path{}
    {
        std::string tmpl = (std::filesystem::temp_directory_path () / "cap-fmu-logger-test-XXXXXX").string ();
        std::vector<char> buf (tmpl.begin (), tmpl.end ());
        buf.push_back ('\0');
        REQUIRE (mkdtemp (buf.data ()) != nullptr);
        path = buf.data ();
    }
    TempLogDir (const TempLogDir &) = delete;
    TempLogDir (TempLogDir &&) = delete;
    auto operator= (const TempLogDir &) -> TempLogDir & = delete;
    auto operator= (TempLogDir &&) -> TempLogDir & = delete;
    ~TempLogDir ()
    {
        std::error_code ec;
        std::filesystem::remove_all (path, ec);
    }
    [[nodiscard]] auto
    str () const -> const std::string &
    {
        return path;
    }
    [[nodiscard]] auto
    logFile () const -> std::string
    {
        return path + "/fmu.log";
    }

  private:
    std::string path;
};
} // namespace

TEST_CASE ("Logger rotates in-flight once the size threshold is exceeded", "[logger]")
{
    TempLogDir dir;
    /* A tiny threshold so the test writes only a handful of lines rather than
     * megabytes (todo/75: production default is 10MB, overridable here via
     * the constructor's max_bytes parameter). */
    Logger logger (dir.str (), LogLevel::info, 100);

    REQUIRE (std::filesystem::exists (dir.logFile ()));
    REQUIRE_FALSE (std::filesystem::exists (dir.logFile () + ".1"));

    /* Each line is well under 100 bytes, so several are needed to cross the
     * threshold and trigger the in-flight rotation. */
    for (int i = 0; i < 10; i++)
    {
        logger.log ("line " + std::to_string (i));
    }
    /* log() only enqueues (todo/88); wait for the worker to catch up before
     * inspecting the file it writes. */
    logger.flush ();

    /* The pre-rotation content moved to .1; the live file is fresh (small). */
    REQUIRE (std::filesystem::exists (dir.logFile ()));
    REQUIRE (std::filesystem::exists (dir.logFile () + ".1"));
    REQUIRE (std::filesystem::file_size (dir.logFile ()) < 100);
}

/* todo/109: severity is stated once, by the level argument, and the line
 * renders it — which is what lets call sites drop the "WARN:" prefixes that
 * used to contradict the level they were logged at. */
TEST_CASE ("Logger names each line's level and filters on it", "[logger]")
{
    TempLogDir dir;
    Logger logger (dir.str (), LogLevel::warning);

    logger.log (LogLevel::error, "an error");
    logger.log (LogLevel::warning, "a warning");
    /* Above the configured level: dropped, not merely untagged. */
    logger.log (LogLevel::info, "an info");
    logger.log (LogLevel::debug, "a debug");
    logger.flush ();

    std::ifstream check (dir.logFile ());
    std::string content ((std::istreambuf_iterator<char> (check)), std::istreambuf_iterator<char> ());
    REQUIRE (content.find ("ERROR an error") != std::string::npos);
    REQUIRE (content.find ("WARNING a warning") != std::string::npos);
    REQUIRE (content.find ("an info") == std::string::npos);
    REQUIRE (content.find ("a debug") == std::string::npos);
}

TEST_CASE ("Logger in-flight rotation keeps writing after rotating", "[logger]")
{
    TempLogDir dir;
    Logger logger (dir.str (), LogLevel::info, 50);

    for (int i = 0; i < 30; i++)
    {
        logger.log ("line " + std::to_string (i));
    }

    /* Several rotations should have occurred; the live file must still be
     * open and accepting writes (not left closed after a failed reopen). */
    logger.flush ();
    REQUIRE (std::filesystem::exists (dir.logFile ()));
    logger.log ("still alive");
    logger.flush ();
    std::ifstream check (dir.logFile ());
    std::string content ((std::istreambuf_iterator<char> (check)), std::istreambuf_iterator<char> ());
    REQUIRE (content.find ("still alive") != std::string::npos);
}

namespace
{
/* Test-only Logger that gates the write for one line behind a promise the
 * test controls, so a stalled disk can be simulated deterministically
 * (todo/88) instead of relying on real I/O being slow. Overrides the
 * protected writeLine() seam (mirroring SMM's fetchSearch/commitSearch test
 * seams) rather than the disk itself. */
class BlockingLogger : public Logger
{
  public:
    using Logger::Logger;
    std::atomic<bool> write_started{ false };
    std::promise<void> release_write{};
    /* std::promise::get_future() may only be called once (a second call
     * throws std::future_error); wrap it in a shared_future, retrieved once
     * here, so writeLine() can wait() on it regardless of how many times it
     * runs (PR 220 review). */
    std::shared_future<void> release_write_future{ release_write.get_future ().share () };

    /* Release the gate and stop the worker before our own members above are
     * destroyed. Two things this guards against (PR 220 review):
     *  - Without releasing first: if a test never reaches
     *    release_write.set_value() (e.g. an earlier REQUIRE failed and
     *    unwound the stack), stopWorker()'s join() would wait forever on a
     *    worker blocked in writeLine()'s wait() below.
     *  - Without calling stopWorker() before returning: writeLine() is
     *    virtual and runs on the worker thread, which Logger::~Logger()
     *    alone would not join until after write_started/release_write/
     *    release_write_future are already destroyed — a background-thread-
     *    vs-destructor race. stopWorker() is idempotent, so ~Logger()
     *    calling it again afterwards is a no-op. */
    ~BlockingLogger () override
    {
        try
        {
            release_write.set_value ();
        }
        catch (const std::future_error &)
        {
            /* Already released by the test; nothing to do. */
        }
        stopWorker ();
    }

  protected:
    void
    writeLine (const std::string &line) override
    {
        write_started.store (true);
        release_write_future.wait ();
        Logger::writeLine (line);
    }
};
} // namespace

/* todo/88 regression: log() must return without waiting on the write, even
 * when the write is stalled (a full/read-only disk, a wedged network log_dir,
 * an in-flight rotation). If log() regressed to a synchronous write, this
 * test would hang (the write is only ever released further down) rather than
 * reach the assertions below. */
TEST_CASE ("Logger::log() does not block on a stalled write", "[logger]")
{
    TempLogDir dir;
    BlockingLogger logger (dir.str ());

    /* Reaching this line at all is the primary proof: log() enqueues and
     * returns immediately, well before the gated write below is released. */
    logger.log ("hello");

    /* Wait for the worker to actually reach the gated write (bounded: it is
     * only waiting on the queue_cv wakeup, not on anything external), so the
     * content check below is not racing the worker's own wakeup latency. */
    while (!logger.write_started.load ())
    {
        std::this_thread::yield ();
    }

    /* The write is still gated: the line must not have reached disk yet. */
    {
        std::ifstream check (dir.logFile ());
        std::string content ((std::istreambuf_iterator<char> (check)), std::istreambuf_iterator<char> ());
        REQUIRE (content.find ("hello") == std::string::npos);
    }

    logger.release_write.set_value ();
    logger.flush ();

    {
        std::ifstream check (dir.logFile ());
        std::string content ((std::istreambuf_iterator<char> (check)), std::istreambuf_iterator<char> ());
        REQUIRE (content.find ("hello") != std::string::npos);
    }
}

/* The queue that makes log() non-blocking (todo/88) has to be bounded, or a
 * log_dir that stops accepting writes turns "never block the producer" into
 * "grow until the companion computer runs out of memory". BlockingLogger holds
 * the worker inside a write, which is exactly the stalled-device condition, so
 * the backlog can be built up deterministically. */
TEST_CASE ("Logger bounds its write queue and records what it dropped", "[logger]")
{
    TempLogDir dir;
    /* Bound of 4 queued lines so the test needs a handful, not thousands. */
    BlockingLogger logger (dir.str (), LogLevel::info, Logger::default_max_log_bytes, 4);

    logger.log ("first");
    /* Wait for the worker to take "first" and stall inside the write, so the
     * remaining lines genuinely queue up behind it. */
    while (!logger.write_started.load ())
    {
        std::this_thread::yield ();
    }

    /* Fill the queue to its bound, then overrun it. keep-1..keep-4 are the last
     * four enqueued, so they are what must survive; drop-1/drop-2 are pushed
     * out from the front. */
    logger.log ("drop-1");
    logger.log ("drop-2");
    logger.log ("keep-1");
    logger.log ("keep-2");
    logger.log ("keep-3");
    logger.log ("keep-4");

    logger.release_write.set_value ();
    logger.flush ();

    std::ifstream check (dir.logFile ());
    std::string content ((std::istreambuf_iterator<char> (check)), std::istreambuf_iterator<char> ());

    /* The oldest queued lines are the ones discarded: during an incident the
     * most recent lines are the ones worth keeping. */
    REQUIRE (content.find ("drop-1") == std::string::npos);
    REQUIRE (content.find ("drop-2") == std::string::npos);
    for (const auto *kept : { "first", "keep-1", "keep-2", "keep-3", "keep-4" })
    {
        REQUIRE (content.find (kept) != std::string::npos);
    }

    /* The loss must be recorded in the log, so a reader cannot mistake the hole
     * for a quiet period. */
    REQUIRE (content.find ("LOG dropped 2 line(s)") != std::string::npos);
    REQUIRE (content.find ("ERROR LOG dropped") != std::string::npos);
}

/* Rotation is destructive --- it shifts fmu.log.1..5 along and discards the
 * oldest --- so it must never be driven by bytes that did not reach the disk.
 * Otherwise a full or disconnected log device erases the existing log history
 * while writing nothing to replace it, at exactly the moment those logs matter.
 * next_rotation_state() is the decision, factored out of writeLine() so it can
 * be checked without a real failing filesystem. */
TEST_CASE ("A failed write neither counts bytes nor triggers rotation", "[logger]")
{
    /* A successful write accumulates and rotates at the threshold. */
    REQUIRE (next_rotation_state (true, 0, 40, 100).bytes_written == 40);
    REQUIRE_FALSE (next_rotation_state (true, 0, 40, 100).rotate);
    REQUIRE (next_rotation_state (true, 60, 40, 100).rotate);
    REQUIRE (next_rotation_state (true, 60, 40, 100).bytes_written == 100);

    /* A failed one does neither, however large the line or however close to
     * the threshold the file already is. */
    REQUIRE (next_rotation_state (false, 60, 40, 100).bytes_written == 60);
    REQUIRE_FALSE (next_rotation_state (false, 60, 40, 100).rotate);
    REQUIRE_FALSE (next_rotation_state (false, 99, 1000000, 100).rotate);
    REQUIRE (next_rotation_state (false, 99, 1000000, 100).bytes_written == 99);

    /* Repeated failures cannot creep the count towards the threshold either. */
    std::size_t bytes = 0;
    for (int i = 0; i < 1000; i++)
    {
        RotationState next = next_rotation_state (false, bytes, 50, 100);
        bytes = next.bytes_written;
        REQUIRE_FALSE (next.rotate);
    }
    REQUIRE (bytes == 0);
}

/* todo/59 acceptance: a subsystem's runtime diagnostic must reach the
 * persistent log file, not only std::cout/std::cerr. known_aircraft is the
 * easiest subsystem to prove this against directly (pure, no sockets); the
 * same ILogger seam is now used by MAV/SMM (verified by compilation, since
 * their diagnostics run on real I/O paths not exercised at unit level). */
TEST_CASE ("known_aircraft diagnostics reach the persistent log file", "[aircraft][logger]")
{
    TempLogDir dir;
    Logger logger (dir.str ());
    known_aircraft ka (logger);

    ka.newPositionReport (PositionData (0, 0, 0, 0, 0, 0, "LOGTEST", 0, 0, 1000, 0, 0, 0));
    logger.flush ();

    std::ifstream check (dir.logFile ());
    std::string content ((std::istreambuf_iterator<char> (check)), std::istreambuf_iterator<char> ());
    REQUIRE (content.find ("Creating new aircraft with callsign LOGTEST") != std::string::npos);
}

/* EventDispatcher (todo/76) is App::run()'s std::visit dispatch policy,
 * extracted so it can be driven here with the same MockMAV/MockSMM used
 * above, plus a MockFSSReporter, instead of needing real sockets. */
namespace
{
/* Test double for the ADS-B throttle's "now" source (todo/81): a plain
 * counter advanced explicitly, so the 1s throttle window can be crossed
 * without a real sleep_for. Injected into EventDispatcher::setNowMsFn as a
 * lambda over a shared_ptr, mirroring the shared-mock pattern MockMAV/MockSMM
 * already use in this fixture. */
struct FakeClock
{
    uint64_t t{ 0 };
    void
    advance (uint64_t ms)
    {
        t += ms;
    }
};

struct DispatcherFixture
{
    std::shared_ptr<MockMAV> mav;
    std::shared_ptr<MockSMM> smm;
    std::shared_ptr<MockFSSReporter> fss;
    std::shared_ptr<FMUStateMachine> sm;
    std::unique_ptr<TempLogDir> log_dir;
    std::unique_ptr<Logger> logger;
    std::unique_ptr<EventDispatcher> dispatcher;
    std::shared_ptr<FakeClock> clock;

    DispatcherFixture () : mav{}, smm{}, fss{}, sm{}, log_dir{}, logger{}, dispatcher{}, clock{} {}
};

auto
make_dispatcher (const std::string &asset_name = "test-asset", int lowbat_threshold = 20) -> DispatcherFixture
{
    DispatcherFixture f;
    f.mav = std::make_shared<MockMAV> ();
    f.smm = std::make_shared<MockSMM> ();
    f.fss = std::make_shared<MockFSSReporter> ();
    f.sm = std::make_shared<FMUStateMachine> (*f.mav, *f.smm);
    f.log_dir = std::make_unique<TempLogDir> ();
    f.logger = std::make_unique<Logger> (f.log_dir->str ());
    f.dispatcher
        = std::make_unique<EventDispatcher> (*f.sm, *f.mav, *f.smm, *f.fss, *f.logger, asset_name, lowbat_threshold);
    f.clock = std::make_shared<FakeClock> ();
    f.dispatcher->setNowMsFn ([clock = f.clock] () { return clock->t; });
    return f;
}
} // namespace

TEST_CASE ("EventDispatcher gates SmmLoadSearch on isSearching()", "[event_dispatcher]")
{
    auto f = make_dispatcher ();

    /* FMUStateMachine starts in fmu_state_manual, not searching: a raced
     * SmmLoadSearch outcome must be dropped (todo/33), not applied. */
    event not_searching = SmmLoadSearch{ nullptr };
    f.dispatcher->dispatch (not_searching);
    REQUIRE (f.mav->load_search_calls == 0);

    f.sm->FSSNewCommand (fss_cmd_continue);
    event while_searching = SmmLoadSearch{ nullptr };
    f.dispatcher->dispatch (while_searching);
    REQUIRE (f.mav->load_search_calls == 1);
}

TEST_CASE ("EventDispatcher routes SmmRtl through the state machine's own arbitration (todo/70)", "[event_dispatcher]")
{
    auto f = make_dispatcher ();

    /* A higher-priority FSS command already in effect. Suppression is now
     * performed by SMMNewCommand's own priority arbitration inside
     * FMUStateMachine, not an isSearching() guard in this handler, so this is
     * exercised with a real command in effect rather than the bare
     * "dispatch before any FSS command" setup the old guard-based test used
     * -- that setup only worked because of an unrelated pre-existing quirk
     * (an uncommanded fss_command defaults to fss_cmd_unknown, which itself
     * maps to searching, same as "fss_cmd_continue with smm_cmd_abandon_search
     * leads to searching" above exploits) that real production sequencing can
     * never reach SmmRtl through, since SMM can only fire it once the FMU has
     * already been in fmu_state_searching at least once. */
    f.sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (f.mav->last_mode == flight_mode_hold);
    int calls_before = f.mav->set_mode_calls;

    event suppressed = SmmRtl{};
    f.dispatcher->dispatch (suppressed);
    REQUIRE (f.mav->set_mode_calls == calls_before);
    REQUIRE (f.mav->last_mode == flight_mode_hold);

    /* While genuinely searching, SmmRtl really does take effect: the FMU
     * moves to waiting_for_tasking (RTL flight mode) without cancelling the
     * searching role (todo/77). */
    f.sm->FSSNewCommand (fss_cmd_continue);
    int cancel_calls_before = f.smm->cancel_calls;
    event real_rtl = SmmRtl{};
    f.dispatcher->dispatch (real_rtl);
    REQUIRE (f.mav->last_mode == flight_mode_rtl);
    REQUIRE (f.smm->cancel_calls == cancel_calls_before);
}

TEST_CASE ("EventDispatcher routes an operator SmmOperatorCommand through SMMNewCommand (todo/90)",
           "[event_dispatcher]")
{
    auto f = make_dispatcher ();

    /* A higher-priority FSS command already in effect: an operator
     * mission-complete command must not override it, exactly like SmmRtl
     * above -- both flow through the same SMMNewCommand arbitration. */
    f.sm->FSSNewCommand (fss_cmd_hold);
    REQUIRE (f.mav->last_mode == flight_mode_hold);
    int calls_before = f.mav->set_mode_calls;

    event suppressed = SmmOperatorCommand{ smm_cmd_mission_complete };
    f.dispatcher->dispatch (suppressed);
    REQUIRE (f.mav->set_mode_calls == calls_before);
    REQUIRE (f.mav->last_mode == flight_mode_hold);

    /* While genuinely searching, an operator mission-complete command really
     * does take effect: the FMU moves to waiting_for_tasking (RTL flight
     * mode) without cancelling the searching role (todo/77), same as the
     * internal SmmRtl outcome. */
    f.sm->FSSNewCommand (fss_cmd_continue);
    int cancel_calls_before = f.smm->cancel_calls;
    event mc = SmmOperatorCommand{ smm_cmd_mission_complete };
    f.dispatcher->dispatch (mc);
    REQUIRE (f.mav->last_mode == flight_mode_rtl);
    REQUIRE (f.smm->cancel_calls == cancel_calls_before);

    /* An operator abandon-search command arriving on top of that in-flight
     * waiting_for_tasking resumes searching (map_smm_state() maps
     * abandon_search to fmu_state_searching) rather than being suppressed by
     * it -- the deliberate interaction the todo calls for confirming rather
     * than assuming. */
    int search_calls_before = f.smm->search_calls;
    event as = SmmOperatorCommand{ smm_cmd_abandon_search };
    f.dispatcher->dispatch (as);
    REQUIRE (f.smm->search_calls > search_calls_before);
}

TEST_CASE (
    "EventDispatcher auto-reacquires a search while waiting_for_tasking, without a duplicate upload (todo/70/77)",
    "[event_dispatcher]")
{
    auto f = make_dispatcher ();

    f.sm->FSSNewCommand (fss_cmd_continue);
    REQUIRE (f.smm->search_calls == 1);

    event rtl_evt = SmmRtl{};
    f.dispatcher->dispatch (rtl_evt);
    REQUIRE (f.mav->last_mode == flight_mode_rtl);
    REQUIRE (f.smm->cancel_calls == 0);

    /* SMM's background retry (kept alive above) reacquires a search and
     * reports it -- no operator action (no FSSNewCommand call here). The
     * first SmmLoadSearch while waiting_for_tasking must not upload directly
     * (that would double-upload against the real SMM's own resume re-issue,
     * see the SmmLoadSearch handler's comment) -- it resumes searching
     * instead. */
    event ls1 = SmmLoadSearch{ nullptr };
    f.dispatcher->dispatch (ls1);
    REQUIRE (f.mav->load_search_calls == 0);
    REQUIRE (f.smm->search_calls == 2);
    REQUIRE (f.smm->cancel_calls == 0);

    /* The real SMM's resume path re-fires load_search_cb once more with the
     * same search, now that the FMU is genuinely searching again -- this is
     * the one that actually uploads. */
    event ls2 = SmmLoadSearch{ nullptr };
    f.dispatcher->dispatch (ls2);
    REQUIRE (f.mav->load_search_calls == 1);
}

TEST_CASE ("EventDispatcher never classifies an unknown battery reading as low, but still feeds the debounce",
           "[event_dispatcher][TC-FS-011][TC-FS-012]")
{
    auto f = make_dispatcher ("asset", 20);
    f.sm->FSSNewCommand (fss_cmd_continue);

    /* One short of the latch count of real low readings... */
    for (int i = 0; i < FMUStateMachine::default_low_battery_latch_count - 1; i++)
    {
        event e = BatteryData{ 10, 1000, 3.5 };
        f.dispatcher->dispatch (e);
    }
    REQUIRE (f.mav->last_mode != flight_mode_rtl);

    /* ...an unknown (-1) reading in between must not be silently dropped nor
     * misclassified as low: it is still fed to the state machine (as
     * not-low), which resets the consecutive-low run exactly like a genuine
     * healthy reading would. */
    event unknown = BatteryData{ -1, -1, 0.0 };
    f.dispatcher->dispatch (unknown);
    REQUIRE (f.fss->report_battery_status_calls == FMUStateMachine::default_low_battery_latch_count);

    for (int i = 0; i < FMUStateMachine::default_low_battery_latch_count - 1; i++)
    {
        event e = BatteryData{ 10, 1000, 3.5 };
        f.dispatcher->dispatch (e);
    }
    REQUIRE (f.mav->last_mode != flight_mode_rtl);

    event last_low = BatteryData{ 10, 1000, 3.5 };
    f.dispatcher->dispatch (last_low);
    REQUIRE (f.mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("EventDispatcher routes ReachedPoint only while searching (todo/69)", "[event_dispatcher]")
{
    auto f = make_dispatcher ();

    /* Not searching (e.g. paused for a goto): a reached event must not move
     * the held search's point nor report bogus search status to FSS. */
    event not_searching = ReachedPoint{ 3 };
    f.dispatcher->dispatch (not_searching);
    REQUIRE (f.smm->reached_point_calls == 0);
    REQUIRE (f.fss->reached_point_calls == 0);

    f.sm->FSSNewCommand (fss_cmd_continue);
    event while_searching = ReachedPoint{ 3 };
    f.dispatcher->dispatch (while_searching);
    REQUIRE (f.smm->reached_point_calls == 1);
    REQUIRE (f.smm->last_reached_point == 3);
    REQUIRE (f.fss->reached_point_calls == 1);
    REQUIRE (f.fss->last_reached_point == 3);
}

TEST_CASE ("EventDispatcher filters own-callsign OtherAircraftReport before ADS-B rebroadcast", "[event_dispatcher]")
{
    auto f = make_dispatcher ("MYCALL", 20);

    PositionData own (0.0, 0.0, 100.0, 0, 0, 0, "MYCALL", 0, 0, 0, 0, 0, 0);
    event own_report = OtherAircraftReport{ own };
    f.dispatcher->dispatch (own_report);
    REQUIRE (f.mav->send_adsb_calls == 0);

    PositionData other (0.0, 0.0, 100.0, 0, 0, 0, "OTHER", 0, 0, 0, 0, 0, 0);
    event other_report = OtherAircraftReport{ other };
    f.dispatcher->dispatch (other_report);
    REQUIRE (f.mav->send_adsb_calls == 1);
}

/* todo/81: the CAP test plan (uav_system_test_plan.md §4.1) requires ADS-B
 * rebroadcast rate-limited to one forward per ICAO address per second, so a
 * busy receiver near a real airport cannot flood ArduPilot with ADSB_VEHICLE
 * updates at dump1090's raw rate. */
TEST_CASE ("EventDispatcher throttles ADS-B rebroadcast to one per ICAO address per second (todo/81)",
           "[event_dispatcher]")
{
    auto f = make_dispatcher ("MYCALL", 20);

    constexpr uint32_t icao_a = 0x0A0A0A;
    constexpr uint32_t icao_b = 0x0B0B0B;
    PositionData a1 (0.0, 0.0, 100.0, 0, 0, 0, "AAA", 0, icao_a, 0, 0, 0, 0);
    PositionData a2 (0.0, 0.0, 100.0, 0, 0, 0, "AAA", 0, icao_a, 0, 0, 0, 0);
    PositionData b1 (0.0, 0.0, 100.0, 0, 0, 0, "BBB", 0, icao_b, 0, 0, 0, 0);

    /* First sighting of A forwards. */
    event first_a = OtherAircraftReport{ a1 };
    f.dispatcher->dispatch (first_a);
    REQUIRE (f.mav->send_adsb_calls == 1);

    /* A second report for the same ICAO within the same second is throttled. */
    event second_a = OtherAircraftReport{ a2 };
    f.dispatcher->dispatch (second_a);
    REQUIRE (f.mav->send_adsb_calls == 1);

    /* A different ICAO in between is unaffected by A's throttle. */
    event first_b = OtherAircraftReport{ b1 };
    f.dispatcher->dispatch (first_b);
    REQUIRE (f.mav->send_adsb_calls == 2);

    /* Once 1000ms have passed, A forwards again. */
    f.clock->advance (1000);
    event third_a = OtherAircraftReport{ a1 };
    f.dispatcher->dispatch (third_a);
    REQUIRE (f.mav->send_adsb_calls == 3);
}

/* todo/85: a peer's ADS-B coordinates are untrusted. A non-finite/out-of-range
 * one must be dropped before rebroadcast rather than forwarded (even clamped)
 * to the autopilot's collision-avoidance, or counted against the per-ICAO
 * rate limit (todo/81) as if it were a legitimate sighting. */
TEST_CASE ("EventDispatcher drops an ADS-B report with an invalid coordinate before rebroadcast (todo/85)",
           "[event_dispatcher]")
{
    auto f = make_dispatcher ("MYCALL", 20);

    double nan = std::numeric_limits<double>::quiet_NaN ();
    PositionData bad (nan, 0.0, 100.0, 0, 0, 0, "BAD", 0, 0xBAD1D, 0, 0, 0, 0);
    event bad_report = OtherAircraftReport{ bad };
    f.dispatcher->dispatch (bad_report);
    REQUIRE (f.mav->send_adsb_calls == 0);

    /* A subsequent valid report for the same ICAO is not throttled by the
     * dropped one -- it was never counted as a sighting. */
    PositionData good (0.0, 0.0, 100.0, 0, 0, 0, "BAD", 0, 0xBAD1D, 0, 0, 0, 0);
    event good_report = OtherAircraftReport{ good };
    f.dispatcher->dispatch (good_report);
    REQUIRE (f.mav->send_adsb_calls == 1);
}

/* todo/68 (now narrowed by todo/92): cap-fmu's own-aircraft PositionData event
 * (the FMU's MAV position report, distinct from OtherAircraftReport/ADS-B
 * above) only reaches FMUStateMachine for the altitude-cap breach check
 * (todo/92) -- and only when the position carries a valid fix
 * (POSITION_FLAG_VALID_COORDS). An invalid-fix reading (e.g. GPS-denial,
 * the 6-arg ctor below with no flags set) is fed to setCurrentAltitude but
 * ignored there, so it still causes no MAV reaction at all, however
 * degenerate the coordinates. Pinned here so a future change to GPS-denial
 * handling does so deliberately rather than silently regressing this gap
 * further. */
TEST_CASE ("EventDispatcher never reacts to an invalid-fix PositionData (todo/68)", "[event_dispatcher]")
{
    auto f = make_dispatcher ();

    auto assert_no_reaction = [&] ()
    {
        int mode_calls_before = f.mav->set_mode_calls;
        flight_mode last_mode_before = f.mav->last_mode;
        int report_calls_before = f.fss->report_position_calls;

        event e = PositionData (-43.5, 172.6, 50.0, 0, 0, 0);
        f.dispatcher->dispatch (e);

        /* Still reported to FSS (that part is unaffected)... */
        REQUIRE (f.fss->report_position_calls == report_calls_before + 1);
        /* ...but no MAV command was issued and the mode is unchanged: an
         * invalid-fix reading is not trusted for the altitude-cap check
         * either. */
        REQUIRE (f.mav->set_mode_calls == mode_calls_before);
        REQUIRE (f.mav->last_mode == last_mode_before);
    };

    /* idle (default fmu_state_manual) */
    assert_no_reaction ();

    /* searching */
    f.sm->FSSNewCommand (fss_cmd_continue);
    assert_no_reaction ();

    /* goto */
    f.sm->FSSNewCommand (fss_cmd_goto, FSSCommandTarget{ Point{}, 100 });
    assert_no_reaction ();
}

/* Acceptance-level proof for todo/92: a sequence of valid-fix, over-cap
 * GLOBAL_POSITION_INT-derived PositionData events, dispatched exactly as
 * main.cpp's registerPositionCB would enqueue them, eventually forces RTL --
 * exercised through EventDispatcher::dispatch() (not FMUStateMachine
 * directly), proving the wiring, not just the state-machine unit. */
TEST_CASE ("EventDispatcher drives the altitude-cap breach latch from over-cap PositionData (todo/92)",
           "[event_dispatcher][altitude_cap]")
{
    auto f = make_dispatcher ();
    f.sm->FSSNewCommand (fss_cmd_hold);

    /* PositionData(lat, lng, alt_m, hdg, vel_hor, vel_ver, flags, alt_agl_m):
     * alt_m (MSL) is left at a harmless value; only alt_agl_m matters here. */
    for (int i = 0; i < FMUStateMachine::default_altitude_breach_latch_count - 1; i++)
    {
        event e = PositionData (-43.5, 172.6, 50.0, 0, 0, 0, POSITION_FLAG_VALID_COORDS, 200.0);
        f.dispatcher->dispatch (e);
    }
    REQUIRE (f.mav->last_mode == flight_mode_hold);

    event last = PositionData (-43.5, 172.6, 50.0, 0, 0, 0, POSITION_FLAG_VALID_COORDS, 200.0);
    f.dispatcher->dispatch (last);
    REQUIRE (f.mav->last_mode == flight_mode_rtl);
}

TEST_CASE ("EventDispatcher does not react to valid-fix, under-cap PositionData", "[event_dispatcher][altitude_cap]")
{
    auto f = make_dispatcher ();
    f.sm->FSSNewCommand (fss_cmd_hold);

    for (int i = 0; i < 20; i++)
    {
        event e = PositionData (-43.5, 172.6, 50.0, 0, 0, 0, POSITION_FLAG_VALID_COORDS, 50.0);
        f.dispatcher->dispatch (e);
    }
    REQUIRE (f.mav->last_mode == flight_mode_hold);
    REQUIRE (f.fss->report_position_calls == 20);
}
