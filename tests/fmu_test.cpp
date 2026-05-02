#include <catch2/catch_test_macros.hpp>

#include "fmu.hpp"

#include <memory>
#include <tuple>

class MockMAV : public IMAV {
public:
    flight_mode last_mode{flight_mode_unknown};
    int set_mode_calls{0};
    bool disarmed{false};
    bool terminated{false};

    MockMAV() = default;
    MockMAV(const MockMAV&) = delete;
    MockMAV(MockMAV&&) = delete;
    auto operator=(const MockMAV&) -> MockMAV& = delete;
    auto operator=(MockMAV&&) -> MockMAV& = delete;
    ~MockMAV() override = default;

    void setMode(flight_mode fm) override { last_mode = fm; set_mode_calls++; }
    void disarm() override { disarmed = true; }
    void terminate() override { terminated = true; }
    void gotoPosition(Point) override {}
    void setAltitude(uint16_t) override {}
    auto getCurrentPosition() -> Point override { return Point{}; }
};

class MockSMM : public ISMM {
public:
    int search_calls{0};

    MockSMM() = default;
    MockSMM(const MockSMM&) = delete;
    MockSMM(MockSMM&&) = delete;
    auto operator=(const MockSMM&) -> MockSMM& = delete;
    auto operator=(MockSMM&&) -> MockSMM& = delete;
    ~MockSMM() override = default;

    void search(Point) override { search_calls++; }
};

class MockFSS : public IFSS {
public:
    Point goto_point{};
    uint16_t altitude_val{0};

    MockFSS() = default;
    MockFSS(const MockFSS&) = delete;
    MockFSS(MockFSS&&) = delete;
    auto operator=(const MockFSS&) -> MockFSS& = delete;
    auto operator=(MockFSS&&) -> MockFSS& = delete;
    ~MockFSS() override = default;

    auto getGoto() -> Point override { return goto_point; }
    auto getAltitude() -> uint16_t override { return altitude_val; }
};

using SM = std::tuple<std::shared_ptr<MockMAV>, std::shared_ptr<MockSMM>, std::shared_ptr<MockFSS>, std::shared_ptr<FMUStateMachine>>;

static auto make_sm() -> SM
{
    auto mav = std::make_shared<MockMAV>();
    auto smm = std::make_shared<MockSMM>();
    auto fss = std::make_shared<MockFSS>();
    auto sm  = std::make_shared<FMUStateMachine>(mav, smm, fss);
    return {mav, smm, fss, sm};
}

TEST_CASE("low battery latches RTL regardless of subsequent FSS commands", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->setLowBattery();
    REQUIRE(mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand(fss_cmd_hold);
    REQUIRE(mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand(fss_cmd_goto);
    REQUIRE(mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand(fss_cmd_manual);
    REQUIRE(mav->last_mode == flight_mode_rtl);
}

TEST_CASE("comms failure latches failsafe until comms restored", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->setCommsFailure(true);
    REQUIRE(mav->last_mode == flight_mode_rtl);

    sm->FSSNewCommand(fss_cmd_hold);
    REQUIRE(mav->last_mode == flight_mode_rtl);

    sm->setCommsFailure(false);
    REQUIRE(mav->last_mode == flight_mode_hold);
}

TEST_CASE("fss_cmd_continue with smm_cmd_none leads to searching", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();
    int calls_before = smm->search_calls;

    sm->FSSNewCommand(fss_cmd_continue);

    REQUIRE(smm->search_calls > calls_before);
}

TEST_CASE("fss_cmd_continue with smm_cmd_abandon_search leads to searching", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->SMMNewCommand(smm_cmd_abandon_search);
    sm->FSSNewCommand(fss_cmd_continue);

    REQUIRE(smm->search_calls > 0);
}

/* Design decision: terminate has the highest priority and overrides both
 * low_battery and comms_failure.  The ground station must always be able to
 * halt the aircraft, even during an emergency RTL. */
TEST_CASE("terminate overrides low battery", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->setLowBattery();
    REQUIRE(mav->last_mode == flight_mode_rtl);
    REQUIRE(!mav->terminated);

    sm->FSSNewCommand(fss_cmd_terminate);
    REQUIRE(mav->terminated);
}

TEST_CASE("terminate overrides comms failure", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->setCommsFailure(true);
    REQUIRE(mav->last_mode == flight_mode_rtl);
    REQUIRE(!mav->terminated);

    sm->FSSNewCommand(fss_cmd_terminate);
    REQUIRE(mav->terminated);
}

/* Design decision: low_battery BLOCKS manual and disarm.  Allowing manual
 * override or a mid-air disarm when the battery is critically low risks loss
 * of the aircraft; RTL is the safe action. */
TEST_CASE("manual blocked when low battery is set", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->setLowBattery();
    sm->FSSNewCommand(fss_cmd_manual);

    REQUIRE(mav->last_mode == flight_mode_rtl);
}

TEST_CASE("disarm blocked when low battery is set", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->setLowBattery();
    sm->FSSNewCommand(fss_cmd_disarm);

    REQUIRE(!mav->disarmed);
    REQUIRE(mav->last_mode == flight_mode_rtl);
}

TEST_CASE("same non-searching state does not re-action", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->FSSNewCommand(fss_cmd_hold);
    int calls = mav->set_mode_calls;

    sm->FSSNewCommand(fss_cmd_hold);
    REQUIRE(mav->set_mode_calls == calls);
}

TEST_CASE("searching always re-actions on new SMM command", "[state_machine]")
{
    auto [mav, smm, fss, sm] = make_sm();

    sm->FSSNewCommand(fss_cmd_continue);
    int calls = smm->search_calls;

    sm->SMMNewCommand(smm_cmd_none);
    REQUIRE(smm->search_calls > calls);
}
