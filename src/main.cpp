#include <bits/stdint-uintn.h>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <list>
#include <queue>
#include <iostream>
#include <thread>
#include <csignal>
#include <unistd.h>
#include "fmu-types.hpp"
#include "fmu.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include "event.hpp"
#include "aircraft.hpp"

constexpr int lowbat_threshold = 20;
constexpr int reconnect_interval = 10;

std::string asset_name;
std::queue<std::shared_ptr<event>> event_queue;
std::mutex main_lock;
std::condition_variable main_cv;

bool running = true;

void sigIntHandler(__attribute__((unused)) int signum)
{
    running = false;
}

static void
enqueue_event (const std::shared_ptr<event> &e)
{
    {
        std::lock_guard<std::mutex> lk(main_lock);
        event_queue.push(e);
    }
    main_cv.notify_one();
}

static void
fss_command_cb (FSSCommand command)
{
    enqueue_event(std::make_shared<event>(command));
}

static void
fss_comms_status_cb (FSSCommsStatus status)
{
    enqueue_event(std::make_shared<event>(status));
}

static void
smm_settings_cb (const SMMSettings &settings)
{
    enqueue_event(std::make_shared<event>(settings));
}

static void
mav_position_cb (const PositionData &pd)
{
    enqueue_event(std::make_shared<event>(event_position, pd));
}

static void
mav_reached_cb (int point)
{
    enqueue_event(std::make_shared<event>(point));
}

static void
mav_battery_cb (const BatteryData &bd)
{
    enqueue_event(std::make_shared<event>(bd));
}

std::shared_ptr<known_aircraft> aircraft = nullptr;

static void
fss_other_traffic_cb (const PositionData &pd)
{
    auto pd_modified = PositionData(pd);
    if (aircraft != nullptr)
    {
        if (aircraft->newPositionReport(pd_modified))
        {
            // Update the ICAO if we didn't get it in the original report
            if (pd_modified.getICAOAddress() == 0)
            {
                pd_modified.setICAOAddress(aircraft->getAircraftICAOAddress(pd_modified.getCallSign()));
            }
            enqueue_event(std::make_shared<event>(event_other_aircraft_report, pd_modified));
        }
    }
}

static void
fss_reconnector (const std::shared_ptr<FSS> &fss, const std::shared_ptr<MAV> &mav)
{
    while (running)
    {
        sleep (reconnect_interval);
        fss->reconnectAll();
        mav->attemptReconnect();
    }
    /* nudge the main loop, in case it hasn't got any events */
    enqueue_event(std::make_shared<event>(event_nudge));
}

auto
main(int argc, char *argv[]) -> int
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " client.json addr port" << std::endl;
        return -1;
    }
    int arg_offset = 1;

    /* Watch out for sigint */
    signal (SIGINT, sigIntHandler);
    /* Ignore SIGPIPE */
    signal (SIGPIPE, SIG_IGN);

    aircraft = std::make_shared<known_aircraft>();

    auto fss = std::make_shared<FSS>(argv[arg_offset++]);
    auto mav = std::make_shared<MAV>(argv[arg_offset], std::stoi(argv[arg_offset+1]));
    auto smm = std::make_shared<SMM>(mav);

    /* Run the reconnector thread */
    std::thread reconnector = std::thread(fss_reconnector, fss, mav);

    /* Get the asset name */
    asset_name = fss->getAssetName();

    /* Setup the State Machine */
    auto state_machine = std::make_shared<FMUStateMachine>(mav, smm, fss);

    /* Connect up the notifications */
    fss->registerCommandCB(fss_command_cb);
    fss->registerCommsStatusCB(fss_comms_status_cb);
    fss->registerSMMSettingsCB(smm_settings_cb);
    fss->registerPositionDataCB(fss_other_traffic_cb);

    mav->registerPositionCB(mav_position_cb);
    mav->registerReachedCB(mav_reached_cb);
    mav->registerBatteryCB(mav_battery_cb);

    while (running)
    {
        std::unique_lock<std::mutex> lk(main_lock);
        while (!event_queue.empty())
        {
            auto e = event_queue.front();
            event_queue.pop();
            lk.unlock();
            switch(e->getType())
            {
                case event_fss_command:
                    state_machine->FSSNewCommand(e->getFSSCommand());
                    break;
                case event_fss_comms_status:
                    state_machine->setCommsFailure((e->getFSSCommsStatus() == fss_comms_failure));
                    break;
                case event_smm_settings:
                    {
                        auto settings = e->getSMMSettings();
                        smm->connect(settings.getURL(), settings.getUsername(), settings.getPassword(), asset_name);
                    }
                    break;
                case event_position:
                    {
                        fss->reportPosition(e->getPositionData());
                        smm->reportPosition(e->getPositionData());
                    }
                    break;
                case event_reached:
                    {
                        fss->reachedPoint(e->getReachedPoint(), smm->currentSearchPoints());
                        smm->reachedPoint(e->getReachedPoint());
                    }
                    break;
                case event_battery_status:
                    {
                        if (e->getBatteryData().getRemaining() < lowbat_threshold)
                        {
                            /* Time to go home */
                            state_machine->setLowBattery();
                        }
                        fss->reportBatteryStatus(e->getBatteryData());
                    }
                    break;
                case event_other_aircraft_report:
                    {
                        PositionData pd = e->getPositionData();
                        if (pd.getCallSign() != fss->getAssetName())
                        {
                            /* Don't tell it about ourself */
                            mav->sendADSB(pd);
                        }
                    }
                    break;
                case event_nudge:
                    break;
                case event_unknown:
                    {
                        std::cout << "Unknown event in queue";
                    }
                    break;
            }
            lk.lock();
        }
        main_cv.wait(lk);
    }

    if (reconnector.joinable())
    {
        reconnector.join();
    }

    /* Cleanup */
    state_machine.reset();
    smm.reset();
    fss.reset();
    mav.reset();

    while (!event_queue.empty())
    {
        auto e = event_queue.front();
        event_queue.pop();
    }
}