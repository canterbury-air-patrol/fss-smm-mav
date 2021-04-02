#include <bits/stdint-uintn.h>
#include <condition_variable>
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

std::string asset_name = "";
std::queue<class event *> event_queue;
std::mutex main_lock;
std::condition_variable main_cv;

bool running = true;

void sigIntHandler(__attribute__((unused)) int signum)
{
    running = false;
}

static void
enqueue_event (event *e)
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
    enqueue_event(new event(command));
}

static void
fss_comms_status_cb (FSSCommsStatus status)
{
    enqueue_event(new event(status));
}

static void
smm_settings_cb (SMMSettings settings)
{
    enqueue_event(new event(settings));
}

static void
mav_position_cb (PositionData pd)
{
    enqueue_event(new event(event_position, pd));
}

static void
mav_reached_cb (int point)
{
    enqueue_event(new event(point));
}

static void
mav_battery_cb (BatteryData bd)
{
    enqueue_event(new event(bd));
}

known_aircraft *aircraft = nullptr;

static void
fss_other_traffic_cb (PositionData pd)
{
    if (aircraft != nullptr)
    {
        if (aircraft->newPositionReport(pd))
        {
            enqueue_event(new event(event_other_aircraft_report, pd));
        }
    }
}

static void
fss_reconnector (FSS *fss, MAV *mav)
{
    while (running)
    {
        sleep (10);
        fss->reconnectAll();
        mav->attemptReconnect();
    }
}

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cout << "Usage: " << argv[0] << " client.json addr port" << std::endl;
        return -1;
    }
    /* Watch out for sigint */
    signal (SIGINT, sigIntHandler);
    /* Ignore SIGPIPE */
    signal (SIGPIPE, SIG_IGN);

    aircraft = new known_aircraft();

    FSS *fss = new FSS(argv[1]);
    MAV *mav = new MAV(argv[2], atoi(argv[3]));
    SMM *smm = new SMM(mav);

    /* Run the reconnector thread */
    std::thread reconnector = std::thread(fss_reconnector, fss, mav);

    /* Get the asset name */
    asset_name = fss->getAssetName();

    /* Setup the State Machine */
    FMUStateMachine *state_machine = new FMUStateMachine(mav, smm, fss);

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
                        if (e->getBatteryData().getRemaining() < 20)
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
            }
            delete e;
            lk.lock();
        }
        main_cv.wait(lk);
    }

    if (reconnector.joinable())
    {
        reconnector.join();
    }

    /* Cleanup */
    delete state_machine;
    delete smm;
    delete fss;
    delete mav;

    while (!event_queue.empty())
    {
        auto e = event_queue.front();
        event_queue.pop();
        delete e;
    }
}