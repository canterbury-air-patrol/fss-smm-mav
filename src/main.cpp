#include <cstdint>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <list>
#include <queue>
#include <iostream>
#include <thread>
#include <variant>
#include <csignal>
#include <unistd.h>
#include <getopt.h>

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
#include "fmu-types.hpp"
#include "fmu.hpp"
#include "mav/mav.hpp"
#include "smm/smm.hpp"
#include "fss/fmu-fss.hpp"
#include "fss/fmu-fss-types.hpp"
#include "smm/smm-types.hpp"
#include "event.hpp"
#include "aircraft.hpp"
#include "logger.hpp"

constexpr int lowbat_threshold = 20;
constexpr int reconnect_interval = 10;

class App {
public:
    App(const char *config_file, const char *addr, int port, terminate_action ta, Logger &t_logger)
        : fss(std::make_unique<FSS>(config_file))
        , mav(std::make_unique<MAV>(addr, port, ta))
        , smm(std::make_unique<SMM>(*mav))
        , aircraft{}
        , event_queue{}
        , main_lock{}
        , main_cv{}
        , reconnect_lock{}
        , reconnect_cv{}
        , running{true}
        , asset_name(fss->getAssetName())
        , logger(t_logger)
    {}

    void run()
    {
        FMUStateMachine state_machine{*mav, *smm, *fss};
        state_machine.setStateChangeCB([this](FMUState s) {
            logger.log(std::string("STATE ") + fmu_state_name(s));
        });
        logger.log("START " + asset_name);

        fss->registerCommandCB([this](FSSCommand command) {
            enqueue_event(std::make_shared<event>(command));
        });
        fss->registerCommsStatusCB([this](FSSCommsStatus status) {
            enqueue_event(std::make_shared<event>(status));
        });
        fss->registerSMMSettingsCB([this](const SMMSettings &settings) {
            enqueue_event(std::make_shared<event>(settings));
        });
        fss->registerPositionDataCB([this](const PositionData &pd) {
            auto pd_modified = PositionData(pd);
            if (aircraft.newPositionReport(pd_modified))
            {
                if (pd_modified.getICAOAddress() == 0)
                {
                    pd_modified.setICAOAddress(aircraft.getAircraftICAOAddress(pd_modified.getCallSign()));
                }
                enqueue_event(std::make_shared<event>(OtherAircraftReport{pd_modified}));
            }
        });

        mav->registerPositionCB([this](const PositionData &pd) {
            enqueue_event(std::make_shared<event>(pd));
        });
        mav->registerReachedCB([this](int point) {
            enqueue_event(std::make_shared<event>(ReachedPoint{point}));
        });
        mav->registerBatteryCB([this](const BatteryData &bd) {
            enqueue_event(std::make_shared<event>(bd));
        });
        mav->registerMavCommsStatusCB([this](MavCommsStatus status) {
            enqueue_event(std::make_shared<event>(status));
        });

        std::thread sig_thread([this]{ signal_waiter(); });
        std::thread reconnector([this]{ fss_reconnector(); });

        while (true)
        {
            std::unique_lock<std::mutex> lk(main_lock);
            main_cv.wait(lk, [this]{ return !event_queue.empty() || !running.load(); });
            if (!running.load() && event_queue.empty())
            {
                break;
            }
            while (!event_queue.empty())
            {
                auto e = event_queue.front();
                event_queue.pop();
                lk.unlock();
                std::visit(overloaded{
                    [&](FSSCommand cmd) {
                        logger.log(std::string("CMD fss ") + fss_cmd_name(cmd));
                        state_machine.FSSNewCommand(cmd);
                    },
                    [&](FSSCommsStatus status) {
                        logger.log(std::string("COMMS fss ") + (status == fss_comms_failure ? "failure" : "okay"));
                        state_machine.setCommsFailure(status == fss_comms_failure);
                    },
                    [&](MavCommsStatus status) {
                        logger.log(std::string("COMMS mav ") + (status == MavCommsStatus::failure ? "failure" : "okay"));
                        state_machine.setMavCommsFailure(status == MavCommsStatus::failure);
                    },
                    [&](SMMSettings settings) {
                        logger.log("SMM connect " + settings.getURL());
                        smm->connect(settings.getURL(), settings.getUsername(), settings.getPassword(), asset_name);
                    },
                    [&](PositionData pd) {
                        fss->reportPosition(pd);
                        smm->reportPosition(pd);
                    },
                    [&](const ReachedPoint &rp) {
                        logger.log("WAYPOINT " + std::to_string(rp.point));
                        fss->reachedPoint(rp.point, smm->currentSearchPoints());
                        smm->reachedPoint(rp.point);
                    },
                    [&](BatteryData bd) {
                        auto remaining = bd.getRemaining();
                        if (remaining >= 0 && remaining < lowbat_threshold)
                        {
                            logger.log("BATTERY low " + std::to_string(remaining) + "%");
                            state_machine.setLowBattery();
                        }
                        fss->reportBatteryStatus(bd);
                    },
                    [&](OtherAircraftReport oar) {
                        if (oar.pd.getCallSign() != fss->getAssetName())
                        {
                            mav->sendADSB(oar.pd);
                        }
                    },
                    [](const Nudge &) {},
                }, *e);
                lk.lock();
            }
        }

        if (reconnector.joinable())
        {
            reconnector.join();
        }
        if (sig_thread.joinable())
        {
            sig_thread.join();
        }

        while (!event_queue.empty())
        {
            event_queue.pop();
        }

        logger.log("STOP");
    }

private:
    void enqueue_event(const std::shared_ptr<event> &e)
    {
        {
            std::lock_guard<std::mutex> lk(main_lock);
            event_queue.push(e);
        }
        main_cv.notify_one();
    }

    void signal_waiter()
    {
        sigset_t mask;
        sigemptyset(&mask);
        sigaddset(&mask, SIGINT);
        int signum = 0;
        sigwait(&mask, &signum);
        running.store(false);
        main_cv.notify_one();
        reconnect_cv.notify_one();
    }

    void fss_reconnector()
    {
        while (running.load())
        {
            {
                std::unique_lock<std::mutex> lk(reconnect_lock);
                reconnect_cv.wait_for(lk, std::chrono::seconds(reconnect_interval), [this]{ return !running.load(); });
            }
            if (!running.load())
            {
                break;
            }
            fss->reconnectAll();
            mav->attemptReconnect();
        }
        enqueue_event(std::make_shared<event>(Nudge{}));
    }

    std::unique_ptr<FSS> fss;
    std::unique_ptr<MAV> mav;
    std::unique_ptr<SMM> smm;
    known_aircraft aircraft;

    std::queue<std::shared_ptr<event>> event_queue;
    std::mutex main_lock;
    std::condition_variable main_cv;

    std::mutex reconnect_lock;
    std::condition_variable reconnect_cv;

    std::atomic<bool> running{true};
    std::string asset_name;
    Logger &logger;
};

static void
print_usage(const char *progname)
{
    std::cerr << "Usage: " << progname << " --terminate-action=none|disarm|terminate client.json addr port" << std::endl;
}

static auto
parse_terminate_action(std::string_view val) -> std::optional<terminate_action>
{
    if (val == "none")
    {
        return terminate_action::none;
    }
    if (val == "disarm")
    {
        return terminate_action::disarm;
    }
    if (val == "terminate")
    {
        return terminate_action::terminate;
    }
    return std::nullopt;
}

auto
main(int argc, char *argv[]) -> int
{
    static const struct option long_options[] = {
        {"terminate-action", required_argument, nullptr, 't'},
        {nullptr, 0, nullptr, 0}
    };

    std::optional<terminate_action> ta;

    int opt;
    int option_index = 0;
    while ((opt = getopt_long(argc, argv, "", long_options, &option_index)) != -1)
    {
        if (opt == 't')
        {
            ta = parse_terminate_action(optarg);
            if (!ta)
            {
                std::cerr << "Error: unknown --terminate-action value '" << optarg << "' (must be none, disarm, or terminate)" << std::endl;
                return 1;
            }
        }
        else
        {
            print_usage(argv[0]);
            return 1;
        }
    }

    if (!ta)
    {
        std::cerr << "Error: --terminate-action is required (none, disarm, or terminate)" << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    if (argc - optind != 3)
    {
        print_usage(argv[0]);
        return 1;
    }

    /* Block SIGINT so it can be handled synchronously by signal_waiter.
     * This mask is inherited by all threads spawned below, ensuring the
     * signal is delivered to the dedicated waiter rather than interrupting
     * arbitrary threads from a signal-handler context. */
    sigset_t sigint_mask;
    sigemptyset(&sigint_mask);
    sigaddset(&sigint_mask, SIGINT);
    pthread_sigmask(SIG_BLOCK, &sigint_mask, nullptr);
    /* Ignore SIGPIPE */
    signal(SIGPIPE, SIG_IGN);

    Logger logger("/var/log/cap-fmu");
    App app(argv[optind], argv[optind + 1], std::stoi(argv[optind + 2]), ta.value(), logger);
    app.run();
    return 0;
}
