#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* Fallback so a build without the autotools-generated config.h still compiles
 * (PACKAGE_STRING is used by --version). */
#ifndef PACKAGE_STRING
#define PACKAGE_STRING "cap-fmu"
#endif

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <exception>
#include <getopt.h>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <variant>

template <class... Ts> struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts> overloaded (Ts...) -> overloaded<Ts...>;
#include "aircraft.hpp"
#include "event.hpp"
#include "fmu-config.hpp"
#include "fmu-types.hpp"
#include "fmu.hpp"
#include "fss/fmu-fss-types.hpp"
#include "fss/fmu-fss.hpp"
#include "logger.hpp"
#include "mav/mav.hpp"
#include "smm/smm-types.hpp"
#include "smm/smm.hpp"

class App
{
  public:
    App (const char *config_file, terminate_action ta, const FmuConfig &cfg, Logger &t_logger)
        : fss (std::make_unique<FSS> (config_file)),
          mav (std::make_unique<MAV> (cfg.mav_address, static_cast<uint16_t> (cfg.mav_port), ta,
                                      MavParams{ cfg.goto_altitude_m, cfg.altitude_floor_m, cfg.altitude_cap_m,
                                                 static_cast<uint32_t> (cfg.position_stream_interval_ms) * 1000U,
                                                 static_cast<uint32_t> (cfg.battery_stream_interval_ms) * 1000U })),
          smm (std::make_unique<SMM> (*mav, cfg.altitude_cap_m, cfg.altitude_floor_m, cfg.camera_fov_deg,
                                      static_cast<uint64_t> (cfg.smm_position_report_interval_ms),
                                      cfg.smm_connect_timeout_s, cfg.smm_transfer_timeout_s)),
          aircraft{}, event_queue{}, main_lock{}, main_cv{}, reconnect_lock{}, reconnect_cv{}, running{ true },
          asset_name (fss->getAssetName ()), logger (t_logger), lowbat_threshold (cfg.lowbat_threshold),
          reconnect_interval_s (cfg.reconnect_interval_s)
    {
    }

    void
    run ()
    {
        FMUStateMachine state_machine{ *mav, *smm, *fss };
        state_machine.setStateChangeCB ([this] (FMUState s)
                                        { logger.log (std::string ("STATE ") + fmu_state_name (s)); });
        logger.log ("START " + asset_name);

        fss->registerCommandCB ([this] (FSSCommand command, const fss_command_ack_responder &ack)
                                { enqueue_event (std::make_shared<event> (FSSCommandEvent{ command, ack })); });
        fss->registerCommsStatusCB ([this] (FSSCommsStatus status)
                                    { enqueue_event (std::make_shared<event> (status)); });
        fss->registerSMMSettingsCB ([this] (const SMMSettings &settings)
                                    { enqueue_event (std::make_shared<event> (settings)); });
        fss->registerPositionDataCB (
            [this] (const PositionData &pd)
            {
                auto pd_modified = PositionData (pd);
                if (aircraft.newPositionReport (pd_modified))
                {
                    if (pd_modified.getICAOAddress () == 0)
                    {
                        pd_modified.setICAOAddress (aircraft.getAircraftICAOAddress (pd_modified.getCallSign ()));
                    }
                    enqueue_event (std::make_shared<event> (OtherAircraftReport{ pd_modified }));
                }
            });

        mav->registerPositionCB ([this] (const PositionData &pd) { enqueue_event (std::make_shared<event> (pd)); });
        mav->registerReachedCB ([this] (int point)
                                { enqueue_event (std::make_shared<event> (ReachedPoint{ point })); });
        mav->registerBatteryCB ([this] (const BatteryData &bd) { enqueue_event (std::make_shared<event> (bd)); });
        mav->registerMavCommsStatusCB ([this] (MavCommsStatus status)
                                       { enqueue_event (std::make_shared<event> (status)); });

        /* All callbacks are now registered; only now open the MAV connection and
         * start its recv/heartbeat threads, so those threads cannot race the
         * registration above. (FSS connects lazily via the reconnector below,
         * which likewise starts after registration.) */
        mav->start ();

        std::thread sig_thread ([this] { signal_waiter (); });
        std::thread reconnector ([this] { fss_reconnector (); });

        while (true)
        {
            std::unique_lock<std::mutex> lk (main_lock);
            main_cv.wait (lk, [this] { return !event_queue.empty () || !running.load (); });
            if (!running.load () && event_queue.empty ())
            {
                break;
            }
            while (!event_queue.empty ())
            {
                auto e = event_queue.front ();
                event_queue.pop ();
                lk.unlock ();
                std::visit (
                    overloaded{
                        [&] (const FSSCommandEvent &ce)
                        {
                            logger.log (std::string ("CMD fss ") + fss_cmd_name (ce.command));
                            FSSCommandResolution res = state_machine.FSSNewCommand (ce.command);
                            /* Acknowledge the resolved outcome back to FSS (no-op
                             * unless the originating connection negotiated the
                             * command-ack feature). */
                            if (ce.ack)
                            {
                                ce.ack (res);
                            }
                        },
                        [&] (FSSCommsStatus status)
                        {
                            logger.log (std::string ("COMMS fss ")
                                        + (status == fss_comms_failure ? "failure" : "okay"));
                            state_machine.setCommsFailure (status == fss_comms_failure);
                        },
                        [&] (MavCommsStatus status)
                        {
                            logger.log (std::string ("COMMS mav ")
                                        + (status == MavCommsStatus::failure ? "failure" : "okay"));
                            state_machine.setMavCommsFailure (status == MavCommsStatus::failure);
                        },
                        [&] (SMMSettings settings)
                        {
                            logger.log ("SMM connect " + settings.getURL ());
                            smm->connect (settings.getURL (), settings.getUsername (), settings.getPassword (),
                                          asset_name);
                        },
                        [&] (const PositionData &pd)
                        {
                            fss->reportPosition (pd);
                            smm->reportPosition (pd);
                        },
                        [&] (const ReachedPoint &rp)
                        {
                            logger.log ("WAYPOINT " + std::to_string (rp.point));
                            fss->reachedPoint (rp.point, smm->currentSearchPoints ());
                            smm->reachedPoint (rp.point);
                        },
                        [&] (BatteryData bd)
                        {
                            auto remaining = bd.getRemaining ();
                            /* A remaining of -1 means "unknown"; only a real reading
                             * below the threshold counts as low. The state machine
                             * debounces these, so feed it every reading (low or not)
                             * to keep its consecutive-low counter accurate. */
                            bool low = remaining >= 0 && remaining < lowbat_threshold;
                            if (low)
                            {
                                logger.log ("BATTERY low " + std::to_string (remaining) + "%");
                            }
                            state_machine.setLowBattery (low);
                            fss->reportBatteryStatus (bd);
                        },
                        [&] (OtherAircraftReport oar)
                        {
                            if (oar.pd.getCallSign () != fss->getAssetName ())
                            {
                                mav->sendADSB (oar.pd);
                            }
                        },
                        [] (const Nudge &) {},
                    },
                    *e);
                lk.lock ();
            }
        }

        if (reconnector.joinable ())
        {
            reconnector.join ();
        }
        if (sig_thread.joinable ())
        {
            sig_thread.join ();
        }

        while (!event_queue.empty ())
        {
            event_queue.pop ();
        }

        logger.log ("STOP");
    }

  private:
    void
    enqueue_event (const std::shared_ptr<event> &e)
    {
        {
            std::lock_guard<std::mutex> lk (main_lock);
            event_queue.push (e);
        }
        main_cv.notify_one ();
    }

    void
    signal_waiter ()
    {
        sigset_t mask;
        sigemptyset (&mask);
        sigaddset (&mask, SIGINT);
        sigaddset (&mask, SIGTERM);
        int signum = 0;
        sigwait (&mask, &signum);
        running.store (false);
        main_cv.notify_one ();
        reconnect_cv.notify_one ();
    }

    void
    fss_reconnector ()
    {
        while (running.load ())
        {
            {
                std::unique_lock<std::mutex> lk (reconnect_lock);
                reconnect_cv.wait_for (lk, std::chrono::seconds (reconnect_interval_s),
                                       [this] { return !running.load (); });
            }
            if (!running.load ())
            {
                break;
            }
            fss->reconnectAll ();
            mav->attemptReconnect ();
            /* Drive a timer-based retry of a pending search acquisition so it is
             * not starved when MAV position reports stop (todo/41). Done after the
             * reconnect attempts above so a slow SMM call cannot delay them within
             * a cycle; the search_retry_ts backoff makes this a cheap no-op when a
             * retry is not yet due. */
            smm->retryPendingSearch ();
        }
        enqueue_event (std::make_shared<event> (Nudge{}));
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

    std::atomic<bool> running{ true };
    std::string asset_name;
    Logger &logger;
    int lowbat_threshold;
    int reconnect_interval_s;
};

static void
print_usage (std::ostream &os, const char *progname)
{
    os << "Usage: " << progname << " --terminate-action=none|disarm|terminate client.json\n";
}

static void
print_help (const char *progname)
{
    print_usage (std::cout, progname);
    std::cout << "\nCanterbury Air Patrol Flight Management Unit.\n\n"
              << "Options:\n"
              << "  --terminate-action=none|disarm|terminate  Flight-termination action (required)\n"
              << "  --version                                 Print version and exit\n"
              << "  --help                                    Print this help and exit\n\n"
              << "Arguments:\n"
              << "  client.json   FSS client configuration file. Its \"fmu\" block also\n"
              << "                carries the MAVLink endpoint (mav_address / mav_port,\n"
              << "                default 127.0.0.1 / 5760).\n";
}

static auto
parse_terminate_action (std::string_view val) -> std::optional<terminate_action>
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
main (int argc, char *argv[]) -> int
{
    static const struct option long_options[] = { { "terminate-action", required_argument, nullptr, 't' },
                                                  { "version", no_argument, nullptr, 'v' },
                                                  { "help", no_argument, nullptr, 'h' },
                                                  { nullptr, 0, nullptr, 0 } };

    std::optional<terminate_action> ta;

    int opt;
    int option_index = 0;
    while ((opt = getopt_long (argc, argv, "", long_options, &option_index)) != -1)
    {
        switch (opt)
        {
            case 't':
                ta = parse_terminate_action (optarg);
                if (!ta)
                {
                    std::cerr << "Error: unknown --terminate-action value '" << optarg
                              << "' (must be none, disarm, or terminate)" << '\n';
                    return 1;
                }
                break;
            case 'v':
                std::cout << PACKAGE_STRING << '\n';
                return 0;
            case 'h':
                print_help (argv[0]);
                return 0;
            default:
                print_usage (std::cerr, argv[0]);
                return 1;
        }
    }

    if (!ta)
    {
        std::cerr << "Error: --terminate-action is required (none, disarm, or terminate)" << '\n';
        print_usage (std::cerr, argv[0]);
        return 1;
    }

    if (argc - optind != 1)
    {
        print_usage (std::cerr, argv[0]);
        return 1;
    }

    /* Block SIGINT and SIGTERM so they can be handled synchronously by signal_waiter.
     * This mask is inherited by all threads spawned below, ensuring the
     * signal is delivered to the dedicated waiter rather than interrupting
     * arbitrary threads from a signal-handler context. */
    sigset_t signal_mask;
    sigemptyset (&signal_mask);
    sigaddset (&signal_mask, SIGINT);
    sigaddset (&signal_mask, SIGTERM);
    pthread_sigmask (SIG_BLOCK, &signal_mask, nullptr);
    /* Ignore SIGPIPE */
    signal (SIGPIPE, SIG_IGN);

    try
    {
        /* The MAVLink endpoint and all other tunables come from the config file's
         * "fmu" block (mav_address / mav_port, validated in loadFmuConfig). */
        FmuConfig cfg = loadFmuConfig (argv[optind]);
        Logger logger (cfg.log_dir, cfg.log_level);
        App app (argv[optind], ta.value (), cfg, logger);
        app.run ();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal: " << e.what () << '\n';
        return 1;
    }
    catch (...)
    {
        std::cerr << "Fatal: unknown exception\n";
        return 1;
    }
    return 0;
}
