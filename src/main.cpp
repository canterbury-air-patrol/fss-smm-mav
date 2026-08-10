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

#include "aircraft.hpp"
#include "event-dispatcher.hpp"
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
    /* Member init order follows declaration order (below), not this list. The list
     * is written to match so it does not trip -Wreorder: the queue and its sync
     * primitives are constructed first, then the callback producers (fss, mav, smm)
     * last — so at destruction the producers (and their threads) are torn down
     * before the queue they enqueue onto. mav precedes smm (SMM holds a MAV&); fss
     * precedes asset_name (initialised from fss->getAssetName()). */
    App (const char *config_file, terminate_action ta, const FmuConfig &cfg, Logger &t_logger)
        : event_queue{}, main_lock{}, main_cv{}, reconnect_lock{}, reconnect_cv{}, running{ true }, logger (t_logger),
          lowbat_threshold (cfg.lowbat_threshold), low_battery_latch_count (cfg.low_battery_latch_count),
          altitude_cap_m (cfg.altitude_cap_m), altitude_breach_latch_count (cfg.altitude_breach_latch_count),
          reconnect_interval_s (cfg.reconnect_interval_s), aircraft (logger), fss (std::make_unique<FSS> (config_file)),
          mav (std::make_unique<MAV> (cfg.mav_address, static_cast<uint16_t> (cfg.mav_port), ta,
                                      MavParams{ cfg.goto_altitude_m, cfg.altitude_floor_m, cfg.altitude_cap_m,
                                                 static_cast<uint32_t> (cfg.position_stream_interval_ms) * 1000U,
                                                 static_cast<uint32_t> (cfg.battery_stream_interval_ms) * 1000U,
                                                 static_cast<uint32_t> (cfg.mav_connect_timeout_s) * 1000U,
                                                 static_cast<uint32_t> (cfg.mav_send_timeout_s) * 1000U },
                                      logger)),
          smm (std::make_unique<SMM> (*mav, logger, cfg.altitude_cap_m, cfg.altitude_floor_m, cfg.camera_fov_deg,
                                      static_cast<uint64_t> (cfg.smm_position_report_interval_ms),
                                      cfg.smm_connect_timeout_s, cfg.smm_transfer_timeout_s)),
          asset_name (fss->getAssetName ())
    {
    }

    void
    run ()
    {
        FMUStateMachine state_machine{ *mav, *smm, low_battery_latch_count, altitude_cap_m,
                                       altitude_breach_latch_count };
        /* Constructing this registers it as the state machine's state-change
         * callback, logging the STATE line on every transition. */
        EventDispatcher dispatcher{ state_machine, *mav, *smm, *fss, logger, asset_name, lowbat_threshold };
        logger.log ("START " + asset_name);

        fss->registerCommandCB (
            [this] (FSSCommand command, const FSSCommandTarget &target, const fss_command_ack_responder &ack)
            { enqueue_event (std::make_shared<event> (FSSCommandEvent{ command, target, ack })); });
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
        mav->registerAutopilotRestartCB ([this] { enqueue_event (std::make_shared<event> (MavAutopilotRestart{})); });

        /* SMM I/O runs on its own worker thread; its flight outcomes come back
         * through the event queue so they are applied on the event-loop thread,
         * where the state machine arbitrates priority. */
        smm->registerLoadSearchCB ([this] (const std::shared_ptr<SMMSearch> &search)
                                   { enqueue_event (std::make_shared<event> (SmmLoadSearch{ search })); });
        smm->registerRtlCB ([this] { enqueue_event (std::make_shared<event> (SmmRtl{})); });
        smm->registerOperatorCommandCB ([this] (SMMCommand cmd)
                                        { enqueue_event (std::make_shared<event> (SmmOperatorCommand{ cmd })); });

        /* Start signal handling before any potentially long external
         * operation: SIGINT/SIGTERM are already blocked process-wide (see
         * main()), so this only decides how promptly a pending signal is
         * consumed once raised, not whether it is lost — but starting it first
         * is cheap and keeps that window as small as possible. */
        std::thread sig_thread ([this] { signal_waiter (); });

        /* All callbacks are now registered; only now open the MAV connection and
         * start its recv/heartbeat threads, so those threads cannot race the
         * registration above. (FSS connects lazily via the reconnector below,
         * which likewise starts after registration.) */
        mav->start ();

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
                dispatcher.dispatch (*e);
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

        /* Drain under main_lock: the mav/smm/fss worker threads are not joined until
         * those objects are destroyed (after run() returns), so one could still
         * enqueue here. Holding the lock keeps this drain consistent with a
         * concurrent enqueue_event(); any event enqueued after it is discarded when
         * the queue is destroyed, which now happens after the producers. */
        {
            std::lock_guard<std::mutex> lk (main_lock);
            while (!event_queue.empty ())
            {
                event_queue.pop ();
            }
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
        /* Publish the flag under each waiter's own mutex before notifying it —
         * the same shape as every other shutdown signal in the tree
         * (Logger::stopWorker, SMM::~SMM, FSS::stopWorker). Storing outside the
         * mutex leaves the classic lost-wakeup window: a waiter can evaluate its
         * predicate as false, then have both the store and the notify land in the
         * gap before it blocks, and never see either. That costs more than a
         * missed wakeup here — run()'s main_cv.wait has no timeout, and a
         * reconnector that missed the notify sleeps out a full
         * reconnect_interval_s (10 s by default, up to 3600) before posting the
         * Nudge{} that lets run() finish, so SIGTERM-to-exit would be bounded by
         * the reconnect interval rather than by the work left to do. Docker's
         * default 10 s stop grace sits right on that edge; being SIGKILLed
         * instead would skip the log drain and the FSS ack flush that
         * stopWorker()/~Logger exist to guarantee.
         *
         * The two regions are kept separate, not merged into one scoped_lock over
         * both mutexes, to preserve the "no function ever holds two of these locks
         * at once" invariant documented in docs/threading.md. */
        {
            std::lock_guard<std::mutex> lk (main_lock);
            running.store (false);
        }
        main_cv.notify_one ();
        {
            /* running is already false by the time this runs; taking and releasing
             * reconnect_lock is the whole point — it cannot overlap a reconnector
             * sitting between its predicate check and its wait, so the notify below
             * either finds it already blocked or finds it about to re-check. */
            std::lock_guard<std::mutex> lk (reconnect_lock);
        }
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
             * not starved when MAV position reports stop. Done after the reconnect
             * attempts above so a slow SMM call cannot delay them within a cycle;
             * the search_retry_ts backoff makes this a cheap no-op when a retry is
             * not yet due. */
            smm->retryPendingSearch ();
        }
        enqueue_event (std::make_shared<event> (Nudge{}));
    }

    /* The event queue and its sync primitives are declared first so they are
     * destroyed LAST — after the callback producers below, whose worker/recv
     * threads enqueue onto this queue and touch main_lock/main_cv. See
     * docs/threading.md for App's place in the full thread inventory and
     * lock-ordering model. */
    std::queue<std::shared_ptr<event>> event_queue;
    std::mutex main_lock;
    std::condition_variable main_cv;

    std::mutex reconnect_lock;
    std::condition_variable reconnect_cv;

    std::atomic<bool> running{ true };
    Logger &logger;
    int lowbat_threshold;
    int low_battery_latch_count;
    uint16_t altitude_cap_m;
    int altitude_breach_latch_count;
    int reconnect_interval_s;
    known_aircraft aircraft;

    /* Callback producers, declared last so they are destroyed FIRST: each joins
     * its worker/recv threads in its destructor, so those threads stop enqueuing
     * before the queue and locks above are torn down. Order within the group: mav
     * before smm (SMM holds a MAV&), and asset_name after fss (it is initialised
     * from fss->getAssetName()). aircraft is declared just above so it outlives
     * fss, whose position callback touches it. */
    std::unique_ptr<FSS> fss;
    std::unique_ptr<MAV> mav;
    std::unique_ptr<SMM> smm;
    std::string asset_name;
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
