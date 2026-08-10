#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fss/fmu-fss.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <variant>
#include <vector>

namespace
{
/* main() (src/main.cpp) ignores SIGPIPE so a send() to a peer that has closed
 * its end reports EPIPE instead of terminating the process. Catch2 supplies its
 * own main() here, so do the same at static-init time (mav_io_test.cpp does the
 * same for the same reason). */
struct IgnoreSigpipe
{
    IgnoreSigpipe () { std::signal (SIGPIPE, SIG_IGN); }
};
const IgnoreSigpipe ignore_sigpipe_once{};

/* The FSS under test is built from a path that does not exist. The client
 * library logs and swallows that (flight-safety-system client-ssl.cpp), leaving
 * a non-null client with an empty server list — so no socket is opened, no TLS
 * is negotiated, and the real sends would be silent no-ops even if the seams
 * below did not intercept them. */
constexpr const char *no_config = "/nonexistent/cap-fmu-fss-test-client.json";

constexpr auto io_timeout = std::chrono::seconds (8);
/* The bound a caller-side (event-loop) FSS call must return inside while the
 * worker is wedged. The gate below stays shut far longer than this, so a call
 * that waited on the worker would blow it. */
constexpr auto responsive_limit = std::chrono::seconds (2);

template <typename Pred>
auto
waitFor (Pred pred, std::chrono::milliseconds timeout) -> bool
{
    const auto deadline = std::chrono::steady_clock::now () + timeout;
    while (std::chrono::steady_clock::now () < deadline)
    {
        if (pred ())
        {
            return true;
        }
        std::this_thread::sleep_for (std::chrono::milliseconds (10));
    }
    return pred ();
}

auto
bounded (const std::function<void ()> &fn) -> bool
{
    auto start = std::chrono::steady_clock::now ();
    fn ();
    return (std::chrono::steady_clock::now () - start) < responsive_limit;
}
} // namespace

/* Test-only accessor for the friend seam in FSS. The task queue and its variant
 * are private, and deliberately so, so this summarises what is queued into plain
 * data the test can assert on rather than handing out the tasks themselves. */
struct FSSTestAccess
{
    struct QueueSummary
    {
        size_t positions{ 0 };
        size_t reached{ 0 };
        size_t battery{ 0 };
        size_t acks{ 0 };
        /* Latitude of the last queued position, to identify *which* report
         * survived coalescing. */
        double last_position_lat{ 0.0 };
        /* Kind of each queued task, in queue order, so a test can assert that
         * coalescing did not reorder the tasks it must not touch. */
        std::vector<std::string> order{};
    };

    static auto
    summarise (FSS &fss) -> QueueSummary
    {
        QueueSummary summary{};
        std::lock_guard<std::mutex> lk (fss.queue_lock);
        for (const auto &task : fss.task_queue)
        {
            if (const auto *p = std::get_if<FSS::FssPositionTask> (&task))
            {
                summary.positions++;
                summary.last_position_lat = p->lat;
                summary.order.emplace_back ("position");
            }
            else if (std::holds_alternative<FSS::FssReachedTask> (task))
            {
                summary.reached++;
                summary.order.emplace_back ("reached");
            }
            else if (std::holds_alternative<FSS::FssBatteryTask> (task))
            {
                summary.battery++;
                summary.order.emplace_back ("battery");
            }
            else
            {
                summary.acks++;
                summary.order.emplace_back ("ack");
            }
        }
        return summary;
    }
};

/* FSS test double exposing the send seams so a test can wedge the worker inside
 * one (block/release) and observe what it dispatched (the counters), with no
 * peer and no TLS. */
class TestFSS : public FSS
{
  public:
    using FSS::FSS;

    /* Release any wedged send and stop the worker here, while this subclass's
     * seams and the gate they park on are still alive -- ~FSS alone is too late,
     * since a derived object is destroyed derived-part-first.
     *
     * Doing it in the destructor rather than at the end of each test is also
     * what keeps a failed assertion a failure instead of a hang: Catch2's
     * REQUIRE throws to unwind the test body, which would skip any release
     * written as the last statement of the test. */
    ~TestFSS () override
    {
        this->block_sends = false;
        this->releaseSends ();
        this->stopWorker ();
    }

    std::atomic<int> position_calls{ 0 };
    std::atomic<int> reached_calls{ 0 };
    std::atomic<int> battery_calls{ 0 };
    std::atomic<int> ack_calls{ 0 };
    /* Set as each seam is entered, so a test can wait for the worker to be
     * genuinely parked rather than guessing at a delay. */
    std::atomic<bool> position_entered{ false };
    std::atomic<bool> ack_entered{ false };
    /* Latitude the last dispatched position carried, to confirm the coalesced
     * survivor is the newest report rather than the first. */
    std::atomic<double> last_position_lat{ 0.0 };
    std::atomic<bool> block_sends{ false };

    void
    releaseSends ()
    {
        {
            std::lock_guard<std::mutex> lk (this->gate_mtx);
            this->gate_open = true;
        }
        this->gate_cv.notify_all ();
    }

  protected:
    void
    sendPosition (const FssPositionTask &t) override
    {
        this->last_position_lat.store (t.lat);
        this->position_calls++;
        this->position_entered = true;
        this->gate ();
    }
    void
    sendReached (const FssReachedTask & /*t*/) override
    {
        this->reached_calls++;
        this->gate ();
    }
    void
    sendBattery (const FssBatteryTask & /*t*/) override
    {
        this->battery_calls++;
        this->gate ();
    }
    void
    sendAck (const FssAckTask &t) override
    {
        this->ack_calls++;
        this->ack_entered = true;
        this->gate ();
        /* Run the real responder too, so the ack path is exercised end to end
         * rather than only counted. */
        FSS::sendAck (t);
    }

  private:
    /* The gate never waits indefinitely. If a regression put a blocking send
     * back on the caller's thread, that thread would park here and never reach
     * the release -- so an unbounded wait would turn the very failure this
     * suite exists to catch into a hung CI job instead of a red one. The cap is
     * far above responsive_limit, so a healthy run never approaches it. */
    static constexpr auto gate_max_wait = std::chrono::seconds (10);

    std::mutex gate_mtx{};
    std::condition_variable gate_cv{};
    bool gate_open{ false };

    void
    gate ()
    {
        if (!this->block_sends.load ())
        {
            return;
        }
        std::unique_lock<std::mutex> lk (this->gate_mtx);
        this->gate_cv.wait_for (lk, gate_max_wait, [this] { return this->gate_open; });
    }
};

/* This is the case that still bites after the client library's outbound fan-out
 * rework: the position/reached/battery reports fan out through the (now
 * non-blocking) sendMsgAll, but the second-phase command ack is a
 * per-connection sendMsg that stays blocking. So it is the ack that the send
 * worker still has to keep off the event loop -- and it is the path that closes
 * the loop on an operator's rtl/terminate. */
TEST_CASE ("FSS callers stay responsive while the worker is wedged in a command-ack send", "[fss]")
{
    TestFSS fss{ no_config };
    fss.block_sends = true;

    std::atomic<int> responder_calls{ 0 };
    fss_command_ack_responder responder = [&responder_calls] (const FSSCommandResolution &) { responder_calls++; };

    /* Wedge the worker inside the ack send. */
    fss.postAck (responder, FSSCommandResolution{});
    REQUIRE (waitFor ([&] { return fss.ack_entered.load (); }, io_timeout));

    /* Every event-loop-facing FSS call must still return promptly. */
    REQUIRE (bounded ([&] { fss.reportPosition (PositionData (-43.5, 172.6, 50.0, 0, 0, 0)); }));
    REQUIRE (bounded ([&] { fss.reachedPoint (1, 4); }));
    REQUIRE (bounded ([&] { fss.reportBatteryStatus (BatteryData (80, 1200, 22.1)); }));
    REQUIRE (bounded ([&] { fss.postAck (responder, FSSCommandResolution{}); }));

    /* Release and drain before the object is destroyed: ~FSS joins a worker that
     * is calling this subclass's seams, so it must not still be parked on the
     * gate when ~TestFSS tears the gate down. */
    fss.block_sends = false;
    fss.releaseSends ();
    REQUIRE (waitFor ([&] { return responder_calls.load () >= 2; }, io_timeout));
}

TEST_CASE ("FSS::enqueue coalesces queued position reports and leaves every other task alone", "[fss]")
{
    TestFSS fss{ no_config };
    fss.block_sends = true;

    /* Wedge the worker on a throwaway report so nothing drains behind us. It is
     * popped before the seam blocks, so the queue is empty again here. */
    fss.reportPosition (PositionData (0.0, 0.0, 0.0, 0, 0, 0));
    REQUIRE (waitFor ([&] { return fss.position_entered.load (); }, io_timeout));

    std::atomic<int> responder_calls{ 0 };
    fss_command_ack_responder responder = [&responder_calls] (const FSSCommandResolution &) { responder_calls++; };

    fss.reportPosition (PositionData (-43.1, 172.1, 50.0, 0, 0, 0));
    fss.reachedPoint (1, 4);
    fss.reportPosition (PositionData (-43.2, 172.2, 51.0, 0, 0, 0));
    fss.reportBatteryStatus (BatteryData (80, 1200, 22.1));
    fss.postAck (responder, FSSCommandResolution{});
    fss.reportPosition (PositionData (-43.3, 172.3, 52.0, 0, 0, 0));

    auto summary = FSSTestAccess::summarise (fss);

    SECTION ("only the newest position survives")
    {
        REQUIRE (summary.positions == 1);
        REQUIRE (summary.last_position_lat == Catch::Approx (-43.3));
    }

    SECTION ("reached, battery and ack are never coalesced away")
    {
        REQUIRE (summary.reached == 1);
        REQUIRE (summary.battery == 1);
        REQUIRE (summary.acks == 1);
    }

    SECTION ("coalescing does not reorder the tasks it leaves behind")
    {
        /* The three superseded positions are gone and the survivor is appended
         * at the back, behind everything that was queued before it. */
        REQUIRE (summary.order == std::vector<std::string>{ "reached", "battery", "ack", "position" });
    }

    fss.block_sends = false;
    fss.releaseSends ();
    REQUIRE (waitFor ([&] { return responder_calls.load () >= 1; }, io_timeout));
}

TEST_CASE ("FSS drains the coalesced queue once the wedged send clears", "[fss]")
{
    TestFSS fss{ no_config };
    fss.block_sends = true;

    fss.reportPosition (PositionData (0.0, 0.0, 0.0, 0, 0, 0));
    REQUIRE (waitFor ([&] { return fss.position_entered.load (); }, io_timeout));

    std::atomic<int> responder_calls{ 0 };
    fss_command_ack_responder responder = [&responder_calls] (const FSSCommandResolution &) { responder_calls++; };

    fss.reportPosition (PositionData (-43.1, 172.1, 50.0, 0, 0, 0));
    fss.reportPosition (PositionData (-43.9, 172.9, 55.0, 0, 0, 0));
    fss.reachedPoint (2, 4);
    fss.reportBatteryStatus (BatteryData (75, 1300, 21.8));
    fss.postAck (responder, FSSCommandResolution{});

    fss.block_sends = false;
    fss.releaseSends ();

    /* Coalescing must not lose a non-position task: each of these went in once
     * and must come out once. The position went in three times (including the
     * throwaway that wedged the worker) and must be sent exactly twice -- the
     * one already in flight, plus the single coalesced survivor. */
    REQUIRE (waitFor ([&] { return fss.ack_calls.load () >= 1; }, io_timeout));
    REQUIRE (waitFor ([&] { return fss.reached_calls.load () >= 1; }, io_timeout));
    REQUIRE (waitFor ([&] { return fss.battery_calls.load () >= 1; }, io_timeout));
    REQUIRE (waitFor ([&] { return fss.position_calls.load () >= 2; }, io_timeout));

    REQUIRE (fss.position_calls.load () == 2);
    REQUIRE (fss.reached_calls.load () == 1);
    REQUIRE (fss.battery_calls.load () == 1);
    REQUIRE (fss.ack_calls.load () == 1);
    REQUIRE (responder_calls.load () == 1);
    REQUIRE (fss.last_position_lat.load () == Catch::Approx (-43.9));
}

/* Uses the real FSS, not TestFSS: the worker must be free to still be running
 * when the destructor starts, and a subclass is destroyed derived-part-first, so
 * parking the worker in a subclass seam across ~FSS would tear down the seam's
 * own members underneath it. */
TEST_CASE ("~FSS drains queued tasks, and a queued ack fires without a usable client", "[fss]")
{
    constexpr int queued_acks = 64;
    std::atomic<int> responder_calls{ 0 };
    fss_command_ack_responder responder = [&responder_calls] (const FSSCommandResolution &) { responder_calls++; };

    {
        FSS fss{ no_config };
        for (int i = 0; i < queued_acks; i++)
        {
            fss.postAck (responder, FSSCommandResolution{});
        }
        /* Destructor runs here: it clears worker_running, but workerLoop only
         * returns once the queue is *also* empty, so everything queued is still
         * dispatched. */
    }

    /* Nothing queued is dropped at shutdown. That these ran at all also covers
     * the ack being independent of ssl_client: this client was built from a
     * nonexistent config and has no servers, and the responder is a plain
     * closure the worker must run regardless. */
    REQUIRE (responder_calls.load () == queued_acks);
}

/* The comms-loss failsafe has to be armed from cold start, and the client
 * library cannot arm it: connectionStatusChange() fires only on a *change* --
 * from serverAdmitted() (the false->true admission edge), from
 * serverRequiresReconnect() (a live server dropping), and from
 * attemptReconnect() only once something has connected. fss_server's
 * constructor does not dial, so a client that has never reached a server
 * reports nothing at all, and the FMU would sit believing FSS was healthy for
 * as long as FSS stayed unreachable. registerCommsStatusCB() closes that by
 * reporting the failure itself, which is also the only point at which there is
 * a callback to report it to. */
TEST_CASE ("registering a comms status callback reports the initial FSS comms failure", "[fss]")
{
    FSS fss{ no_config };

    std::vector<FSSCommsStatus> reported;
    fss.registerCommsStatusCB ([&reported] (FSSCommsStatus status) { reported.push_back (status); });

    /* Synchronous, on the registering thread: in App::run() this lands in the
     * event queue before the reconnector that opens the first connection is
     * even started, so the FMU can never observe an unreported cold start. */
    REQUIRE (reported.size () == 1);
    REQUIRE (reported.at (0) == fss_comms_failure);
}
