#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <variant>

#include <secure-string.hpp>

#include "../ilogger.hpp"
#include "../mav/mav.hpp"
#include "ismm.hpp"
#include "smm-types.hpp"

extern "C"
{
#include <smm-asset.h>
};

/* SMM client.
 *
 * All blocking SMM HTTP I/O (connect/login, position report, search acquire and
 * accept) runs on a dedicated worker thread, never on the caller (event-loop)
 * thread: the public methods only enqueue a task and return immediately, so a
 * slow or hung SMM endpoint can never stall queued FSS commands (rtl/terminate)
 * (todo/33). Any resulting flight action is reported back through the
 * load-search / RTL callbacks, which the App routes through the event queue so
 * it is applied on the event-loop thread (where the state machine arbitrates
 * priority). The worker itself never commands MAV.
 *
 * See docs/threading.md for the full thread inventory, data ownership, and
 * lock-ordering model this worker/queue is one piece of. */
class SMM : public ISMM
{
    /* Test-only accessor: lets the resume/responsiveness tests inject held-search
     * / connected-asset state, since the only production path to it is HTTP. */
    friend struct SMMTestAccess;

  private:
    MAV &mav;
    ILogger &logger;
    smm_connection conn{ nullptr };
    std::string smm_host{};
    flight_safety_system::secure_string smm_user{};
    flight_safety_system::secure_string smm_pass{};
    std::string asset_name{};
    /* Local asset properties (from client.json), fixed at construction. */
    uint16_t altitude_cap{ 122 };
    uint16_t altitude_floor{ 10 };
    double camera_fov_deg{ 90.0 };
    /* Guards the fields the worker publishes and a test may inject — current_search
     * and asset. Held only briefly (never across an smm_asset_* call), so a
     * currentSearchPoints() read (which uses the atomic cache below) and test
     * injection never contend with the worker's blocking I/O. */
    std::mutex state_lock{};
    std::shared_ptr<SMMSearch> current_search{ nullptr };
    /* Lock-free mirror of current_search->getPointsCount() so the event loop can
     * read it (in the reached-point handler) without taking state_lock and thus
     * without ever waiting on the worker. Updated whenever current_search changes. */
    std::atomic<int> current_search_points{ 0 };
    smm_assets assets_list{ nullptr };
    size_t assets_list_count{ 0 };
    smm_asset asset{ nullptr };
    uint64_t position_report_last_ts{ 0 };
    /* Minimum gap between SMM position reports, milliseconds (from config). */
    uint64_t position_report_interval_ms{ 1000 };
    /* Connect / total-transfer timeouts applied to the SMM connection (seconds,
     * from config) so a slow or hung endpoint cannot block a call for the full
     * library-default TCP window. Applied after login in connect(). */
    long connect_timeout_s{ 5 };
    long transfer_timeout_s{ 10 };
    /* The searching role, granted by search() and revoked by cancelSearch() (both
     * on the event loop). Atomic so the worker can re-check it right before
     * committing a search: a command/latch that took over during the blocking
     * fetch clears it, aborting the accept so nothing is committed on the SMM
     * server when the FMU is no longer searching (todo/33). */
    std::atomic<bool> search_active{ false };
    uint64_t search_retry_ts{ 0 };
    static constexpr uint64_t search_retry_interval_ms{ 5000 };

    /* Flight-action callbacks invoked by the worker. The App wires these to the
     * event queue so the action is applied on the event-loop thread. */
    std::function<void (std::shared_ptr<SMMSearch>)> load_search_cb{};
    std::function<void ()> rtl_cb{};

    /* Worker thread + its task queue. The queue is a deque so reportPosition can
     * coalesce (drop a superseded queued report) and retryPendingSearch can
     * dedupe (at most one queued). */
    struct ConnectTask
    {
        std::string host{};
        flight_safety_system::secure_string user{};
        flight_safety_system::secure_string pass{};
        std::string asset_name{};
    };
    struct ReportPositionTask
    {
        PositionData pd{};
    };
    struct ReachedPointTask
    {
        int point{ 0 };
    };
    struct SearchTask
    {
        Point pos{};
    };
    struct RetryPendingTask
    {
        Point pos{};
    };
    using SmmTask = std::variant<ConnectTask, ReportPositionTask, ReachedPointTask, SearchTask, RetryPendingTask>;

    std::thread worker_thread{};
    std::mutex queue_lock{};
    std::condition_variable queue_cv{};
    std::deque<SmmTask> task_queue{};
    bool worker_running{ true };

    void enqueue (SmmTask task);
    void workerLoop ();
    /* Worker-thread task handlers (blocking smm_asset_* I/O lives here). */
    void doConnect (const ConnectTask &t);
    void doReportPosition (PositionData t_pd);
    void doReachedPoint (int point);
    void doSearch (Point current_pos);
    /* Acquire a search only if one is wanted (search_active) and none is held —
     * the shared guard behind both retry triggers (fresh position / reconnect
     * timer). */
    void maybeAcquire (Point current_pos);
    void tryAcquireSearch (Point current_pos);
    void publishSearch (std::shared_ptr<SMMSearch> acquired);
    auto hasHeldSearch () -> bool;

    void connect ();
    void disconnect ();

  protected:
    /* Seams over the blocking SMM library calls so a test can inject a slow or
     * observable response. Defaults call straight through to the library. */
    virtual auto fetchSearch (double lat, double lon) -> smm_search;
    virtual auto commitSearch (SMMSearch &candidate) -> bool;
    virtual void reportPositionToSmm (double lat, double lon, int32_t alt, int heading);

  public:
    /* The tuning parameters (report interval, connect/transfer timeouts) have no
     * defaults here: each value lives once in FmuConfig (its default) and every
     * caller passes it through, so there is no duplicated literal to drift. */
    SMM (MAV &t_mav, ILogger &t_logger, uint16_t t_altitude_cap, uint16_t t_altitude_floor, double t_camera_fov_deg,
         uint64_t t_position_report_interval_ms, long t_connect_timeout_s, long t_transfer_timeout_s);
    SMM (SMM &) = delete;
    SMM (SMM &&) = delete;
    auto operator= (SMM &) -> SMM & = delete;
    auto operator= (SMM &&) -> SMM & = delete;
    ~SMM () override;
    /* Register the flight-action callbacks: load an acquired/resumed search,
     * or report SMM has nothing to search right now -- either a failed
     * acquire attempt (tryAcquireSearch) or a held search's last waypoint
     * completing with none queued behind it (doReachedPoint, todo/70). Call
     * once before SMM activity begins. */
    void registerLoadSearchCB (std::function<void (std::shared_ptr<SMMSearch>)> cb);
    void registerRtlCB (std::function<void ()> cb);
    void connect (const std::string &host, const flight_safety_system::secure_string &user,
                  const flight_safety_system::secure_string &pass, const std::string &asset_name) override;
    void search (Point current_pos) override;
    void cancelSearch () override;
    /* Timer-driven retry of a pending (active but not yet acquired) search.
     * Called periodically off the reconnect thread so acquisition is not
     * coupled solely to MAV position-report cadence (see todo/41). The current
     * MAV position is read here, on the caller (reconnect) thread, and carried to
     * the worker so the worker never touches MAV. */
    void retryPendingSearch ();
    void reportPosition (PositionData t_pd) override;
    void reachedPoint (int point) override;
    auto currentSearchPoints () -> int override;
};
