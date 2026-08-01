#include <algorithm>
#include <cmath>
#include <cstring>
#include <strings.h>
#include <utility>
#include <vector>

#include "connection-state.hpp"
#include "search-acquire.hpp"
#include "search-altitude.hpp"
#include "smm.hpp"
#include "util.hpp"
#include <smm-asset.h>

namespace
{
template <class... Ts> struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts> overloaded (Ts...) -> overloaded<Ts...>;
} // namespace

SMM::SMM (MAV &t_mav, ILogger &t_logger, uint16_t t_altitude_cap, uint16_t t_altitude_floor, double t_camera_fov_deg,
          uint64_t t_position_report_interval_ms, long t_connect_timeout_s, long t_transfer_timeout_s)
    : mav (t_mav), logger (t_logger), altitude_cap (t_altitude_cap), altitude_floor (t_altitude_floor),
      camera_fov_deg (t_camera_fov_deg), position_report_interval_ms (t_position_report_interval_ms),
      connect_timeout_s (t_connect_timeout_s), transfer_timeout_s (t_transfer_timeout_s)
{
    //    smm_asset_debugging_set (true);
    /* Start the worker now: no tasks are enqueued until the App wires up events,
     * so the worker simply waits on an empty queue until then. */
    this->worker_thread = std::thread (&SMM::workerLoop, this);
}

SMM::~SMM ()
{
    /* Stop and join the worker before tearing down the connection: once joined,
     * no thread can touch conn / current_search, so the disconnect is race-free
     * and no in-flight outcome callback can fire into a half-destroyed App. */
    {
        std::lock_guard<std::mutex> lk (this->queue_lock);
        this->worker_running = false;
    }
    this->queue_cv.notify_one ();
    if (this->worker_thread.joinable ())
    {
        this->worker_thread.join ();
    }
    this->disconnect ();
    this->current_search = nullptr;
}

void
SMM::registerLoadSearchCB (std::function<void (std::shared_ptr<SMMSearch>)> cb)
{
    this->load_search_cb = std::move (cb);
}

void
SMM::registerRtlCB (std::function<void ()> cb)
{
    this->rtl_cb = std::move (cb);
}

void
SMM::registerOperatorCommandCB (std::function<void (SMMCommand)> cb)
{
    this->operator_command_cb = std::move (cb);
}

auto
SMM::fetchSearch (double lat, double lon) -> smm_search
{
    return smm_asset_get_search (this->asset, lat, lon);
}

auto
SMM::commitSearch (SMMSearch &candidate) -> bool
{
    return candidate.accept ();
}

void
SMM::reportPositionToSmm (double lat, double lon, int32_t alt, int heading)
{
    smm_asset_report_position (this->asset, lat, lon, alt, heading, 3);
}

auto
SMM::lastOperatorCommand () -> smm_asset_command
{
    return smm_asset_last_command (this->asset);
}

void
SMM::enqueue (SmmTask task)
{
    {
        std::lock_guard<std::mutex> lk (this->queue_lock);
        if (std::holds_alternative<ReportPositionTask> (task))
        {
            /* Coalesce: a newer position supersedes any queued (unsent) report, so
             * a backlog cannot build while the worker is in a slow call. Reporting
             * the latest position is all that matters. */
            for (auto it = this->task_queue.begin (); it != this->task_queue.end ();)
            {
                it = std::holds_alternative<ReportPositionTask> (*it) ? this->task_queue.erase (it) : std::next (it);
            }
        }
        else if (std::holds_alternative<RetryPendingTask> (task))
        {
            /* Dedupe: at most one pending-search retry queued — they are idempotent
             * and the backoff timestamp rate-limits the actual fetch anyway. */
            if (std::any_of (this->task_queue.begin (), this->task_queue.end (),
                             [] (const SmmTask &queued) { return std::holds_alternative<RetryPendingTask> (queued); }))
            {
                return;
            }
        }
        this->task_queue.push_back (std::move (task));
    }
    this->queue_cv.notify_one ();
}

void
SMM::workerLoop ()
{
    while (true)
    {
        SmmTask task;
        {
            std::unique_lock<std::mutex> lk (this->queue_lock);
            this->queue_cv.wait (lk, [this] { return !this->task_queue.empty () || !this->worker_running; });
            if (!this->worker_running && this->task_queue.empty ())
            {
                return;
            }
            task = std::move (this->task_queue.front ());
            this->task_queue.pop_front ();
        }
        std::visit (
            overloaded{
                [this] (const ConnectTask &t) { this->doConnect (t); },
                [this] (const ReportPositionTask &t) { this->doReportPosition (t.pd); },
                [this] (const ReachedPointTask &t) { this->doReachedPoint (t.point); },
                [this] (const SearchTask &t) { this->doSearch (t.pos); },
                [this] (const RetryPendingTask &t) { this->maybeAcquire (t.pos); },
            },
            task);
    }
}

void
SMM::disconnect ()
{
    if (this->assets_list != nullptr)
    {
        this->asset = nullptr;
        smm_asset_free_assets (this->assets_list, this->assets_list_count);
        this->assets_list = nullptr;
        this->assets_list_count = 0;
    }
    if (this->conn != nullptr)
    {
        smm_connection_close (this->conn);
        this->conn = nullptr;
    }
}

void
SMM::connect ()
{
    /* smm_asset_connect() needs NUL-terminated C strings, but
     * secure_string::data() is not NUL-terminated (it is an exact-length
     * byte buffer, see secure-string.hpp) and std::string does not scrub its
     * buffer on destruction (and may reallocate) — either alone would leave
     * the credentials in non-zeroed heap memory. Build local NUL-terminated
     * buffers (zero-initialised, so the trailing byte is already '\0') and
     * scrub them explicitly once the call returns. */
    std::vector<char> user_cstr (this->smm_user.size () + 1, '\0');
    std::memcpy (user_cstr.data (), this->smm_user.data (), this->smm_user.size ());
    std::vector<char> pass_cstr (this->smm_pass.size () + 1, '\0');
    std::memcpy (pass_cstr.data (), this->smm_pass.data (), this->smm_pass.size ());
    this->conn = smm_asset_connect (this->smm_host.c_str (), user_cstr.data (), pass_cstr.data ());
    explicit_bzero (user_cstr.data (), user_cstr.size ());
    explicit_bzero (pass_cstr.data (), pass_cstr.size ());
    if (this->conn == nullptr)
    {
        this->logger.log (LogLevel::error, "SMM: Connection failed (no connection)");
        return;
    }

    /* Bound how long any single SMM request can block. The library defaults
     * (30s connect / 60s transfer) are far too long for a flight-safety loop:
     * SMM I/O runs on the worker thread (public methods enqueue and return, so
     * queued FSS commands are never blocked on SMM HTTP — todo/33), but an
     * unbounded call would still pin the worker for the full TCP window, delaying
     * shutdown, the search-acquire retry, and any SMM work queued behind it. Set
     * before the login below so even the login is bounded. */
    smm_asset_connection_timeouts_set (this->conn, this->connect_timeout_s, this->transfer_timeout_s);

    /* smm_asset_connect() only validates the host; the library authenticates
     * lazily on the first request and reports SMM_CONNECTION_NEW until then.
     * Log in eagerly so the state check below reflects the real outcome. */
    smm_asset_connection_login (this->conn);

    if (!smm_connection_is_connected (this->conn))
    {
        /* Oh dear */
        this->logger.log (LogLevel::error, "SMM: Connection failed ("
                                               + std::to_string (smm_asset_connection_get_state (this->conn)) + ")");
        this->disconnect ();
        return;
    }

    /* Find our asset */
    if (smm_asset_get_assets (this->conn, &this->assets_list, &this->assets_list_count))
    {
        for (size_t i = 0; i < this->assets_list_count; i++)
        {
            if (strcmp (smm_asset_name (this->assets_list[i]), this->asset_name.c_str ()) == 0)
            {
                this->asset = this->assets_list[i];
                break;
            }
        }
        if (this->asset == nullptr)
        {
            this->logger.log (LogLevel::error, "SMM: Failed to find this asset");
            this->disconnect ();
            return;
        }
    }
    else
    {
        this->logger.log (LogLevel::error, "SMM: Failed to get assets");
        this->disconnect ();
    }
}

void
SMM::connect (const std::string &t_host, const flight_safety_system::secure_string &t_user,
              const flight_safety_system::secure_string &t_pass, const std::string &t_asset_name)
{
    /* Connect/login/get-assets all block on the network, so run them on the
     * worker rather than the event-loop thread that called us. */
    this->enqueue (ConnectTask{ t_host, t_user, t_pass, t_asset_name });
}

void
SMM::doConnect (const ConnectTask &t)
{
    if (this->conn != nullptr)
    {
        /* If the details have changed, or the connection has failed, disconnect */
        if (this->smm_host != t.host || this->smm_user != t.user || this->smm_pass != t.pass
            || this->asset_name != t.asset_name
            || smm_asset_connection_get_state (this->conn) != SMM_CONNECTION_CONNECTED)
        {
            this->logger.log (LogLevel::info, "SMM: Details have changed");
            this->disconnect ();
        }
    }
    /* If there is no connection, store the details and connect */
    if (this->conn == nullptr)
    {
        this->smm_host = t.host;
        this->smm_user = t.user;
        this->smm_pass = t.pass;
        this->asset_name = t.asset_name;

        this->logger.log (LogLevel::info, "SMM: Connecting (" + this->smm_host + "," + this->asset_name + ")");
        this->connect ();
    }
}

void
SMM::reportPosition (PositionData t_pd)
{
    /* Position reporting is unconditional (it keeps SMM updated during RTL/hold/
     * terminate too) but it is an HTTP call, so it runs on the worker. */
    this->enqueue (ReportPositionTask{ std::move (t_pd) });
}

void
SMM::doReportPosition (PositionData t_pd)
{
    if (this->asset)
    {
        uint64_t curr_ts = current_timestamp_ms ();
        if (this->position_report_last_ts + this->position_report_interval_ms <= curr_ts)
        {
            /* smm_asset_last_command() reads a value cached on the asset from
             * the last position-report response (the library doc comment: "is
             * set in response to a position report; normally this is checked
             * after smm_asset_report_position"), so checking the cached value
             * from the PREVIOUS report here -- rather than after the call
             * below -- costs at most one report interval of extra latency
             * (operator commands are latched server-side and are not
             * sub-second time-critical), in exchange for keeping this the
             * only virtual call in this block before the one report_calls-style
             * test doubles (mav_io_test.cpp's TestSMM) synchronise on: putting
             * it after would leave a virtual dispatch racing a test's
             * destructor the instant reportPositionToSmm() returns (todo/90). */
            this->checkOperatorCommand ();

            Point p = t_pd.getP ();
            /* SMM expects altitude in metres, which is also PositionData's unit,
             * so it is forwarded directly. std::lround on a non-finite double is
             * undefined; a real altitude in metres is always well within int32_t
             * range, so a finiteness guard (reporting 0 otherwise) is enough. */
            double alt_m = t_pd.getAltitudeMetres ();
            int32_t alt = std::isfinite (alt_m) ? static_cast<int32_t> (std::lround (alt_m)) : 0;
            this->reportPositionToSmm (p.getLatitude (), p.getLongitude (), alt, t_pd.getHeading () / 100);
            this->position_report_last_ts = curr_ts;
        }
    }
    /* Opportunistic retry: a fresh position arrived, so use it to (re)attempt
     * acquisition. This is one of two retry triggers; retryPendingSearch() drives
     * the other off the reconnect timer so a search is still retried when position
     * reports stop (see todo/41). checkOperatorCommand() above may just have
     * dropped a held search (abandon-search), so this also serves as its
     * immediate reacquire attempt. */
    this->maybeAcquire (t_pd.getP ());
}

void
SMM::checkOperatorCommand ()
{
    smm_asset_command cmd = this->lastOperatorCommand ();
    if (cmd == this->last_seen_operator_command)
    {
        /* Not a new command: smm_asset_last_command() keeps reporting the same
         * value on every report until the operator issues a different one, so
         * without this guard an abandon-search would re-fire (and re-drop a
         * freshly (re)acquired search) once per second for as long as it
         * remains the server's last recorded command. */
        return;
    }
    this->last_seen_operator_command = cmd;
    switch (cmd)
    {
        case SMM_COMMAND_ABANDON_SEARCH:
            /* Drop the held search without completing it server-side (an
             * explicit smm_search_complete would mark it finished, which an
             * abandon does not mean) -- maybeAcquire(), called right after this
             * returns in doReportPosition(), picks up whatever SMM offers next
             * for this asset. Reset the retry backoff too, so this is retried
             * immediately rather than waiting out a stale failed-acquire
             * backoff. */
            this->publishSearch (nullptr);
            this->search_retry_ts = 0;
            if (this->operator_command_cb)
            {
                this->operator_command_cb (smm_cmd_abandon_search);
            }
            break;
        case SMM_COMMAND_MISSION_COMPLETE:
            if (this->operator_command_cb)
            {
                this->operator_command_cb (smm_cmd_mission_complete);
            }
            break;
        default:
            /* NONE, and every command outside this pass's scope (circle/rtl/
             * goto/continue/unknown -- see smm-command.hpp), is a no-op. */
            break;
    }
}

void
SMM::retryPendingSearch ()
{
    /* Timer-driven counterpart to the opportunistic retry in reportPosition().
     * Driven from the reconnect thread, it re-attempts acquisition of a search
     * that is active but not yet acquired even when MAV position reports have
     * stopped (a GPS/telemetry gap) — without it, an incoming position report was
     * the only retry trigger, so acquisition (or the disconnected-RTL fallback in
     * tryAcquireSearch) could stall indefinitely after an SMM reconnect. The last
     * known MAV position is read here, on the caller (reconnect) thread, and
     * carried to the worker so the worker never touches MAV. The search_retry_ts
     * backoff still rate-limits the actual fetches. */
    this->enqueue (RetryPendingTask{ this->mav.getCurrentPosition () });
}

auto
SMM::hasHeldSearch () -> bool
{
    std::lock_guard<std::mutex> lk (this->state_lock);
    return this->current_search != nullptr;
}

void
SMM::maybeAcquire (Point current_pos)
{
    if (this->search_active.load () && !this->hasHeldSearch ())
    {
        this->tryAcquireSearch (current_pos);
    }
}

/* Publishes an acquired search as the held search and updates the lock-free
 * point-count cache the event loop reads. Worker-only. */
void
SMM::publishSearch (std::shared_ptr<SMMSearch> acquired)
{
    int points = acquired != nullptr ? acquired->getPointsCount () : 0;
    {
        std::lock_guard<std::mutex> lk (this->state_lock);
        this->current_search = std::move (acquired);
    }
    this->current_search_points.store (points);
}

/* Tries to acquire a search from SMM. On failure, sets a retry timestamp so
 * periodic calls back off. Runs entirely on the worker thread, reached from both
 * retry triggers — doReportPosition() (on a fresh position) and the reconnect
 * timer (RetryPendingTask) — only when a search is wanted and none is held.
 *
 * The blocking fetch runs lock-free (the worker is the sole mutator of the
 * acquire state); a resulting flight action is reported via the rtl_cb /
 * load_search_cb callbacks, which the App applies on the event loop only while
 * still searching. search_active is re-checked right before the committing
 * accept: a command/latch that revoked the searching role during the fetch
 * aborts the accept, so nothing is committed on the SMM server when the FMU is no
 * longer searching (todo/33). */
void
SMM::tryAcquireSearch (Point current_pos)
{
    uint64_t curr_ts = current_timestamp_ms ();
    switch (search_acquire_action (this->asset != nullptr, this->search_retry_ts, curr_ts))
    {
        case SearchAcquireAction::backoff:
            return;
        case SearchAcquireAction::disconnected_rtl:
            /* SMM disconnected or asset discovery failed while a search is still
             * active. smm_asset_get_search() must never run with a null asset, so
             * fall back to a safe RTL and back off; search_active stays set, so the
             * search is retried once the asset is rediscovered after reconnect. */
            if (this->rtl_cb)
            {
                this->rtl_cb ();
            }
            this->search_retry_ts = curr_ts + search_retry_interval_ms;
            return;
        case SearchAcquireAction::fetch:
            break;
    }
    int retries = 0;
    while (true)
    {
        auto new_search = this->fetchSearch (current_pos.getLatitude (), current_pos.getLongitude ());

        /* If the searching role was revoked while we were fetching, clean up and
         * exit before committing anything on the server. */
        if (!this->search_active.load ())
        {
            if (new_search != nullptr)
            {
                smm_search_destroy (new_search);
            }
            return;
        }

        if (new_search == nullptr)
        {
            if (this->rtl_cb)
            {
                this->rtl_cb ();
            }
            this->search_retry_ts = current_timestamp_ms () + search_retry_interval_ms;
            return;
        }
        /* Fetch the waypoints (and validate them) before accepting, so an
         * un-loadable search is never committed to on the server. */
        auto candidate = std::make_shared<SMMSearch> (new_search, this->altitude_cap, this->altitude_floor,
                                                      this->camera_fov_deg, this->logger);
        /* Re-check search_active immediately before accept: this is the
         * load-bearing guard — accept() commits on the SMM server and the
         * event-loop guard cannot undo it. */
        if (this->search_active.load () && candidate->isValid () && this->commitSearch (*candidate))
        {
            this->publishSearch (candidate);
            if (this->load_search_cb)
            {
                this->load_search_cb (candidate);
            }
            this->search_retry_ts = 0;
            return;
        }
        /* candidate's destructor destroys the search; it was not accepted. If the
         * role was revoked, stop without an RTL fallback. */
        if (!this->search_active.load ())
        {
            return;
        }
        if (++retries >= 3)
        {
            if (this->rtl_cb)
            {
                this->rtl_cb ();
            }
            this->search_retry_ts = current_timestamp_ms () + search_retry_interval_ms;
            return;
        }
    }
}

void
SMM::search (Point current_pos)
{
    /* Grant the searching role synchronously (so an in-flight worker acquire that
     * was about to abort sees it set), then do the acquire/resume on the worker. */
    this->search_active.store (true);
    this->enqueue (SearchTask{ current_pos });
}

void
SMM::doSearch (Point current_pos)
{
    /* A user-initiated search must not inherit a prior failed-acquire backoff. */
    this->search_retry_ts = 0;
    auto held = std::shared_ptr<SMMSearch>{};
    {
        std::lock_guard<std::mutex> lk (this->state_lock);
        held = this->current_search;
    }
    if (held != nullptr)
    {
        /* We already hold a search that was paused by an interrupting command
         * (hold/rtl, which cancelSearch()'d without dropping current_search and
         * cleared search_loaded on the MAV side). Re-issue it so `continue`
         * resumes it from the last point: loadSearch re-uploads and jumps to the
         * current point when the mission is no longer loaded, and is a no-op if it
         * still is. */
        if (this->load_search_cb)
        {
            this->load_search_cb (held);
        }
        return;
    }
    /* tryAcquireSearch() guards the null-asset case itself (RTL + back off, then
     * retry after reconnect), so the disconnected path no longer needs a special
     * case here and both entry points share one rule. */
    this->tryAcquireSearch (current_pos);
}

void
SMM::cancelSearch ()
{
    /* Revoke the searching role synchronously (no I/O): this is the arbitration
     * token the worker re-checks before committing a search, so a command/latch
     * that fires during a blocking acquire aborts the accept. Pause, not abandon:
     * current_search is deliberately retained so a later `continue` resumes it
     * from the last reached point. The search is only truly dropped on completion
     * (reachedPoint) or a failed (re)acquire. */
    this->search_active.store (false);
}

void
SMM::reachedPoint (int point)
{
    this->enqueue (ReachedPointTask{ point });
}

void
SMM::doReachedPoint (int point)
{
    /* See if we have completed this search or not */
    auto held = std::shared_ptr<SMMSearch>{};
    {
        std::lock_guard<std::mutex> lk (this->state_lock);
        held = this->current_search;
    }
    if (held != nullptr && held->reachedPoint (point))
    {
        this->publishSearch (nullptr);
        /* Nothing is held now, so the FMU has nothing to search: report the
         * same outcome as a failed (re)acquire (todo/70) so the state
         * machine moves to waiting-for-tasking immediately, rather than only
         * once the next opportunistic acquire attempt (still retried, since
         * search_active is deliberately left untouched here) eventually
         * fails. Covered end-to-end by the CAP Tier-3 suite (test_b06,
         * todo/77); SMMSearch::reachedPoint()'s completion branch cannot be
         * driven from a unit test without a live SMM search handle. */
        if (this->rtl_cb)
        {
            this->rtl_cb ();
        }
    }
}

auto
SMM::currentSearchPoints () -> int
{
    /* Lock-free read of the cached count so the event loop never waits on the
     * worker (which may be mid-HTTP-call). */
    return this->current_search_points.load ();
}

SMMSearch::SMMSearch (smm_search t_search, uint16_t altitude_cap, uint16_t altitude_floor, double camera_fov_deg,
                      ILogger &logger)
{
    this->search = t_search;
    smm_waypoints wps = nullptr;
    size_t wps_count = 0;
    /* A failed fetch (e.g. transient network error) leaves the search with no
     * points; only treat it as valid once we have at least one waypoint, and
     * only if every one of them is a valid coordinate (todo/85) -- one bad
     * waypoint invalidates the whole candidate rather than uploading a
     * partially-corrupt mission, and flows through the same failed-acquire
     * retry/RTL path in tryAcquireSearch() as a fetch failure. */
    if (smm_search_get_waypoints (this->search, &wps, &wps_count) && wps_count > 0)
    {
        ParsedWaypoints parsed = parse_waypoints (wps, wps_count);
        for (const auto &wp : parsed.points)
        {
            this->addPoint (wp);
        }
        this->valid = parsed.all_valid;
        if (!parsed.all_valid)
        {
            logger.log (LogLevel::warning, "SMM search has an invalid waypoint (non-finite or out-of-range "
                                           "coordinate); discarding the candidate rather than loading it");
        }
    }
    smm_waypoints_free (wps, wps_count);
    /* Derive the flight altitude from the search's sweep (lane) width and the
     * camera geometry, then clamp into [floor, cap]. The pure helpers live in
     * search-altitude.hpp so the formula and clamp are unit tested directly;
     * here we keep the warning logging. */
    double sweep_width = static_cast<double> (smm_search_sweep_width (search));
    double raw_altitude = raw_search_altitude (sweep_width, camera_fov_deg);
    this->altitude = clamp_search_altitude (raw_altitude, altitude_floor, altitude_cap);
    if (raw_altitude > altitude_cap)
    {
        logger.log (LogLevel::warning, "SMM: Derived altitude (" + std::to_string (raw_altitude) + "m) for sweep width "
                                           + std::to_string (sweep_width) + "m exceeds altitude cap ("
                                           + std::to_string (altitude_cap) + "m), clamping altitude");
    }
    else if (raw_altitude < altitude_floor)
    {
        logger.log (LogLevel::warning, "SMM: Derived altitude (" + std::to_string (raw_altitude) + "m) for sweep width "
                                           + std::to_string (sweep_width) + "m below altitude floor ("
                                           + std::to_string (altitude_floor) + "m), clamping altitude");
    }
}

auto
SMMSearch::accept () -> bool
{
    /* accept() must only be called on a valid search; guard against misuse so
     * we never commit an empty search to the server. */
    if (!this->isValid ())
    {
        return false;
    }
    return smm_search_accept (this->search);
}

auto
SMMSearch::getPointsCount () -> int
{
    return static_cast<int> (this->points.size ());
}

auto
SMMSearch::reachedPoint (int point) -> bool
{
    if (this->search != nullptr)
    {
        if (point >= this->getPointsCount ())
        {
            /* Search completed, yay */
            smm_search_complete (this->search);
            smm_search_destroy (this->search);
            this->search = nullptr;
            return true;
        }
    }
    this->current_point = point;
    return false;
}

SMMSearch::~SMMSearch ()
{
    if (this->search != nullptr)
    {
        smm_search_destroy (this->search);
        this->search = nullptr;
    }
}
