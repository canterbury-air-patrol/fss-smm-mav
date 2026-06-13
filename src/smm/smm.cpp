#include <cmath>
#include <cstring>
#include <iostream>

#include "smm.hpp"
#include "util.hpp"
#include <smm-asset.h>

SMM::SMM (MAV &t_mav) : mav (t_mav)
{
    //    smm_asset_debugging_set (true);
}

SMM::~SMM ()
{
    std::lock_guard<std::mutex> lk (this->search_lock);
    this->disconnect ();
    this->current_search = nullptr;
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
    std::string user_cstr (this->smm_user.data (), this->smm_user.size ());
    std::string pass_cstr (this->smm_pass.data (), this->smm_pass.size ());
    this->conn = smm_asset_connect (this->smm_host.c_str (), user_cstr.c_str (), pass_cstr.c_str ());

    /* smm_asset_connect() only validates the host; the library authenticates
     * lazily on the first request and reports SMM_CONNECTION_NEW until then.
     * Log in eagerly so the state check below reflects the real outcome. */
    smm_asset_connection_login (this->conn);

    if (smm_asset_connection_get_state (this->conn) != SMM_CONNECTION_CONNECTED)
    {
        /* Oh dear */
        std::cout << "SMM: Connection failed (" << smm_asset_connection_get_state (this->conn) << ")" << '\n';
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
            std::cout << "SMM: Failed to find this asset" << '\n';
            this->disconnect ();
            return;
        }
    }
    else
    {
        std::cout << "SMM: Failed to get assets" << '\n';
        this->disconnect ();
    }
}

void
SMM::connect (const std::string &t_host, const flight_safety_system::secure_string &t_user,
              const flight_safety_system::secure_string &t_pass, const std::string &t_asset_name)
{
    std::lock_guard<std::mutex> lk (this->search_lock);
    if (this->conn != nullptr)
    {
        /* If the details have changed, or the connection has failed, disconnect */
        if (this->smm_host != t_host || this->smm_user != t_user || this->smm_pass != t_pass
            || this->asset_name != t_asset_name
            || smm_asset_connection_get_state (this->conn) != SMM_CONNECTION_CONNECTED)
        {
            std::cout << "SMM: Details have changed" << '\n';
            this->disconnect ();
        }
    }
    /* If there is no connection, store the details and connect */
    if (this->conn == nullptr)
    {
        this->smm_host = t_host;
        this->smm_user = t_user;
        this->smm_pass = t_pass;
        this->asset_name = t_asset_name;

        std::cout << "SMM: Connecting (" << this->smm_host << "," << this->asset_name << ")" << '\n';
        this->connect ();
    }
}

void
SMM::reportPosition (PositionData t_pd)
{
    std::lock_guard<std::mutex> lk (this->search_lock);
    if (this->asset)
    {
        uint64_t curr_ts = current_timestamp_ms ();
        if (this->position_report_last_ts + 1000 <= curr_ts)
        {
            Point p = t_pd.getP ();
            smm_asset_report_position (this->asset, p.getLatitude (), p.getLongitude (),
                                       static_cast<int32_t> (std::lround (t_pd.getAltitude ())),
                                       t_pd.getHeading () / 100, 3);
            this->position_report_last_ts = curr_ts;
        }
    }
    if (this->search_active && this->current_search == nullptr)
    {
        this->tryAcquireSearch (t_pd.getP ());
    }
}

/* Called with search_lock held. Tries to acquire a search from SMM.
 * On failure, sets a retry timestamp so periodic calls back off. */
void
SMM::tryAcquireSearch (Point current_pos)
{
    uint64_t curr_ts = current_timestamp_ms ();
    if (this->search_retry_ts > curr_ts)
    {
        return;
    }
    int retries = 0;
    while (this->current_search == nullptr)
    {
        auto new_search = smm_asset_get_search (this->asset, current_pos.getLatitude (), current_pos.getLongitude ());
        if (new_search == nullptr)
        {
            this->mav.setMode (flight_mode_rtl);
            this->search_retry_ts = current_timestamp_ms () + search_retry_interval_ms;
            return;
        }
        /* Fetch the waypoints (and validate them) before accepting, so an
         * un-loadable search is never committed to on the server. */
        auto candidate = std::make_shared<SMMSearch> (new_search);
        if (candidate->isValid () && candidate->accept ())
        {
            this->current_search = candidate;
            break;
        }
        /* candidate's destructor destroys the search; it was not accepted. */
        if (++retries >= 3)
        {
            this->mav.setMode (flight_mode_rtl);
            this->search_retry_ts = current_timestamp_ms () + search_retry_interval_ms;
            return;
        }
    }
    this->mav.loadSearch (this->current_search);
    this->search_retry_ts = 0;
}

void
SMM::search (Point current_pos)
{
    std::lock_guard<std::mutex> lk (this->search_lock);
    if (this->asset == nullptr)
    {
        this->mav.setMode (flight_mode_rtl);
        return;
    }
    this->search_active = true;
    if (this->current_search != nullptr)
    {
        return;
    }
    this->tryAcquireSearch (current_pos);
}

void
SMM::cancelSearch ()
{
    std::lock_guard<std::mutex> lk (this->search_lock);
    this->search_active = false;
    this->search_retry_ts = 0;
}

void
SMM::reachedPoint (int point)
{
    /* See if we have completed this search or not */
    std::lock_guard<std::mutex> lk (this->search_lock);
    if (this->current_search != nullptr)
    {
        if (this->current_search->reachedPoint (point))
        {
            this->current_search = nullptr;
        }
    }
}

auto
SMM::currentSearchPoints () -> int
{
    std::lock_guard<std::mutex> lk (this->search_lock);
    if (this->current_search != nullptr)
    {
        return this->current_search->getPointsCount ();
    }
    return 0;
}

SMMSearch::SMMSearch (smm_search t_search)
{
    this->search = t_search;
    smm_waypoints wps = nullptr;
    size_t wps_count = 0;
    /* A failed fetch (e.g. transient network error) leaves the search with no
     * points; only treat it as valid once we have at least one waypoint. */
    if (smm_search_get_waypoints (this->search, &wps, &wps_count) && wps_count > 0)
    {
        for (size_t i = 0; i < wps_count; i++)
        {
            Point wp (wps[i]->lat, wps[i]->lon);
            this->addPoint (wp);
        }
        this->valid = true;
    }
    smm_waypoints_free (wps, wps_count);
    this->altitude = static_cast<int> (smm_search_sweep_width (search));
}

auto
SMMSearch::accept () -> bool
{
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
