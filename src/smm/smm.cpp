#include <cstring>
#include <iostream>

#include "smm.hpp"
#include "util.hpp"
#include <smm-asset.h>

SMM::SMM(std::shared_ptr<MAV> t_mav) : mav(std::move(t_mav))
{
//    smm_asset_debugging_set (true);
}

SMM::~SMM()
{
    this->disconnect();
    this->search_lock.lock();
    if (this->current_search)
    {
        this->current_search = nullptr;
    }
    this->search_lock.unlock();
}

void
SMM::disconnect()
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
SMM::connect()
{
    this->conn = smm_asset_connect (this->smm_host.c_str(), this->smm_user.c_str(), this->smm_pass.c_str());

    if (smm_asset_connection_get_state (this->conn) != SMM_CONNECTION_CONNECTED)
    {
        /* Oh dear */
        std::cout << "SMM: Connection failed (" << smm_asset_connection_get_state(this->conn) << ")" << std::endl;
        this->disconnect();
        return;
    }

    /* Find our asset */
    if (smm_asset_get_assets (this->conn, &this->assets_list, &this->assets_list_count))
    {
        for (size_t i = 0; i < this->assets_list_count; i++)
        {
            if (strcmp (smm_asset_name (this->assets_list[i]), this->asset_name.c_str()) == 0)
            {
                this->asset = this->assets_list[i];
                break;
            }
        }
        if (this->asset == nullptr)
        {
            std::cout << "SMM: Failed to find this asset" << std::endl;
            this->disconnect();
            return;
        }
    }
    else
    {
        std::cout << "SMM: Failed to get assets" << std::endl;
        this->disconnect();
    }
}

void
SMM::connect(const std::string &t_host, const std::string &t_user, const std::string &t_pass, const std::string &t_asset_name)
{
    if (this->conn != nullptr)
    {
        /* If the details have changed, or the connection has failed, disconnect */
        if (this->smm_host != t_host || this->smm_user != t_user || this->smm_pass != t_pass || this->asset_name != t_asset_name || smm_asset_connection_get_state (this->conn) != SMM_CONNECTION_CONNECTED)
        {
            std::cout << "SMM: Details have changed" << std::endl;
            this->disconnect();
        }
    }
    /* If there is no connection, store the details and connect */
    if (this->conn == nullptr)
    {
        this->smm_host = t_host;
        this->smm_user = t_user;
        this->smm_pass = t_pass;
        this->asset_name = t_asset_name;

        std::cout << "SMM: Connecting (" << this->smm_host << "," << this->smm_user << "," << this->asset_name << ")" << std::endl;
        this->connect();
    }
}


void
SMM::reportPosition(PositionData t_pd)
{
    if (this->asset)
    {
        uint64_t curr_ts = current_timestamp_ms();
        if (this->position_report_last_ts + 1000 <= curr_ts)
        {
            Point p = t_pd.getP();
            smm_asset_report_position (this->asset, p.getLatitude(), p.getLongitude(), t_pd.getAltitude(), t_pd.getHeading() / 100, 3);
            this->position_report_last_ts = curr_ts;
        }
    }
}

void SMM::search(Point current_pos)
{
    /* Search, or find a search to perform */
    if (this->asset == nullptr)
    {
        this->mav->setMode(flight_mode_rtl);
        return;
    }
    std::lock_guard<std::mutex> lk(this->search_lock);
    int retries = 0;
    while (this->current_search == nullptr)
    {
        auto new_search = smm_asset_get_search(this->asset, current_pos.getLatitude(), current_pos.getLongitude());
        if (new_search == nullptr)
        {
            /* No search to perform */
            /* Enter RTL and exit */
            this->mav->setMode(flight_mode_rtl);
            return;
        }
        if (smm_search_accept (new_search))
        {
            this->current_search = std::make_shared<SMMSearch>(new_search);
        }
        else
        {
            smm_search_destroy (new_search);
        }
        retries++;
        if (retries >= 3)
        {
            /* Failed to start a search 3 times, back-off for a while */
            this->mav->setMode(flight_mode_rtl);
            return;
        }
    }
    /* Load the search into AP */
    this->mav->loadSearch(this->current_search);
}

void SMM::reachedPoint(int point)
{
    /* See if we have completed this search or not */
    std::lock_guard<std::mutex> lk(this->search_lock);
    if (this->current_search != nullptr)
    {
        if (this->current_search->reachedPoint(point))
        {
            this->current_search = nullptr;
        }
    }
}

auto SMM::currentSearchPoints() -> int
{
    std::lock_guard<std::mutex> lk(this->search_lock);
    if (this->current_search != nullptr)
    {
        return this->current_search->getPointsCount();
    }
    return 0;
}

SMMSearch::SMMSearch(smm_search t_search)
{
    this->search = t_search;
    smm_waypoints wps = nullptr;
	size_t wps_count = 0;
    smm_search_get_waypoints (this->search, &wps, &wps_count);
	for (size_t i = 0; i < wps_count; i++)
	{
		Point wp(wps[i]->lat, wps[i]->lon);
        this->addPoint(wp);
	}
	smm_waypoints_free (wps, wps_count);
    this->altitude = smm_search_sweep_width (search);
}

auto
SMMSearch::getPointsCount() -> int
{
    return this->points.size();
}

auto
SMMSearch::reachedPoint(int point) -> bool
{
    if (this->search != nullptr)
    {
        if (point >= this->getPointsCount())
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

SMMSearch::~SMMSearch()
{
    if (this->search != nullptr)
    {
        smm_search_destroy (this->search);
        this->search = nullptr;
    }
}
