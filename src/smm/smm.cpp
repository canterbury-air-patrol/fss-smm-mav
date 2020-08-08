#include <cstring>

#include "smm.hpp"
#include <smm-asset.h>

SMM::SMM(MAV *t_mav) : mav(t_mav)
{
    smm_asset_debugging_set (true);
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
        this->disconnect();
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
        if (this->asset == NULL)
        {
            this->disconnect();
        }
    }
    else
    {
        this->disconnect();
    }
}

void
SMM::connect(std::string t_host, std::string t_user, std::string t_pass, std::string t_asset_name)
{
    if (this->conn == nullptr)
    {
        /* If the details have changed, or the connection has failed, disconnect */
        if (this->smm_host != t_host || this->smm_user != t_user || this->smm_pass != t_pass || this->asset_name != t_asset_name || smm_asset_connection_get_state (this->conn) != SMM_CONNECTION_CONNECTED)
        {
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

        this->connect();
    }
}

void SMM::search(Point current_pos)
{
    /* Search, or find a search to perform */
    if (this->asset == nullptr)
    {
        return;
    }
    while (this->current_search == nullptr)
    {
        smm_search new_search = smm_asset_get_search(this->asset, current_pos.getLongitude(), current_pos.getLongitude());
        if (new_search == nullptr)
        {
            /* No search to perform */
            /* Enter RTL and exit */
            return;
        }
        if (smm_search_accept (new_search))
        {
            this->current_search = new SMMSearch(new_search);
        }
    }
    /* Load the search into AP */
    this->mav->loadSearch(this->current_search);
}

SMMSearch::SMMSearch(smm_search search)
{
    smm_waypoints wps = nullptr;
	size_t wps_count = 0;
    smm_search_get_waypoints (search, &wps, &wps_count);
	for (size_t i = 0; i < wps_count; i++)
	{
		Point wp;
        this->addPoint(wp);
	}
	smm_waypoints_free (wps, wps_count);
}