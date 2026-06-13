#include "fmu-fss-types.hpp"
#include "fmu-fss.hpp"
#include "internal.hpp"
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>

auto
FSS::getAssetName () -> std::string
{
    if (this->ssl_client != nullptr)
    {
        return this->ssl_client->getAssetName ();
    }
    return "";
}

auto
FSS::getAltitude () -> uint16_t
{
    std::lock_guard<std::mutex> lk (this->state_lock);
    return this->assigned_altitude;
}

auto
FSS::getGoto () -> Point
{
    std::lock_guard<std::mutex> lk (this->state_lock);
    return this->goto_point;
}

void
FSS::setGoto (Point p)
{
    std::lock_guard<std::mutex> lk (this->state_lock);
    this->goto_point = p;
}

void
FSS::registerCommandCB (notify_fss_command_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerCommandCB (std::move (cb));
    }
};

void
FSS::registerCommsStatusCB (notify_fss_comms_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerCommsStatusCB (std::move (cb));
    }
};

void
FSS::registerSMMSettingsCB (notify_smm_settings_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerSMMSettingsCB (std::move (cb));
    }
}

void
FSS::registerPositionDataCB (notify_position_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerPositionDataCB (std::move (cb));
    }
}

void
FSS::reportPosition (PositionData t_pd)
{
    if (this->ssl_client != nullptr)
    {
        Point p = t_pd.getP ();
        this->ssl_client->sendPosition (p.getLatitude (), p.getLongitude (), static_cast<int16_t> (t_pd.getAltitude ()),
                                        t_pd.getHeading (), t_pd.getVelocityHorizontal (), t_pd.getVelocityVertical ());
    }
}

void
FSS::reachedPoint (int point, int total_points)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->reachedPoint (point, total_points);
    }
}

void
FSS::reportBatteryStatus (BatteryData bd)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->sendBatteryStatus (bd.getRemaining (), bd.getConsumed (), bd.getVoltage ());
    }
}

void
FSS::reconnectAll ()
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->attemptReconnect ();
    }
}

FSS::FSS (const std::string &config_file)
{
    this->ssl_client = std::make_shared<fss_client_ssl> (config_file.c_str ());
    this->ssl_client->registerGotoUpdateCB ([this] (Point p) { this->setGoto (p); });
}