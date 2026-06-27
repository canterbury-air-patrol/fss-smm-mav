#include "altitude-units.hpp"
#include "fmu-fss-types.hpp"
#include "fmu-fss.hpp"
#include "internal.hpp"
#include <cmath>
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

void
FSS::setAltitude (uint16_t alt)
{
    std::lock_guard<std::mutex> lk (this->state_lock);
    this->assigned_altitude = alt;
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
        /* PositionData carries metres; the FSS wire altitude is feet. std::lround
         * is undefined for a non-finite altitude, so guard it the same way
         * SMM::reportPosition does and report 0 for a bad reading. */
        const double alt_ft_d = metres_to_feet (t_pd.getAltitudeMetres ());
        auto alt_ft
            = std::isfinite (alt_ft_d) ? static_cast<int16_t> (std::lround (alt_ft_d)) : static_cast<int16_t> (0);
        this->ssl_client->sendPosition (p.getLatitude (), p.getLongitude (), alt_ft, t_pd.getHeading (),
                                        t_pd.getVelocityHorizontal (), t_pd.getVelocityVertical ());
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
    /* The wire altitude is uint32_t; assigned_altitude is uint16_t. Altitudes
     * are in feet, so the value always fits well within 16 bits (65535ft is far
     * above any operating ceiling) and the narrowing cast cannot lose data. */
    this->ssl_client->registerAltitudeUpdateCB ([this] (uint32_t alt)
                                                { this->setAltitude (static_cast<uint16_t> (alt)); });
}
