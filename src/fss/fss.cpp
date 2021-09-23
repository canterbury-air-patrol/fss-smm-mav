#include "fmu-fss-types.hpp"
#include "fmu-fss.hpp"
#include "internal.hpp"
#include <cstdint>
#include <memory>
#include <type_traits>


auto
FSS::getAssetName() -> std::string
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            return this->ssl_client->getAssetName();
        }
    } else {
        if (this->client != nullptr)
        {
            return this->client->getAssetName();
        }
    }
    return "";
}

auto
FSS::getAltitude() -> uint16_t
{
    return this->assigned_altitude;
}

auto
FSS::getGoto() -> Point
{
    return this->goto_point;
}

void FSS::setGoto(Point p)
{
    this->goto_point = p;
}

void FSS::registerCommandCB(notify_fss_command_cb cb)
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            this->ssl_client->registerCommandCB(cb);
        }
    } else {
        if (this->client != nullptr)
        {
            this->client->registerCommandCB(cb);
        }
    }
};

void FSS::registerCommsStatusCB(notify_fss_comms_cb cb)
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            this->ssl_client->registerCommsStatusCB(cb);
        }
    } else {
        if (this->client != nullptr)
        {
            this->client->registerCommsStatusCB(cb);
        }
    }
};

void FSS::registerSMMSettingsCB(notify_smm_settings_cb cb)
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            this->ssl_client->registerSMMSettingsCB(cb);
        }
    } else {
        if (this->client != nullptr)
        {
            this->client->registerSMMSettingsCB(cb);
        }
    }
}

void FSS::registerPositionDataCB(notify_position_cb cb)
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            this->ssl_client->registerPositionDataCB(cb);
        }
    } else {
        if (this->client != nullptr)
        {
            this->client->registerPositionDataCB(cb);
        }
    }
}

void FSS::reportPosition(PositionData t_pd)
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            Point p = t_pd.getP();
            this->ssl_client->sendPosition(p.getLatitude(), p.getLongitude(), static_cast<uint16_t>(t_pd.getAltitude()), t_pd.getHeading(), t_pd.getVelocityHorizontal(), t_pd.getVelocityVertical());
        }
    } else {
        if (this->client != nullptr)
        {
            Point p = t_pd.getP();
            this->client->sendPosition(p.getLatitude(), p.getLongitude(), static_cast<uint16_t>(t_pd.getAltitude()), t_pd.getHeading(), t_pd.getVelocityHorizontal(), t_pd.getVelocityVertical());
        }
    }
}

void FSS::reachedPoint(int point, int total_points)
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            this->ssl_client->reachedPoint(point, total_points);
        }
    } else {
        if (this->client != nullptr)
        {
            this->client->reachedPoint(point, total_points);
        }
    }
}

void FSS::reportBatteryStatus(BatteryData bd)
{
    if (this->ssl)
    {
        if (this->ssl_client != nullptr)
        {
            this->ssl_client->sendBatteryStatus(bd.getRemaining(), bd.getConsumed());
        }
    } else {
        if (this->client != nullptr)
        {
            this->client->sendBatteryStatus(bd.getRemaining(), bd.getConsumed());
        }
    }
}

static void
goto_updated (void *priv, Point p)
{
    if (priv != nullptr)
    {
        auto fss = static_cast<FSS *>(priv);
        fss->setGoto(p);
    }
}

void
FSS::reconnectAll()
{
    if (this->ssl)
    {
        this->ssl_client->attemptReconnect();
    } else {
        this->client->attemptReconnect();
    }
}

FSS::FSS(bool t_ssl, std::string config_file)
{
    this->ssl = t_ssl;
    if (this->ssl)
    {
        this->ssl_client = std::make_shared<fss_client_ssl>(config_file.c_str());
    } else {
        this->client = std::make_shared<fss_client>(config_file.c_str());
    }
    if (this->client != nullptr)
    {
        this->client->registerGotoUpdateCB(goto_updated, this);
    }
}