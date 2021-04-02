#include "fmu-fss-types.hpp"
#include "fmu-fss.hpp"
#include "internal.hpp"

std::string FSS::getAssetName()
{
    if (this->client != nullptr)
    {
        return this->client->getAssetName();
    }
    return "";
}

uint16_t FSS::getAltitude()
{
    return this->assigned_altitude;
}

Point FSS::getGoto()
{
    return this->goto_point;
}

void FSS::setGoto(Point p)
{
    this->goto_point = p;
}

void FSS::registerCommandCB(notify_fss_command_cb cb)
{
    if (this->client != nullptr)
    {
        this->client->registerCommandCB(cb);
    }
};

void FSS::registerCommsStatusCB(notify_fss_comms_cb cb)
{
    if (this->client != nullptr)
    {
        this->client->registerCommsStatusCB(cb);
    }
};

void FSS::registerSMMSettingsCB(notify_smm_settings_cb cb)
{
    if (this->client != nullptr)
    {
        this->client->registerSMMSettingsCB(cb);
    }
}

void FSS::registerPositionDataCB(notify_position_cb cb)
{
    if (this->client != nullptr)
    {
        this->client->registerPositionDataCB(cb);
    }
}

void FSS::reportPosition(PositionData t_pd)
{
    if (this->client != nullptr)
    {
        Point p = t_pd.getP();
        this->client->sendPosition(p.getLatitude(), p.getLongitude(), t_pd.getAltitude(), t_pd.getHeading(), t_pd.getVelocityHorizontal(), t_pd.getVelocityVertical());
    }
}

void FSS::reachedPoint(int point, int total_points)
{
    if (this->client != nullptr)
    {
        this->client->reachedPoint(point, total_points);
    }
}

void FSS::reportBatteryStatus(BatteryData bd)
{
    if (this->client != nullptr)
    {
        this->client->sendBatteryStatus(bd.getRemaining(), bd.getConsumed());
    }
}

static void
goto_updated (void *priv, Point p)
{
    if (priv != nullptr)
    {
        FSS *fss = (FSS *)priv;
        fss->setGoto(p);
    }
}

void
FSS::reconnectAll()
{
    this->client->attemptReconnect();
}

FSS::FSS(const char *config_file)
{
    this->client = new fss_client(config_file);
    if (this->client != nullptr)
    {
        this->client->registerGotoUpdateCB(goto_updated, this);
    }
}

FSS::~FSS()
{
    if (this->client != nullptr)
    {
        delete this->client;
        this->client = nullptr;
    }
}