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

void FSS::registerCommandCB(notify_fss_command_cb cb, void *priv)
{
    if (this->client != nullptr)
    {
        this->client->registerCommandCB(cb, priv);
    }
};

void FSS::registerCommsStatusCB(notify_fss_comms_cb cb, void *priv)
{
    if (this->client != nullptr)
    {
        this->client->registerCommsStatusCB(cb, priv);
    }
};

void FSS::registerSMMSettingsCB(notify_smm_settings_cb cb, void *priv)
{
    if (this->client != nullptr)
    {
        this->client->registerSMMSettingsCB(cb, priv);
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

FSS::FSS(const char *config_file)
{
    this->client = new fss_client(config_file);
    if (this->client != nullptr)
    {
        this->client->registerGotoUpdateCB(goto_updated, this);
    }
}