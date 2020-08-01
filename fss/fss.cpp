#include "fmu-fss.hpp"
#include "internal.hpp"

uint16_t FSS::getAltitude()
{
    return this->assigned_altitude;
}

Point FSS::getGoto()
{
    return this->goto_point;
}

FSS::FSS(const char *config_file)
{
    this->client = new fss_client(config_file);
}