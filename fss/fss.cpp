#include "fss.hpp"

uint16_t FSS::getAltitude()
{
    return this->assigned_altitude;
}

Point FSS::getGoto()
{
    return this->goto_point;
}
