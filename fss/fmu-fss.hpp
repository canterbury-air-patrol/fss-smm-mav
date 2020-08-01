#pragma once
#include "fmu-fss-types.hpp"
#include "../fmu-types.hpp"
#include <bits/stdint-uintn.h>
#include <string>

class fss_client;

class FSS {
private:
    fss_client *client{nullptr};
    uint16_t assigned_altitude{0};
    Point goto_point{};
public:
    FSS(const char *config_file);
    Point getGoto();
    uint16_t getAltitude();
};