#pragma once
#include "../fmu-types.hpp"
#include <bits/stdint-uintn.h>

enum FSSCommand {
    fss_cmd_unknown,
    fss_cmd_manual,
    fss_cmd_rtl,
    fss_cmd_hold,
    fss_cmd_altitude,
    fss_cmd_goto,
    fss_cmd_continue,
    fss_cmd_disarm,
    fss_cmd_terminate,
};

class FSS {
private:
    uint16_t assigned_altitude;
    Point goto_point;
public:
    FSS();
    Point getGoto();
    uint16_t getAltitude();
};