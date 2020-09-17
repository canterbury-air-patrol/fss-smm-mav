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
    ~FSS();
    std::string getAssetName();
    Point getGoto();
    void setGoto(Point);
    uint16_t getAltitude();
    void reportPosition(PositionData pd);
    void registerCommandCB(notify_fss_command_cb cb, void *priv);
    void registerCommsStatusCB(notify_fss_comms_cb cb, void *priv);
    void registerSMMSettingsCB(notify_smm_settings_cb cb, void *priv);
    void registerPositionDataCB(notify_position_cb cb, void *priv);
    void reachedPoint(int point, int total_points);
    void reportBatteryStatus(BatteryData bd);
    void reconnectAll();
};