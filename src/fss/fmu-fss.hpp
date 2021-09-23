#pragma once
#include "fmu-fss-types.hpp"
#include "../fmu-types.hpp"
#include <bits/stdint-uintn.h>
#include <memory>
#include <string>

#include "internal.hpp"

class FSS {
private:
    std::shared_ptr<fss_client> client{nullptr};
    std::shared_ptr<fss_client_ssl> ssl_client{nullptr};
    uint16_t assigned_altitude{0};
    Point goto_point{};
    bool ssl{false};
public:
    FSS(bool t_ssl, std::string config_file);
    auto getAssetName() -> std::string;
    auto getGoto() -> Point;
    void setGoto(Point);
    auto getAltitude() -> uint16_t;
    void reportPosition(PositionData pd);
    void registerCommandCB(notify_fss_command_cb cb);
    void registerCommsStatusCB(notify_fss_comms_cb cb);
    void registerSMMSettingsCB(notify_smm_settings_cb cb);
    void registerPositionDataCB(notify_position_cb cb);
    void reachedPoint(int point, int total_points);
    void reportBatteryStatus(BatteryData bd);
    void reconnectAll();
};