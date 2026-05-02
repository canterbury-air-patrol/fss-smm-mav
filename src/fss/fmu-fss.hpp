#pragma once
#include "fmu-fss-types.hpp"
#include "ifss.hpp"
#include <cstdint>
#include <memory>
#include <string>

#include "internal.hpp"

class FSS : public IFSS {
private:
    std::shared_ptr<fss_client_ssl> ssl_client{nullptr};
    uint16_t assigned_altitude{0};
    Point goto_point{};
public:
    FSS(std::string config_file);
    FSS(FSS&) = delete;
    FSS(FSS&&) = delete;
    auto operator=(FSS&) -> FSS& = delete;
    auto operator=(FSS&&) -> FSS& = delete;
    ~FSS() override = default;
    auto getAssetName() -> std::string;
    auto getGoto() -> Point override;
    void setGoto(Point);
    auto getAltitude() -> uint16_t override;
    void reportPosition(PositionData pd);
    void registerCommandCB(notify_fss_command_cb cb);
    void registerCommsStatusCB(notify_fss_comms_cb cb);
    void registerSMMSettingsCB(notify_smm_settings_cb cb);
    void registerPositionDataCB(notify_position_cb cb);
    void reachedPoint(int point, int total_points);
    void reportBatteryStatus(BatteryData bd);
    void reconnectAll();
};
