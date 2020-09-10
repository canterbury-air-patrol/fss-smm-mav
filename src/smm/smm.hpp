#pragma once
#include <bits/stdint-uintn.h>
#include <list>
#include <string>
#include <mutex>

#include "../fmu-types.hpp"
#include "../mav/mav.hpp"

extern "C" {
#include <smm-asset.h>
};

enum SMMCommand {
    smm_cmd_none,
    smm_cmd_abandon_search,
    smm_cmd_mission_complete,
};

class SMM {
private:
    MAV *mav{nullptr};
    smm_connection conn{nullptr};
    std::string smm_host{};
    std::string smm_user{};
    std::string smm_pass{};
    std::string asset_name{};
    std::mutex search_lock{};
    SMMSearch *current_search{nullptr};
    smm_assets assets_list{nullptr};
    size_t assets_list_count{0};
    smm_asset asset{nullptr};
    uint64_t position_report_last_ts{0};
    void connect();
    void disconnect();
public:
    SMM(MAV *t_mav);
    ~SMM();
    void connect(std::string host, std::string user, std::string pass, std::string asset_name);
    void search(Point current_pos);
    void reportPosition(PositionData t_pd);
    void reachedPoint(int point);
    int currentSearchPoints();
};