#pragma once
#include <cstdint>
#include <list>
#include <string>
#include <mutex>
#include <memory>

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
    std::shared_ptr<MAV> mav{nullptr};
    smm_connection conn{nullptr};
    std::string smm_host{};
    std::string smm_user{};
    std::string smm_pass{};
    std::string asset_name{};
    std::mutex search_lock{};
    std::shared_ptr<SMMSearch> current_search{nullptr};
    smm_assets assets_list{nullptr};
    size_t assets_list_count{0};
    smm_asset asset{nullptr};
    uint64_t position_report_last_ts{0};
    void connect();
    void disconnect();
public:
    explicit SMM(std::shared_ptr<MAV> t_mav);
    SMM(SMM&) = delete;
    SMM(SMM&&) = delete;
    auto operator=(SMM&) -> SMM& = delete;
    auto operator=(SMM&&) -> SMM& = delete;
    ~SMM();
    void connect(const std::string &host, const std::string &user, const std::string &pass, const std::string &asset_name);
    void search(Point current_pos);
    void reportPosition(PositionData t_pd);
    void reachedPoint(int point);
    auto currentSearchPoints() -> int;
};