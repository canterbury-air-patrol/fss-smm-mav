#pragma once
#include <bits/stdint-uintn.h>
#include <list>
#include <string>

#include "../fmu-types.hpp"

extern "C" {
#include <smm-asset.h>
};

class SMMSearch {
private:
    std::list<Point> points{};
    uint64_t search_id{0};
public:
    SMMSearch(uint64_t id) : search_id(id) {};
    void addPoint(Point p) { this->points.push_back(p); };
    uint64_t getSearchId() { return this->search_id; };
    const std::list<Point> getPoints() { return this->points; };
};

enum SMMCommand {
    smm_cmd_none,
    smm_cmd_abandon_search,
    smm_cmd_mission_complete,
};

class SMM {
private:
    smm_connection conn{nullptr};
    std::string smm_host{};
    std::string smm_user{};
    std::string smm_pass{};
    std::string asset_name{};
    SMMSearch *current_search{nullptr};
    smm_assets assets_list;
    size_t assets_list_count;
    smm_asset asset;
    void connect();
    void disconnect();
public:
    SMM();
    void connect(std::string host, std::string user, std::string pass, std::string asset_name);
    void search();
};