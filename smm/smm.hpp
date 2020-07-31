#pragma once
#include <bits/stdint-uintn.h>
#include <list>

#include "../fmu-types.hpp"

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
    SMMSearch *current_search{nullptr};
public:
    SMM() {};
    void search();
};