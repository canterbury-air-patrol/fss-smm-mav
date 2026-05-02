#pragma once
#include <string>
#include <vector>

#include <secure-string.hpp>

#include "../fmu-core-types.hpp"
#include "smm-command.hpp"

extern "C" {
#include <smm-asset.h>
};

class SMMSettings {
private:
    std::string url{};
    flight_safety_system::secure_string user{};
    flight_safety_system::secure_string pass{};
public:
    SMMSettings() = default;
    SMMSettings(std::string t_url, flight_safety_system::secure_string t_user, flight_safety_system::secure_string t_pass) : url(std::move(t_url)), user(std::move(t_user)), pass(std::move(t_pass)) {};
    auto getURL() -> std::string { return this->url; };
    auto getUsername() -> const flight_safety_system::secure_string & { return this->user; };
    auto getPassword() -> const flight_safety_system::secure_string & { return this->pass; };
};

class SMMSearch {
private:
    std::vector<Point> points{};
    int current_point{0};
    int altitude{0};
    smm_search search{nullptr};
public:
    SMMSearch() = default;
    explicit SMMSearch(smm_search);
    SMMSearch(SMMSearch&) = delete;
    SMMSearch(SMMSearch&&) = delete;
    auto operator=(SMMSearch&) -> SMMSearch& = delete;
    auto operator=(SMMSearch&&) -> SMMSearch& = delete;
    ~SMMSearch();
    void addPoint(Point p) { this->points.push_back(p); };
    auto getCurrentPointIdx() -> int { return this->current_point; };
    auto getCurrentPoint() -> Point { return this->points[this->current_point]; };
    auto getPoints() -> const std::vector<Point> { return this->points; };
    auto getAltitude() -> uint16_t { return this->altitude; };
    auto getPointsCount() -> int;
    auto reachedPoint(int point) -> bool;
};
