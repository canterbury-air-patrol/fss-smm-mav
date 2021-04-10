#pragma once
#include <string>
#include <vector>

#include "../fmu-types.hpp"

extern "C" {
#include <smm-asset.h>
};

class SMMSettings {
private:
    std::string url{};
    std::string user{};
    std::string pass{};
public:
    SMMSettings() = default;
    SMMSettings(std::string t_url, std::string t_user, std::string t_pass) : url(std::move(t_url)), user(std::move(t_user)), pass(std::move(t_pass)) {};
    auto getURL() -> std::string { return this->url; };
    auto getUsername() -> std::string { return this->user; };
    auto getPassword() -> std::string { return this->pass; };
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
    ~SMMSearch();
    void addPoint(Point p) { this->points.push_back(p); };
    auto getCurrentPointIdx() -> int { return this->current_point; };
    auto getCurrentPoint() -> Point { return this->points[this->current_point]; };
    auto getPoints() -> const std::vector<Point> { return this->points; };
    auto getAltitude() -> uint16_t { return this->altitude; };
    auto getPointsCount() -> int;
    auto reachedPoint(int point) -> bool;
};