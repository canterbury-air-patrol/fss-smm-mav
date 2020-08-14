#pragma once
#include <string>
#include <vector>

#include "../fmu-types.hpp"

extern "C" {
#include <smm-asset.h>
};

class SMMSettings {
private:
    std::string url;
    std::string user;
    std::string pass;
public:
    SMMSettings(std::string t_url, std::string t_user, std::string t_pass) : url(t_url), user(t_user), pass(t_pass) {};
    std::string getURL() { return this->url; };
    std::string getUsername() { return this->user; };
    std::string getPassword() { return this->pass; };
};

class SMMSearch {
private:
    std::vector<Point> points{};
    int current_point{0};
    int altitude{0};
public:
    SMMSearch() {};
    SMMSearch(smm_search);
    void addPoint(Point p) { this->points.push_back(p); };
    int getCurrentPointIdx() { return this->current_point; };
    Point getCurrentPoint() { return this->points[this->current_point]; };
    const std::vector<Point> getPoints() { return this->points; };
    uint16_t getAltitude() { return this->altitude; };
};