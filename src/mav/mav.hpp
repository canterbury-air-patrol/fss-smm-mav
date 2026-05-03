#pragma once
#include "imav.hpp"
#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include <cstdint>
#include <memory>

#include <string>

class mav_connection;

class MAV : public IMAV {
private:
    std::shared_ptr<mav_connection> connection;
    flight_mode mode{flight_mode_unknown};
    bool armed{false};
    Point goto_position{};
    Point goto_position_sent{};
    uint16_t target_altitude{0};
public:
    MAV(std::string t_addr, uint16_t t_port);
    MAV(MAV&) = delete;
    MAV(MAV&&) = delete;
    auto operator=(MAV&) -> MAV& = delete;
    auto operator=(MAV&&) -> MAV& = delete;
    ~MAV() override = default;
    auto getFlightMode() -> flight_mode { return this->mode; };
    auto getArmed() -> bool { return this->armed; };
    void attemptReconnect();
    void setMode(flight_mode fm) override;
    void disarm() override;
    void terminate() override;
    void gotoPosition(Point to) override;
    void setAltitude(uint16_t alt) override;
    auto getCurrentPosition() -> Point override;
    void loadSearch(const std::shared_ptr<SMMSearch> &);
    void sendADSB(PositionData pd);
    void registerPositionCB(notify_position_cb cb);
    void registerReachedCB(notify_reached_cb);
    void registerBatteryCB(notify_battery_status_cb cb);
};
