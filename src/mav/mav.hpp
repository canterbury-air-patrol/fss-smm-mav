#pragma once
#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include "imav.hpp"
#include <cstdint>
#include <memory>

#include <string>

enum class terminate_action
{
    none,
    disarm,
    terminate,
};

class mav_connection;

class MAV : public IMAV
{
  private:
    std::shared_ptr<mav_connection> connection;
    flight_mode mode{ flight_mode_unknown };
    bool armed{ false };
    Point goto_position{};
    uint16_t target_altitude{ 0 };
    terminate_action action;

  public:
    MAV (std::string t_addr, uint16_t t_port, terminate_action ta, uint16_t t_goto_altitude_m,
         uint16_t t_altitude_floor_m, uint16_t t_altitude_cap_m);
    MAV (MAV &) = delete;
    MAV (MAV &&) = delete;
    auto operator= (MAV &) -> MAV & = delete;
    auto operator= (MAV &&) -> MAV & = delete;
    ~MAV () override = default;
    auto
    getFlightMode () -> flight_mode
    {
        return this->mode;
    };
    auto
    getArmed () -> bool
    {
        return this->armed;
    };
    /* Open the MAV connection and start its background threads. Call once,
     * after registering callbacks, so those threads cannot race registration. */
    void start ();
    void attemptReconnect ();
    void setMode (flight_mode fm) override;
    void disarm () override;
    void terminate () override;
    void gotoPosition (Point to) override;
    void setAltitude (uint16_t alt) override;
    auto getCurrentPosition () -> Point override;
    void loadSearch (const std::shared_ptr<SMMSearch> &);
    void sendADSB (PositionData pd);
    void registerPositionCB (notify_position_cb cb);
    void registerReachedCB (notify_reached_cb);
    void registerBatteryCB (notify_battery_status_cb cb);
    void registerMavCommsStatusCB (notify_mav_comms_cb cb) override;
};
