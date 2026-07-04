#pragma once
#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include "imav.hpp"
#include "mav-params.hpp"
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
    uint32_t target_altitude{ 0 };
    terminate_action action;

  public:
    MAV (std::string t_addr, uint16_t t_port, terminate_action ta, const MavParams &t_params);
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
    auto setMode (flight_mode fm) -> bool override;
    auto disarm () -> bool override;
    auto terminate () -> bool override;
    void gotoPosition (Point to) override;
    auto setAltitude (uint32_t alt) -> bool override;
    auto getCurrentPosition () -> Point override;
    void loadSearch (const std::shared_ptr<SMMSearch> &search) override;
    void sendADSB (PositionData pd) override;
    void registerPositionCB (notify_position_cb cb);
    void registerReachedCB (notify_reached_cb);
    void registerBatteryCB (notify_battery_status_cb cb);
    void registerMavCommsStatusCB (notify_mav_comms_cb cb) override;
};
