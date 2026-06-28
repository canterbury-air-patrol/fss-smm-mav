#pragma once
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include <secure-string.hpp>

#include "../mav/mav.hpp"
#include "ismm.hpp"
#include "smm-types.hpp"

extern "C"
{
#include <smm-asset.h>
};

class SMM : public ISMM
{
    /* Test-only accessor: lets the resume tests inject a held (paused) search,
     * since the only production path to current_search is an HTTP acquire. */
    friend struct SMMTestAccess;

  private:
    MAV &mav;
    smm_connection conn{ nullptr };
    std::string smm_host{};
    flight_safety_system::secure_string smm_user{};
    flight_safety_system::secure_string smm_pass{};
    std::string asset_name{};
    /* Local asset properties (from client.json), fixed at construction. */
    uint16_t altitude_cap{ 122 };
    uint16_t altitude_floor{ 10 };
    double camera_fov_deg{ 90.0 };
    std::mutex search_lock{};
    std::shared_ptr<SMMSearch> current_search{ nullptr };
    smm_assets assets_list{ nullptr };
    size_t assets_list_count{ 0 };
    smm_asset asset{ nullptr };
    uint64_t position_report_last_ts{ 0 };
    /* Minimum gap between SMM position reports, milliseconds (from config). */
    uint64_t position_report_interval_ms{ 1000 };
    /* Connect / total-transfer timeouts applied to the SMM connection (seconds,
     * from config) so a slow or hung endpoint cannot block a call for the full
     * library-default TCP window. Applied after login in connect(). */
    long connect_timeout_s{ 5 };
    long transfer_timeout_s{ 10 };
    bool search_active{ false };
    uint64_t search_retry_ts{ 0 };
    static constexpr uint64_t search_retry_interval_ms{ 5000 };
    void connect ();
    void disconnect ();
    void tryAcquireSearch (std::unique_lock<std::mutex> &lock, Point current_pos);

  public:
    /* The tuning parameters (report interval, connect/transfer timeouts) have no
     * defaults here: each value lives once in FmuConfig (its default) and every
     * caller passes it through, so there is no duplicated literal to drift. */
    SMM (MAV &t_mav, uint16_t t_altitude_cap, uint16_t t_altitude_floor, double t_camera_fov_deg,
         uint64_t t_position_report_interval_ms, long t_connect_timeout_s, long t_transfer_timeout_s);
    SMM (SMM &) = delete;
    SMM (SMM &&) = delete;
    auto operator= (SMM &) -> SMM & = delete;
    auto operator= (SMM &&) -> SMM & = delete;
    ~SMM () override;
    void connect (const std::string &host, const flight_safety_system::secure_string &user,
                  const flight_safety_system::secure_string &pass, const std::string &asset_name);
    void search (Point current_pos) override;
    void cancelSearch () override;
    /* Timer-driven retry of a pending (active but not yet acquired) search.
     * Called periodically off the reconnect thread so acquisition is not
     * coupled solely to MAV position-report cadence (see todo/41). */
    void retryPendingSearch ();
    void reportPosition (PositionData t_pd);
    void reachedPoint (int point);
    auto currentSearchPoints () -> int;
};
