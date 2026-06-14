#pragma once
#include <cmath>
#include <cstdint>

/* Pure helpers for the search flight-altitude derivation, factored out of the
 * SMMSearch constructor so the safety-critical formula and clamp can be unit
 * tested without pulling in the smm-asset C library. */

/* Raw flight altitude (metres AGL, unclamped) for a search of the given sweep
 * (lane) width, imaged by a downward-pointing camera with total cross-track
 * field of view camera_fov_deg. The ground footprint width at altitude h is
 * 2 * h * tan(fov / 2), so h = width / (2 * tan(fov / 2)). The caller must
 * ensure camera_fov_deg is in (0, 180) so tan(fov / 2) is strictly positive
 * (the config loader validates this). */
inline auto
raw_search_altitude (double sweep_width, double camera_fov_deg) -> double
{
    /* std::numbers::pi is C++20; this project is C++17, so use a local constant
     * rather than the non-standard M_PI macro from transitive includes. */
    constexpr double pi = 3.14159265358979323846;
    double half_fov_rad = (camera_fov_deg * pi / 180.0) / 2.0;
    return sweep_width / (2.0 * std::tan (half_fov_rad));
}

/* Clamp a derived altitude into [altitude_floor, altitude_cap]. The floor keeps
 * a tiny or zero sweep width from putting the search at ground level; the cap is
 * the regulatory ceiling. altitude_floor <= altitude_cap is assumed (guaranteed
 * at config load), so the two bounds never conflict. */
inline auto
clamp_search_altitude (double altitude, uint16_t altitude_floor, uint16_t altitude_cap) -> uint16_t
{
    if (altitude > altitude_cap)
    {
        return altitude_cap;
    }
    if (altitude < altitude_floor)
    {
        return altitude_floor;
    }
    return static_cast<uint16_t> (altitude);
}
