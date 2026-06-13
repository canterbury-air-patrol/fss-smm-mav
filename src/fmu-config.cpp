#include "fmu-config.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <limits>

namespace
{
constexpr double feet_to_metres = 0.3048;

/* Validate and store an altitude cap given in `value`, scaled to metres by
 * `to_metres` (1.0 for metres, feet_to_metres for feet). Non-numeric or
 * out-of-(uint16_t)range values are rejected with a warning, leaving the
 * existing default in place rather than silently truncating. */
void
setAltitudeCap (FmuConfig &cfg, const char *key, const Json::Value &value, double to_metres)
{
    if (!value.isNumeric ())
    {
        std::cerr << "Config: " << key << " is not numeric, using default " << cfg.altitude_cap_m << "m\n";
        return;
    }
    double metres = value.asDouble () * to_metres;
    if (metres < 0.0 || metres > std::numeric_limits<uint16_t>::max ())
    {
        std::cerr << "Config: " << key << " (" << value.asDouble () << ") out of range, using default "
                  << cfg.altitude_cap_m << "m\n";
        return;
    }
    cfg.altitude_cap_m = static_cast<uint16_t> (std::lround (metres));
}

/* Validate and store an integer config value, rejecting non-integral or
 * out-of-[lo, hi] values with a warning and leaving the default in place. */
void
setRangedInt (int &target, const char *key, const Json::Value &value, int lo, int hi)
{
    if (!value.isIntegral ())
    {
        std::cerr << "Config: " << key << " is not an integer, using default " << target << "\n";
        return;
    }
    int v = value.asInt ();
    if (v < lo || v > hi)
    {
        std::cerr << "Config: " << key << " (" << v << ") out of range [" << lo << ", " << hi << "], using default "
                  << target << "\n";
        return;
    }
    target = v;
}
} // namespace

auto
loadFmuConfig (const std::string &config_file) -> FmuConfig
{
    FmuConfig cfg{};

    std::ifstream f (config_file);
    if (!f.is_open ())
    {
        std::cerr << "Config: unable to open " << config_file << ", using defaults\n";
        return cfg;
    }

    /* Parse explicitly so a malformed file is reported (and ignored) rather
     * than throwing; each field below is then type-checked individually so a
     * single bad value falls back to its default instead of discarding the
     * whole block. */
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    if (!Json::parseFromStream (builder, f, &root, &errs))
    {
        std::cerr << "Config: failed to parse " << config_file << ": " << errs << ", using defaults\n";
        return cfg;
    }

    const Json::Value &fmu = root["fmu"];
    if (!fmu.isObject ())
    {
        return cfg;
    }

    /* The cap may be given in metres or feet; feet is the usual unit for the
     * regulatory ceiling (e.g. 400ft). Both are stored internally as metres.
     * If both are present, feet wins. */
    bool have_cap_m = fmu.isMember ("altitude_cap_m");
    bool have_cap_ft = fmu.isMember ("altitude_cap_ft");
    if (have_cap_m && have_cap_ft)
    {
        std::cerr << "Config: both altitude_cap_m and altitude_cap_ft set, using altitude_cap_ft\n";
    }
    if (have_cap_ft)
    {
        setAltitudeCap (cfg, "altitude_cap_ft", fmu["altitude_cap_ft"], feet_to_metres);
    }
    else if (have_cap_m)
    {
        setAltitudeCap (cfg, "altitude_cap_m", fmu["altitude_cap_m"], 1.0);
    }

    if (fmu.isMember ("camera_fov_deg"))
    {
        const Json::Value &fov_value = fmu["camera_fov_deg"];
        if (!fov_value.isNumeric ())
        {
            std::cerr << "Config: camera_fov_deg is not numeric, using default " << cfg.camera_fov_deg << "\n";
        }
        /* A total field of view outside (0, 180) makes the altitude derivation
         * degenerate (tan(fov/2) <= 0 or undefined). */
        else if (double fov = fov_value.asDouble (); fov > 0.0 && fov < 180.0)
        {
            cfg.camera_fov_deg = fov;
        }
        else
        {
            std::cerr << "Config: camera_fov_deg (" << fov << ") out of range (0,180), using default "
                      << cfg.camera_fov_deg << "\n";
        }
    }

    if (fmu.isMember ("lowbat_threshold"))
    {
        setRangedInt (cfg.lowbat_threshold, "lowbat_threshold", fmu["lowbat_threshold"], 0, 100);
    }
    if (fmu.isMember ("reconnect_interval_s"))
    {
        setRangedInt (cfg.reconnect_interval_s, "reconnect_interval_s", fmu["reconnect_interval_s"], 1, 3600);
    }

    return cfg;
}
