#include "fmu-config.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <limits>

namespace
{
constexpr double feet_to_metres = 0.3048;

/* Validate and store an altitude (metres) into `target`, scaled by `to_metres`
 * (1.0 for metres, feet_to_metres for feet). Non-numeric or out-of-(uint16_t)
 * range values are rejected with a warning, leaving the existing default in
 * place rather than silently truncating. */
void
setAltitudeMetres (uint16_t &target, const char *key, const Json::Value &value, double to_metres)
{
    if (!value.isNumeric ())
    {
        std::cerr << "Config: " << key << " is not numeric, using default " << target << "m\n";
        return;
    }
    double metres = value.asDouble () * to_metres;
    if (metres < 0.0 || metres > std::numeric_limits<uint16_t>::max ())
    {
        std::cerr << "Config: " << key << " (" << value.asDouble () << ") out of range, using default " << target
                  << "m\n";
        return;
    }
    target = static_cast<uint16_t> (std::lround (metres));
}

/* Load an altitude that may be given in metres (`key_m`) or feet (`key_ft`)
 * into `target`. If both are present, feet wins (with a warning). */
void
loadAltitude (uint16_t &target, const Json::Value &fmu, const char *key_m, const char *key_ft)
{
    bool have_m = fmu.isMember (key_m);
    bool have_ft = fmu.isMember (key_ft);
    if (have_m && have_ft)
    {
        std::cerr << "Config: both " << key_m << " and " << key_ft << " set, using " << key_ft << "\n";
    }
    if (have_ft)
    {
        setAltitudeMetres (target, key_ft, fmu[key_ft], feet_to_metres);
    }
    else if (have_m)
    {
        setAltitudeMetres (target, key_m, fmu[key_m], 1.0);
    }
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

    /* The cap and floor may each be given in metres or feet; feet is the usual
     * unit for the regulatory ceiling (e.g. 400ft). Both are stored as metres. */
    loadAltitude (cfg.altitude_cap_m, fmu, "altitude_cap_m", "altitude_cap_ft");
    loadAltitude (cfg.altitude_floor_m, fmu, "altitude_floor_m", "altitude_floor_ft");
    /* The floor must never sit above the cap, or the derived altitude could not
     * satisfy both bounds. */
    if (cfg.altitude_floor_m > cfg.altitude_cap_m)
    {
        std::cerr << "Config: altitude_floor (" << cfg.altitude_floor_m << "m) exceeds altitude_cap ("
                  << cfg.altitude_cap_m << "m), clamping floor to cap\n";
        cfg.altitude_floor_m = cfg.altitude_cap_m;
    }

    /* The goto altitude may be given in metres or feet, like the cap/floor.
     * Clamp it into [floor, cap] afterwards: a goto must not be flown above the
     * regulatory ceiling, nor below the floor (and the cap/floor have already
     * been resolved and made consistent above). */
    loadAltitude (cfg.goto_altitude_m, fmu, "goto_altitude_m", "goto_altitude_ft");
    if (cfg.goto_altitude_m > cfg.altitude_cap_m)
    {
        std::cerr << "Config: goto_altitude (" << cfg.goto_altitude_m << "m) exceeds altitude_cap ("
                  << cfg.altitude_cap_m << "m), clamping to cap\n";
        cfg.goto_altitude_m = cfg.altitude_cap_m;
    }
    else if (cfg.goto_altitude_m < cfg.altitude_floor_m)
    {
        std::cerr << "Config: goto_altitude (" << cfg.goto_altitude_m << "m) below altitude_floor ("
                  << cfg.altitude_floor_m << "m), clamping to floor\n";
        cfg.goto_altitude_m = cfg.altitude_floor_m;
    }

    if (fmu.isMember ("camera_fov_deg"))
    {
        const Json::Value &fov_value = fmu["camera_fov_deg"];
        if (!fov_value.isNumeric ())
        {
            std::cerr << "Config: camera_fov_deg is not numeric, using default " << cfg.camera_fov_deg << "\n";
        }
        else
        {
            double fov = fov_value.asDouble ();
            /* A total field of view outside (0, 180) makes the altitude
             * derivation degenerate (tan(fov/2) <= 0 or undefined). */
            if (fov > 0.0 && fov < 180.0)
            {
                cfg.camera_fov_deg = fov;
            }
            else
            {
                std::cerr << "Config: camera_fov_deg (" << fov << ") out of range (0,180), using default "
                          << cfg.camera_fov_deg << "\n";
            }
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

    if (fmu.isMember ("log_level"))
    {
        const Json::Value &lvl = fmu["log_level"];
        if (!lvl.isString ())
        {
            std::cerr << "Config: log_level is not a string, using default\n";
        }
        else if (std::string s = lvl.asString (); s == "error")
        {
            cfg.log_level = LogLevel::error;
        }
        else if (s == "info")
        {
            cfg.log_level = LogLevel::info;
        }
        else if (s == "debug")
        {
            cfg.log_level = LogLevel::debug;
        }
        else
        {
            std::cerr << "Config: log_level (" << s << ") unknown (error|info|debug), using default\n";
        }
    }

    return cfg;
}
