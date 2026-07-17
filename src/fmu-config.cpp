#include "fmu-config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <limits>
#include <stdexcept>

namespace
{
/* True for an empty string or one that is only whitespace. Used to reject a
 * blank mav_address, which would otherwise be passed to the MAV connection as a
 * "valid" host and fail later in a harder-to-diagnose way. */
auto
is_blank (const std::string &s) -> bool
{
    return std::all_of (s.begin (), s.end (), [] (unsigned char c) { return std::isspace (c) != 0; });
}
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

    /* The config file is load-bearing for the FSS client (name/SSL/server
     * config constructed from the same file, see App's FSS member), which
     * fails fatally without it regardless of what this function does. So an
     * unopenable or unparsable file fails hard here too, rather than warning
     * "using defaults" and then failing fatally moments later on the FSS
     * side for the same underlying cause. Per-key fallbacks below (for the
     * optional "fmu" block once the file itself is known-good) are
     * unaffected. */
    std::ifstream f (config_file);
    if (!f.is_open ())
    {
        throw std::runtime_error ("Config: unable to open " + config_file);
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    if (!Json::parseFromStream (builder, f, &root, &errs))
    {
        throw std::runtime_error ("Config: failed to parse " + config_file + ": " + errs);
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
    if (fmu.isMember ("low_battery_latch_count"))
    {
        setRangedInt (cfg.low_battery_latch_count, "low_battery_latch_count", fmu["low_battery_latch_count"], 1, 100);
    }
    if (fmu.isMember ("reconnect_interval_s"))
    {
        setRangedInt (cfg.reconnect_interval_s, "reconnect_interval_s", fmu["reconnect_interval_s"], 1, 3600);
    }
    if (fmu.isMember ("position_stream_interval_ms"))
    {
        setRangedInt (cfg.position_stream_interval_ms, "position_stream_interval_ms",
                      fmu["position_stream_interval_ms"], 50, 60000);
    }
    if (fmu.isMember ("battery_stream_interval_ms"))
    {
        setRangedInt (cfg.battery_stream_interval_ms, "battery_stream_interval_ms", fmu["battery_stream_interval_ms"],
                      50, 60000);
    }
    if (fmu.isMember ("smm_position_report_interval_ms"))
    {
        setRangedInt (cfg.smm_position_report_interval_ms, "smm_position_report_interval_ms",
                      fmu["smm_position_report_interval_ms"], 100, 60000);
    }
    /* Upper bounds match the smm-asset library defaults (30s connect / 60s
     * transfer): a configured value above them would only ever loosen the bound,
     * which defeats the point, so they are capped there. */
    if (fmu.isMember ("smm_connect_timeout_s"))
    {
        setRangedInt (cfg.smm_connect_timeout_s, "smm_connect_timeout_s", fmu["smm_connect_timeout_s"], 1, 30);
    }
    if (fmu.isMember ("smm_transfer_timeout_s"))
    {
        setRangedInt (cfg.smm_transfer_timeout_s, "smm_transfer_timeout_s", fmu["smm_transfer_timeout_s"], 1, 60);
    }

    /* The MAV endpoint is safety-relevant: silently falling back to the default
     * could connect the FMU to the wrong (or no) autopilot. So, unlike the other
     * tunables, a present-but-invalid mav_address/mav_port fails hard (throws,
     * caught by main() which reports "Fatal" and exits) rather than warning and
     * keeping the default. An absent key still uses the default endpoint. */
    if (fmu.isMember ("mav_address"))
    {
        const Json::Value &addr = fmu["mav_address"];
        if (!addr.isString ())
        {
            throw std::runtime_error ("Config (" + config_file + "): mav_address must be a string");
        }
        std::string s = addr.asString ();
        if (is_blank (s))
        {
            throw std::runtime_error ("Config (" + config_file + "): mav_address is empty or whitespace-only");
        }
        cfg.mav_address = s;
    }
    if (fmu.isMember ("mav_port"))
    {
        const Json::Value &port = fmu["mav_port"];
        if (!port.isIntegral ())
        {
            throw std::runtime_error ("Config (" + config_file + "): mav_port must be an integer");
        }
        int v = port.asInt ();
        if (v < 1 || v > 65535)
        {
            throw std::runtime_error ("Config (" + config_file + "): mav_port (" + std::to_string (v)
                                      + ") out of range [1, 65535]");
        }
        cfg.mav_port = v;
    }
    if (fmu.isMember ("mav_connect_timeout_s"))
    {
        setRangedInt (cfg.mav_connect_timeout_s, "mav_connect_timeout_s", fmu["mav_connect_timeout_s"], 1, 30);
    }
    if (fmu.isMember ("mav_send_timeout_s"))
    {
        setRangedInt (cfg.mav_send_timeout_s, "mav_send_timeout_s", fmu["mav_send_timeout_s"], 1, 10);
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

    if (fmu.isMember ("log_dir"))
    {
        const Json::Value &dir = fmu["log_dir"];
        if (!dir.isString ())
        {
            std::cerr << "Config: log_dir is not a string, using default " << cfg.log_dir << "\n";
        }
        else if (std::string s = dir.asString (); s.empty ())
        {
            std::cerr << "Config: log_dir is empty, using default " << cfg.log_dir << "\n";
        }
        else
        {
            cfg.log_dir = s;
        }
    }

    return cfg;
}
