#include "fmu-config.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <json/json.h>

namespace {
constexpr double feet_to_metres = 0.3048;
} // namespace

auto
loadAssetConfig (const std::string &config_file) -> AssetConfig
{
    AssetConfig cfg{};

    std::ifstream f (config_file);
    if (!f.is_open ())
    {
        std::cerr << "Config: unable to open " << config_file << ", using defaults\n";
        return cfg;
    }

    /* jsoncpp throws Json::Exception on malformed JSON or wrong-typed
     * accessors; fall back to defaults rather than aborting the FMU. */
    try
    {
        Json::Value root;
        f >> root;
        const Json::Value &fmu = root["fmu"];
        if (fmu.isObject ())
        {
            /* The cap may be given in metres or feet; feet is the usual unit
             * for the regulatory ceiling (e.g. 400ft). Both are stored
             * internally as metres. If both are present, feet wins. */
            bool have_cap_m = fmu.isMember ("altitude_cap_m");
            bool have_cap_ft = fmu.isMember ("altitude_cap_ft");
            if (have_cap_m && have_cap_ft)
            {
                std::cerr << "Config: both altitude_cap_m and altitude_cap_ft set, using altitude_cap_ft\n";
            }
            if (have_cap_ft)
            {
                cfg.altitude_cap_m
                    = static_cast<uint16_t> (std::lround (fmu["altitude_cap_ft"].asDouble () * feet_to_metres));
            }
            else if (have_cap_m)
            {
                cfg.altitude_cap_m = static_cast<uint16_t> (fmu["altitude_cap_m"].asUInt ());
            }
            if (fmu.isMember ("camera_fov_deg"))
            {
                double fov = fmu["camera_fov_deg"].asDouble ();
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
    }
    catch (const Json::Exception &e)
    {
        std::cerr << "Config: failed to parse " << config_file << ": " << e.what () << ", using defaults\n";
        return AssetConfig{};
    }

    return cfg;
}
