#include "fmu-config.hpp"

#include <fstream>
#include <iostream>
#include <json/json.h>

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
            if (fmu.isMember ("altitude_cap_m"))
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
