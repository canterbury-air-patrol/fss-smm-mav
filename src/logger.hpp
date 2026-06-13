#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <string_view>

#include "fmu.hpp"
#include "fss/fmu-fss-types.hpp"

inline const char *
fmu_state_name (FMUState s)
{
    switch (s)
    {
        case fmu_state_manual:
            return "manual";
        case fmu_state_searching:
            return "searching";
        case fmu_state_goto:
            return "goto";
        case fmu_state_altitude_adjust:
            return "altitude_adjust";
        case fmu_state_rtl:
            return "rtl";
        case fmu_state_hold:
            return "hold";
        case fmu_state_low_battery:
            return "low_battery";
        case fmu_state_failsafe:
            return "failsafe";
        case fmu_state_disarmed:
            return "disarmed";
        case fmu_state_terminate:
            return "terminate";
    }
    return "unknown";
}

inline const char *
fss_cmd_name (FSSCommand cmd)
{
    switch (cmd)
    {
        case fss_cmd_unknown:
            return "unknown";
        case fss_cmd_manual:
            return "manual";
        case fss_cmd_rtl:
            return "rtl";
        case fss_cmd_hold:
            return "hold";
        case fss_cmd_altitude:
            return "altitude";
        case fss_cmd_goto:
            return "goto";
        case fss_cmd_continue:
            return "continue";
        case fss_cmd_disarm:
            return "disarm";
        case fss_cmd_terminate:
            return "terminate";
    }
    return "unknown";
}

class Logger
{
  public:
    explicit Logger (std::string_view dir);

    void log (std::string_view msg);

  private:
    static std::string timestamp ();
    static void rotate (const std::string &base, int max_rotations);

    std::ofstream file;
    std::mutex lock;
};
