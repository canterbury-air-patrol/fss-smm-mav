#pragma once

#include <cstddef>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>

#include "fmu-core-types.hpp"
#include "fmu-state-types.hpp"
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
    /* Default size threshold (bytes) at which log() rotates the file
     * in-flight; overridable (e.g. by tests) via the constructor. */
    static constexpr std::size_t default_max_log_bytes = 10UL * 1024 * 1024;

    explicit Logger (std::string_view dir, LogLevel level = LogLevel::info,
                     std::size_t max_bytes = default_max_log_bytes);

    /* Log at info level. */
    void log (std::string_view msg);
    /* Log at an explicit level; emitted only if it passes the configured
     * verbosity. */
    void log (LogLevel msg_level, std::string_view msg);

  private:
    static std::string timestamp ();
    static void rotate (const std::string &base, int rotation_count);
    /* Close, rotate, and reopen the log file, resetting the byte counter.
     * Shared by the constructor's startup rotation and log()'s in-flight
     * rotation once max_log_bytes is exceeded. */
    void openFresh ();

    static constexpr int max_rotations = 5;

    std::string log_path;
    std::ofstream file;
    std::mutex lock;
    LogLevel level;
    /* A long-running process would otherwise append to a single file
     * forever (rotation previously only ran at startup, so the "5
     * rotations" retention was really "5 process starts", not a size or
     * time bound). Once the current file reaches this many bytes, log()
     * rotates it like a restart would. */
    std::size_t max_log_bytes;
    /* Bytes written to the current file, tracked incrementally rather than
     * stat-ing the file on every log() call. */
    std::size_t bytes_written{ 0 };
};
