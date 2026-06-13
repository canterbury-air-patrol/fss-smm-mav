#include "logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

Logger::Logger (std::string_view dir, LogLevel t_level) : file{}, lock{}, level (t_level)
{
    std::string log_dir (dir);
    std::string log_path = log_dir + "/fmu.log";

    std::error_code ec;
    std::filesystem::create_directories (log_dir, ec);
    if (ec)
    {
        std::cerr << "logger: cannot create " << log_dir << ": " << ec.message () << '\n';
        return;
    }

    rotate (log_path, 5);

    file.open (log_path, std::ios::out | std::ios::trunc);
    if (!file.is_open ())
    {
        std::cerr << "logger: cannot open " << log_path << '\n';
    }
}

void
Logger::rotate (const std::string &base, int max_rotations)
{
    std::error_code ec;
    for (int i = max_rotations - 1; i >= 1; --i)
    {
        std::string src = base + "." + std::to_string (i);
        std::string dst = base + "." + std::to_string (i + 1);
        if (std::filesystem::exists (src, ec))
        {
            std::filesystem::rename (src, dst, ec);
        }
    }
    if (std::filesystem::exists (base, ec))
    {
        std::filesystem::rename (base, base + ".1", ec);
    }
}

std::string
Logger::timestamp ()
{
    auto now = std::chrono::system_clock::now ();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds> (now.time_since_epoch ()) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t (now);
    std::tm tm_buf{};
    gmtime_r (&t, &tm_buf);

    std::ostringstream oss;
    oss << std::put_time (&tm_buf, "%Y-%m-%dT%H:%M:%S") << '.' << std::setfill ('0') << std::setw (3) << ms.count ()
        << 'Z';
    return oss.str ();
}

void
Logger::log (std::string_view msg)
{
    log (LogLevel::info, msg);
}

void
Logger::log (LogLevel msg_level, std::string_view msg)
{
    /* Higher enum value == more verbose; drop anything above the configured
     * level. The on-disk line format is unchanged so existing log consumers
     * keep working regardless of level. */
    if (msg_level > level)
    {
        return;
    }
    std::lock_guard<std::mutex> lk (lock);
    if (!file.is_open ())
    {
        return;
    }
    file << timestamp () << ' ' << msg << '\n';
    file.flush ();
}
