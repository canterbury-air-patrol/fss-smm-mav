#include "logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

Logger::Logger (std::string_view dir, LogLevel t_level, std::size_t max_bytes)
    : log_path{}, file{}, level (t_level), max_log_bytes (max_bytes)
{
    std::string log_dir (dir);
    log_path = log_dir + "/fmu.log";

    std::error_code ec;
    std::filesystem::create_directories (log_dir, ec);
    if (ec)
    {
        std::cerr << "logger: cannot create " << log_dir << ": " << ec.message () << '\n';
        return;
    }

    /* Single-threaded so far (the worker below does not exist yet): safe to
     * open/rotate directly here. */
    openFresh ();
    this->worker_thread = std::thread (&Logger::workerLoop, this);
}

Logger::~Logger ()
{
    /* Signal shutdown and join: workerLoop() keeps draining line_queue even
     * after worker_running is false (its exit condition is "not running AND
     * empty"), so every line already enqueued before this destructor runs is
     * still written before the file is closed. */
    {
        std::lock_guard<std::mutex> lk (this->queue_lock);
        this->worker_running = false;
    }
    this->queue_cv.notify_one ();
    if (this->worker_thread.joinable ())
    {
        this->worker_thread.join ();
    }
}

void
Logger::openFresh ()
{
    if (file.is_open ())
    {
        file.close ();
    }
    rotate (log_path, max_rotations);
    file.open (log_path, std::ios::out | std::ios::trunc);
    bytes_written = 0;
    if (!file.is_open ())
    {
        std::cerr << "logger: cannot open " << log_path << '\n';
    }
}

void
Logger::rotate (const std::string &base, int rotation_count)
{
    std::error_code ec;
    for (int i = rotation_count - 1; i >= 1; --i)
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
Logger::log (LogLevel msg_level, std::string_view msg)
{
    /* Higher enum value == more verbose; drop anything above the configured
     * level. The on-disk line format is unchanged so existing log consumers
     * keep working regardless of level. */
    if (msg_level > level)
    {
        return;
    }
    /* Formatting is pure CPU work (chrono + gmtime_r + ostringstream), not
     * I/O: safe to do on the caller's thread. The write itself is not
     * (todo/88), so only the formatted line crosses onto the queue. */
    std::string line = timestamp () + ' ' + std::string (msg) + '\n';
    {
        std::lock_guard<std::mutex> lk (this->queue_lock);
        this->line_queue.push_back (std::move (line));
    }
    this->queue_cv.notify_one ();
}

void
Logger::writeLine (const std::string &line)
{
    if (!file.is_open ())
    {
        return;
    }
    file << line;
    file.flush ();
    bytes_written += line.size ();
    /* A long-running process previously rotated only at startup (so "5
     * rotations" retention was really "5 process starts"). Rotate here too
     * once the current file crosses the size threshold, same as a restart
     * would, so a long flight or a chatty debug level cannot grow the file
     * without bound. */
    if (bytes_written >= max_log_bytes)
    {
        openFresh ();
    }
}

void
Logger::workerLoop ()
{
    while (true)
    {
        std::string line;
        {
            std::unique_lock<std::mutex> lk (this->queue_lock);
            this->queue_cv.wait (lk, [this] { return !this->line_queue.empty () || !this->worker_running; });
            if (!this->worker_running && this->line_queue.empty ())
            {
                return;
            }
            line = std::move (this->line_queue.front ());
            this->line_queue.pop_front ();
            this->busy = true;
        }
        this->writeLine (line);
        {
            std::lock_guard<std::mutex> lk (this->queue_lock);
            this->busy = false;
            if (this->line_queue.empty ())
            {
                this->idle_cv.notify_all ();
            }
        }
    }
}

void
Logger::flush ()
{
    std::unique_lock<std::mutex> lk (this->queue_lock);
    this->idle_cv.wait (lk, [this] { return this->line_queue.empty () && !this->busy; });
}
