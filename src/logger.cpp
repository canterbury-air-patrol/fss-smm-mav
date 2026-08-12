#include "logger.hpp"

#include "log-rotation.hpp"
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

Logger::Logger (std::string_view dir, LogLevel t_level, std::size_t max_bytes, std::size_t max_queued)
    /* Floor the bound at 1: log()'s drop-oldest step pops before it pushes, so
     * a bound of 0 would pop from an empty deque. A caller asking for 0 wants
     * "queue as little as possible", and 1 is the smallest bound that means
     * anything. */
    : log_path{}, file{}, level (t_level), max_log_bytes (max_bytes), max_queued_lines (max_queued > 0 ? max_queued : 1)
{
    std::string log_dir (dir);
    log_path = log_dir + "/fmu.log";

    std::error_code ec;
    std::filesystem::create_directories (log_dir, ec);
    if (ec)
    {
        /* Leaves logging_enabled false: no worker thread is started, and log()
         * returns at its first line rather than formatting and queueing output
         * that provably cannot be written. See the flag's declaration. */
        std::cerr << "logger: cannot create " << log_dir << ": " << ec.message () << '\n';
        return;
    }

    /* Single-threaded so far (the worker below does not exist yet): safe to
     * open/rotate directly here. */
    openFresh ();
    if (!file.is_open ())
    {
        /* The directory exists but the file underneath it does not open (a
         * read-only mount, a full filesystem, fmu.log occupied by a directory
         * that rotation could not move aside). openFresh() has already
         * complained on stderr, so nothing more is said here.
         *
         * Disposition is deliberately identical to the create_directories()
         * failure above: both mean "logging could not be set up at startup",
         * and it would be arbitrary for one to skip cheaply while the other
         * formatted and queued every line for a worker whose writeLine()
         * discards it (!file.is_open() is that function's first test). Nothing
         * reopens the file later either --- writeLine() returns before
         * bytes_written can advance, so the rotation that calls openFresh()
         * again is unreachable --- so this is as permanent as the directory
         * failure, not a transient the worker might ride out. */
        return;
    }
    this->logging_enabled = true;
    this->worker_thread = std::thread (&Logger::workerLoop, this);
}

Logger::~Logger () { this->stopWorker (); }

void
Logger::stopWorker ()
{
    /* Idempotent: a subclass destructor may have already called this (see
     * the doc on the declaration) before this runs again from ~Logger().
     * Also safe when logging is disabled and no worker was ever started: the
     * flag work below is harmless and worker_thread is not joinable. */
    {
        std::lock_guard<std::mutex> lk (this->queue_lock);
        if (this->worker_stopped)
        {
            return;
        }
        this->worker_stopped = true;
        /* Signal shutdown: workerLoop() keeps draining line_queue even after
         * worker_running is false (its exit condition is "not running AND
         * empty"), so every line already enqueued before this call is still
         * written before the file is closed. */
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
    /* Logging could not be set up at all, so there is no worker and no file:
     * return before the formatting work below, which would otherwise be spent
     * on a line with nowhere to go. Checked ahead of the level test because it
     * is the more absolute of the two. */
    if (!this->logging_enabled)
    {
        return;
    }
    /* Higher enum value == more verbose; drop anything above the configured
     * level. */
    if (msg_level > level)
    {
        return;
    }
    /* Formatting is pure CPU work (chrono + gmtime_r + ostringstream), not
     * I/O: safe to do on the caller's thread. The write itself is not, so
     * only the formatted line crosses onto the queue.
     *
     * The level goes in the line rather than in the message text, so a call
     * site states severity exactly once and the two cannot drift. This is
     * the one change to the on-disk format: lines gained a level token
     * between the timestamp and the message. */
    std::string line = timestamp () + ' ' + log_level_name (msg_level) + ' ' + std::string (msg) + '\n';
    {
        std::lock_guard<std::mutex> lk (this->queue_lock);
        /* At the bound, drop the OLDEST queued line rather than refusing the
         * new one. Whatever has wedged the log device will not be fixed by
         * preserving the start of the backlog, and during an incident the most
         * recent lines are the ones worth keeping. Blocking here instead is not
         * an option: it would reintroduce exactly the event-loop stall that
         * moving the I/O to this worker removed. */
        if (this->line_queue.size () >= this->max_queued_lines)
        {
            this->line_queue.pop_front ();
            this->dropped_lines++;
        }
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
    bool write_ok = file.good ();
    RotationState next = next_rotation_state (write_ok, bytes_written, line.size (), max_log_bytes);
    bytes_written = next.bytes_written;
    if (!write_ok)
    {
        /* The write did not reach the disk (full filesystem, disconnected
         * mount, a revoked handle). Counting these bytes would drive the
         * rotation below on output that does not exist, and rotation is
         * destructive: every max_log_bytes of *failed* writes would shift
         * fmu.log.1..5 along and discard fmu.log.5, so a disk-full event would
         * quietly erase the existing log history while writing nothing in its
         * place --- exactly when those logs matter most. Leave bytes_written
         * alone so that cannot happen.
         *
         * clear() the error state so a transient failure can recover on the
         * next line rather than latching logging off for the rest of the
         * flight. */
        file.clear ();
        if (!write_failed)
        {
            write_failed = true;
            std::cerr << "logger: write to " << log_path << " failed; log lines are being lost\n";
        }
        return;
    }
    if (write_failed)
    {
        write_failed = false;
        std::cerr << "logger: writes to " << log_path << " have recovered\n";
    }
    /* A long-running process previously rotated only at startup (so "5
     * rotations" retention was really "5 process starts"). Rotate here too
     * once the current file crosses the size threshold, same as a restart
     * would, so a long flight or a chatty debug level cannot grow the file
     * without bound. */
    if (next.rotate)
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
        std::size_t dropped = 0;
        {
            std::unique_lock<std::mutex> lk (this->queue_lock);
            this->queue_cv.wait (lk, [this] { return !this->line_queue.empty () || !this->worker_running; });
            if (!this->worker_running && this->line_queue.empty ())
            {
                return;
            }
            line = std::move (this->line_queue.front ());
            this->line_queue.pop_front ();
            /* Claim any drops accumulated since the last marker, so they are
             * reported exactly once even though log() may add more while the
             * write below is in progress. */
            dropped = this->dropped_lines;
            this->dropped_lines = 0;
            this->busy = true;
        }
        if (dropped > 0)
        {
            /* Record the gap in the log itself. Written at error level: losing
             * flight diagnostics is a failure, and the marker is what stops a
             * reader mistaking the hole for a quiet period. */
            this->writeLine (timestamp () + ' ' + log_level_name (LogLevel::error) + " LOG dropped "
                             + std::to_string (dropped) + " line(s): the write queue reached its bound\n");
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
    /* Returns immediately when logging is disabled rather than waiting on a
     * worker that does not exist: log() enqueues nothing in that
     * configuration, so the predicate is already true on the first
     * evaluation. That is the whole reason log() returns early instead of
     * queueing lines nobody drains --- a single queued line with no worker
     * would park this wait forever. */
    std::unique_lock<std::mutex> lk (this->queue_lock);
    this->idle_cv.wait (lk, [this] { return this->line_queue.empty () && !this->busy; });
}
