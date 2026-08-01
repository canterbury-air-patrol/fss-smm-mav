#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

#include "fmu-core-types.hpp"
#include "fmu-state-types.hpp"
#include "fss/fmu-fss-types.hpp"
#include "ilogger.hpp"
#include "smm/smm-command.hpp"

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
        case fmu_state_waiting_for_tasking:
            return "waiting_for_tasking";
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

inline const char *
smm_cmd_name (SMMCommand cmd)
{
    switch (cmd)
    {
        case smm_cmd_none:
            return "none";
        case smm_cmd_abandon_search:
            return "abandon_search";
        case smm_cmd_mission_complete:
            return "mission_complete";
    }
    return "unknown";
}

/* Rendered into every log line by Logger::log(), which is what lets call
 * sites state severity once — in the level argument — instead of repeating
 * it in the message text (todo/109). Upper case so it reads distinctly from
 * the category tags (STATE/CMD/COMMS/...) that follow it. */
inline const char *
log_level_name (LogLevel level)
{
    switch (level)
    {
        case LogLevel::error:
            return "ERROR";
        case LogLevel::warning:
            return "WARNING";
        case LogLevel::info:
            return "INFO";
        case LogLevel::debug:
            return "DEBUG";
    }
    return "UNKNOWN";
}

/* Every diagnostic (STATE/CMD/COMMS/BATTERY/...) formats and enqueues a line
 * from log() and returns immediately; a dedicated worker thread does the
 * actual write/flush/rotation. This mirrors SMM's and FSS's worker pattern
 * (todo/33, todo/51): several EventDispatcher branches call log() before the
 * state-machine method that commands the autopilot (e.g. the FSS command
 * event logs before FSSNewCommand(), which is what sends rtl/terminate/etc.),
 * so a blocking log() — a full disk, a wedged network log_dir, an in-flight
 * rotation — would otherwise stall the event-loop thread that arbitrates
 * every flight-safety command (todo/88). */
class Logger : public ILogger
{
  public:
    /* Default size threshold (bytes) at which log() rotates the file
     * in-flight; overridable (e.g. by tests) via the constructor. */
    static constexpr std::size_t default_max_log_bytes = 10UL * 1024 * 1024;

    explicit Logger (std::string_view dir, LogLevel level = LogLevel::info,
                     std::size_t max_bytes = default_max_log_bytes);
    Logger (const Logger &) = delete;
    Logger (Logger &&) = delete;
    auto operator= (const Logger &) -> Logger & = delete;
    auto operator= (Logger &&) -> Logger & = delete;
    /* Stops and joins the worker thread, draining any lines still queued
     * (same drain-on-shutdown pattern as SMM/FSS) before the file is closed. */
    ~Logger () override;

    using ILogger::log;

    /* Format the line (timestamp + message; no I/O) and enqueue it for the
     * worker thread. Emitted only if msg_level passes the configured
     * verbosity. Never blocks on disk I/O. */
    void log (LogLevel msg_level, std::string_view msg) override;

    /* Test-only synchronization point: block until every line enqueued so far
     * has actually been written to disk. Production code never calls this —
     * log() intentionally returns without waiting on I/O — but a test that
     * reads the log file right after logging needs to know the worker has
     * caught up. */
    void flush ();

  private:
    static std::string timestamp ();
    static void rotate (const std::string &base, int rotation_count);
    /* Close, rotate, and reopen the log file, resetting the byte counter.
     * Shared by the constructor's startup rotation and the worker's in-flight
     * rotation once max_log_bytes is exceeded. Worker-thread-only once the
     * worker has started (see workerLoop). */
    void openFresh ();
    /* Runs on the dedicated worker thread for the lifetime of the Logger:
     * dequeues one formatted line at a time and writes it, so log() itself
     * never touches `file`. */
    void workerLoop ();

  protected:
    /* The actual blocking write/flush/rotation for one already-formatted
     * line. Worker-thread-only. Virtual, mirroring SMM's fetchSearch/
     * commitSearch seams: a test can override it to observe or gate a write
     * (e.g. to prove log() itself never waits on one) without a real slow
     * disk. The default does the real write. */
    virtual void writeLine (const std::string &line);

    /* Stops and joins the worker thread if not already done; idempotent.
     * Logger::~Logger() calls this. A subclass that overrides writeLine()
     * and owns state the override reads/writes (e.g. a test double gating a
     * write on its own member) MUST call this as the very first statement
     * in its own destructor, before any of its members are destroyed:
     * writeLine() is virtual and runs on the worker thread, which is not
     * guaranteed stopped until this returns, so the worker could otherwise
     * still be inside the override while the derived object's own
     * destructor is tearing down the members it reads (a background-thread-
     * vs-destructor race — flagged in PR 220 review). */
    void stopWorker ();

  private:
    static constexpr int max_rotations = 5;

    std::string log_path;
    std::ofstream file;
    LogLevel level;
    /* A long-running process would otherwise append to a single file
     * forever (rotation previously only ran at startup, so the "5
     * rotations" retention was really "5 process starts", not a size or
     * time bound). Once the current file reaches this many bytes, the
     * worker rotates it like a restart would. */
    std::size_t max_log_bytes;
    /* Bytes written to the current file, tracked incrementally rather than
     * stat-ing the file on every write. Worker-thread-only. */
    std::size_t bytes_written{ 0 };

    /* Guards line_queue and busy (see below); queue_cv wakes the worker on a
     * new line or shutdown, idle_cv wakes flush() once the worker has fully
     * caught up. */
    std::mutex queue_lock{};
    std::condition_variable queue_cv{};
    std::condition_variable idle_cv{};
    std::deque<std::string> line_queue{};
    bool worker_running{ true };
    /* Set by stopWorker() so a second call (e.g. Logger::~Logger() after a
     * subclass destructor already called it) is a no-op rather than trying
     * to join an already-joined thread (which would throw). Guarded by
     * queue_lock. */
    bool worker_stopped{ false };
    /* True while the worker is between dequeuing a line and finishing its
     * write, so flush() can tell "queue empty" from "queue empty because the
     * last line is still being written" apart. Guarded by queue_lock. */
    bool busy{ false };
    std::thread worker_thread{};
};
