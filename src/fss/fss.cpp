#include "altitude-units.hpp"
#include "fmu-fss-types.hpp"
#include "fmu-fss.hpp"
#include "internal.hpp"
#include <cmath>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <variant>

namespace
{
template <class... Ts> struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts> overloaded (Ts...) -> overloaded<Ts...>;
} // namespace

auto
FSS::getAssetName () -> std::string
{
    if (this->ssl_client != nullptr)
    {
        return this->ssl_client->getAssetName ();
    }
    return "";
}

void
FSS::registerCommandCB (notify_fss_command_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerCommandCB (std::move (cb));
    }
};

void
FSS::registerCommsStatusCB (notify_fss_comms_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerCommsStatusCB (std::move (cb));
    }
};

void
FSS::registerSMMSettingsCB (notify_smm_settings_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerSMMSettingsCB (std::move (cb));
    }
}

void
FSS::registerPositionDataCB (notify_position_cb cb)
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->registerPositionDataCB (std::move (cb));
    }
}

void
FSS::reportPosition (PositionData t_pd)
{
    Point p = t_pd.getP ();
    /* PositionData carries metres; the FSS wire altitude is feet. std::lround
     * is undefined for a non-finite altitude, so guard it the same way
     * SMM::reportPosition does and report 0 for a bad reading. The conversion is
     * done here (pure, no I/O) so the queued task carries the final wire values
     * and the worker only does the blocking send. */
    const double alt_ft_d = metres_to_feet (t_pd.getAltitudeMetres ());
    auto alt_ft = std::isfinite (alt_ft_d) ? static_cast<int16_t> (std::lround (alt_ft_d)) : static_cast<int16_t> (0);
    bool fix_valid = (t_pd.getFlags () & POSITION_FLAG_VALID_COORDS) != 0;
    this->enqueue (FssPositionTask{ p.getLatitude (), p.getLongitude (), alt_ft, t_pd.getHeading (),
                                    t_pd.getVelocityHorizontal (), t_pd.getVelocityVertical (), fix_valid });
}

void
FSS::reachedPoint (int point, int total_points)
{
    this->enqueue (FssReachedTask{ point, total_points });
}

void
FSS::reportBatteryStatus (BatteryData bd)
{
    this->enqueue (FssBatteryTask{ bd.getRemaining (), bd.getConsumed (), bd.getVoltage () });
}

void
FSS::postAck (const fss_command_ack_responder &ack, const FSSCommandResolution &res)
{
    this->enqueue (FssAckTask{ ack, res });
}

void
FSS::enqueue (FssTask task)
{
    {
        std::lock_guard<std::mutex> lk (this->queue_lock);
        if (std::holds_alternative<FssPositionTask> (task))
        {
            /* Coalesce: a newer position supersedes any queued (unsent) report, so
             * a backlog cannot build while the worker is in a slow send. Reporting
             * the latest position is all that matters (same as SMM). reached-point,
             * battery, and acks each carry distinct meaning, so they are not
             * coalesced. */
            for (auto it = this->task_queue.begin (); it != this->task_queue.end ();)
            {
                it = std::holds_alternative<FssPositionTask> (*it) ? this->task_queue.erase (it) : std::next (it);
            }
        }
        this->task_queue.push_back (std::move (task));
    }
    this->queue_cv.notify_one ();
}

void
FSS::workerLoop ()
{
    while (true)
    {
        FssTask task;
        {
            std::unique_lock<std::mutex> lk (this->queue_lock);
            this->queue_cv.wait (lk, [this] { return !this->task_queue.empty () || !this->worker_running; });
            if (!this->worker_running && this->task_queue.empty ())
            {
                return;
            }
            task = std::move (this->task_queue.front ());
            this->task_queue.pop_front ();
        }
        if (this->ssl_client == nullptr)
        {
            continue;
        }
        std::visit (
            overloaded{
                [this] (const FssPositionTask &t)
                { this->ssl_client->sendPosition (t.lat, t.lng, t.alt, t.heading, t.hor_vel, t.ver_vel, t.fix_valid); },
                [this] (const FssReachedTask &t) { this->ssl_client->reachedPoint (t.point, t.total_points); },
                [this] (const FssBatteryTask &t)
                { this->ssl_client->sendBatteryStatus (t.remaining, t.consumed, t.voltage); },
                [] (const FssAckTask &t)
                {
                    if (t.ack)
                    {
                        t.ack (t.res);
                    }
                },
            },
            task);
    }
}

void
FSS::reconnectAll ()
{
    if (this->ssl_client != nullptr)
    {
        this->ssl_client->attemptReconnect ();
    }
}

FSS::FSS (const std::string &config_file)
{
    this->ssl_client = std::make_shared<fss_client_ssl> (config_file.c_str ());
    /* Start the send worker last, once ssl_client is fully constructed. No tasks
     * are enqueued until the App wires up events and starts reporting, so the
     * worker simply waits on an empty queue until then. */
    this->worker_thread = std::thread (&FSS::workerLoop, this);
}

FSS::~FSS ()
{
    /* Stop and join the worker before the members it touches (ssl_client) are
     * destroyed. A wedged peer can only delay shutdown by the time of one
     * in-flight blocking send; the upstream send timeout bounds even that. */
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
