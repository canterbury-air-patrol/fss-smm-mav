#pragma once

#include "../fmu-state-types.hpp"

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

/* Groups the redundant per-server deliveries of a single logical command so they
 * all receive the same terminal acknowledgement.
 *
 * The FMU connects to every FSS server at once and the web frontend pushes the
 * same command to each, so the same logical command arrives once per connection
 * (on different recv threads, with per-connection ids and possibly slightly
 * skewed timestamps). We action it exactly once and ack every copy with the same
 * outcome, so the operator sees a consistent result across the per-server rows
 * rather than 'actioned' on one and 'noop' on the rest.
 *
 * This class holds only the bookkeeping — identity matching, the pending-copy
 * list, the cached resolution and the epoch — and is deliberately free of any
 * transport type so it is unit-testable without the SSL stack. The owner
 * (fss_client_ssl) supplies the per-copy ack target as the template parameter
 * Target (e.g. a weak_ptr<fss_connection> + acked_id) and performs the actual
 * I/O on the targets this class hands back.
 *
 * Thread-safety: every public method locks the internal mutex. Deliveries arrive
 * on per-connection recv threads; the resolution is delivered from the
 * event-loop thread. Methods return the targets to ack by value so the caller
 * can do the (wire) I/O after the lock is released. */
template <typename Target> class CommandAckGroup
{
  public:
    enum class Disposition
    {
        /* First delivery of a new logical command: the caller must actuate it
         * once (enqueue the state-machine event). `superseded` carries any copies
         * of a now-displaced older command to ack as superseded; `epoch` is the
         * token to pass back to resolve(). */
        actuate,
        /* Redundant delivery whose outcome is already known: ack `resolution` to
         * this copy immediately. */
        already_resolved,
        /* Redundant delivery still awaiting the outcome: nothing to do now, the
         * copy was queued and will be acked when resolve() runs. */
        pending,
        /* A genuinely older, different command from a slower server: do NOT
         * actuate; ack it as superseded so it is not left without an ack. */
        stale_superseded,
    };

    struct DeliveryResult
    {
        Disposition disposition{ Disposition::pending };
        uint64_t epoch{ 0 };
        std::optional<FSSCommandResolution> resolution{};
        std::vector<Target> superseded{};
    };

    /* `t_tolerance_ms` is the window within which two timestamps count as the same
     * logical command. */
    explicit CommandAckGroup (uint64_t t_tolerance_ms) : tolerance_ms (t_tolerance_ms) {}

    /* `t_command` is an opaque command-identity value (the caller's enum), matched
     * for equality; `t_timestamp` is matched within the tolerance. */
    auto
    onDelivery (int t_command, uint64_t t_timestamp, Target copy) -> DeliveryResult
    {
        const std::scoped_lock lock (this->mtx);
        DeliveryResult result;

        if (this->active && this->command == t_command && withinTolerance (t_timestamp))
        {
            /* Redundant re-delivery of the command already being handled. */
            if (this->resolution.has_value ())
            {
                result.disposition = Disposition::already_resolved;
                result.resolution = this->resolution;
            }
            else
            {
                this->pending.push_back (std::move (copy));
                result.disposition = Disposition::pending;
            }
            return result;
        }

        if (this->active && this->command != t_command && t_timestamp < this->timestamp
            && (this->timestamp - t_timestamp) < this->tolerance_ms)
        {
            /* Older, different command: the newer one is already in effect. */
            result.disposition = Disposition::stale_superseded;
            return result;
        }

        /* A new logical command supersedes the current group. Hand back any copies
         * of the old command that never resolved so the caller acks them as
         * superseded, then open a fresh group with this copy as its first member. */
        result.superseded = std::move (this->pending);
        this->active = true;
        this->epoch++;
        this->command = t_command;
        this->timestamp = t_timestamp;
        this->pending.clear ();
        this->pending.push_back (std::move (copy));
        this->resolution.reset ();
        result.disposition = Disposition::actuate;
        result.epoch = this->epoch;
        return result;
    }

    /* The terminal outcome for the command opened at `t_epoch` has arrived. Caches
     * it and returns every copy still pending in that group so the caller acks
     * them. Returns empty if a newer command has already superseded this group
     * (its copies were handed back by onDelivery()). */
    auto
    resolve (uint64_t t_epoch, const FSSCommandResolution &res) -> std::vector<Target>
    {
        const std::scoped_lock lock (this->mtx);
        if (this->epoch != t_epoch)
        {
            return {};
        }
        this->resolution = res;
        std::vector<Target> to_ack;
        to_ack.swap (this->pending);
        return to_ack;
    }

  private:
    auto
    withinTolerance (uint64_t t_timestamp) const -> bool
    {
        uint64_t skew = t_timestamp >= this->timestamp ? t_timestamp - this->timestamp : this->timestamp - t_timestamp;
        return skew < this->tolerance_ms;
    }

    uint64_t tolerance_ms;
    std::mutex mtx{};
    bool active{ false };
    uint64_t epoch{ 0 };
    int command{ 0 };
    uint64_t timestamp{ 0 };
    std::vector<Target> pending{};
    std::optional<FSSCommandResolution> resolution{};
};
