#pragma once

#include "../fmu-state-types.hpp"

#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

/* Identity payload for payload-bearing FSS commands. Two deliveries of the same
 * command type are the same logical command only if their payloads also match,
 * so a re-targeted goto or a changed altitude opens a new group and actuates
 * rather than replaying the first command's cached acknowledgement.
 *
 * goto carries a target position (floating-point degrees, compared with a small
 * tolerance so wire round-trip noise is not mistaken for a new command);
 * altitude carries an integer altitude (compared exactly). Commands without a
 * payload use the default (Kind::none), which always compares equal. */
struct CommandPayload
{
    enum class Kind : uint8_t
    {
        none,
        position,
        altitude,
    };

    /* Tolerance for the goto target position comparison: ~1e-7 deg is roughly
     * 1 cm of latitude, well below the precision of any real retarget but above
     * wire round-trip noise. Named at struct scope so the dedup sensitivity is
     * visible and tunable in one place. */
    static constexpr double position_tolerance_deg = 1e-7;

    Kind kind{ Kind::none };
    double lat{ 0.0 };
    double lng{ 0.0 };
    int32_t altitude{ 0 };

    static auto
    forPosition (double t_lat, double t_lng) -> CommandPayload
    {
        CommandPayload payload;
        payload.kind = Kind::position;
        payload.lat = t_lat;
        payload.lng = t_lng;
        return payload;
    }

    static auto
    forAltitude (int32_t t_altitude) -> CommandPayload
    {
        CommandPayload payload;
        payload.kind = Kind::altitude;
        payload.altitude = t_altitude;
        return payload;
    }

    auto
    sameAs (const CommandPayload &other) const -> bool
    {
        if (this->kind != other.kind)
        {
            return false;
        }
        switch (this->kind)
        {
            case Kind::none:
                return true;
            case Kind::altitude:
                return this->altitude == other.altitude;
            case Kind::position:
                return std::fabs (this->lat - other.lat) < position_tolerance_deg
                       && std::fabs (this->lng - other.lng) < position_tolerance_deg;
        }
        return false;
    }
};

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
     * logical command (repeat deliveries of the same command/payload from
     * redundant servers). It does NOT bound staleness rejection: a different,
     * older command is always superseded regardless of how far outside this window
     * it falls. */
    explicit CommandAckGroup (uint64_t t_tolerance_ms) : tolerance_ms (t_tolerance_ms) {}

    /* `t_command` is an opaque command-identity value (the caller's enum), matched
     * for equality; `t_payload` is the payload for payload-bearing commands,
     * matched via CommandPayload::sameAs (so a changed goto target / altitude is a
     * new logical command, not a duplicate); `t_timestamp` is matched within the
     * tolerance.
     *
     * `t_server_command_id`/`t_connection_key` are the retry-vs-redundant signal:
     * the dispatching server's per-connection command identifier (0 = not
     * reported, e.g. a legacy peer or the capability was not negotiated) and an
     * opaque identifier for the server it arrived from (the caller supplies its
     * endpoint). Per the upstream contract, ids are comparable only within one
     * connection, never across connections — so this only ever compares a
     * delivery's id against what THAT SAME key last reported for the active
     * group, never against another server's id.
     *
     * The key must name the server itself and must not be recycled between two
     * different servers — an id compared against a stale entry left behind by
     * some other server is exactly the cross-server comparison the contract
     * forbids, and would read either as a retry that never happened (the group
     * re-actuates) or as a redelivery of an action already handled (a genuine
     * operator retry is swallowed). */
    auto
    onDelivery (int t_command, uint64_t t_timestamp, Target copy, const CommandPayload &t_payload = {},
                uint64_t t_server_command_id = 0, const std::string &t_connection_key = {}) -> DeliveryResult
    {
        const std::scoped_lock lock (this->mtx);
        DeliveryResult result;

        const bool same_logical_command = this->command == t_command && this->payload.sameAs (t_payload);

        /* A deliberate operator retry: this connection previously reported a
         * different id for the currently active group. A fresh id from a
         * connection we have NOT yet seen this epoch is not a retry — it is the
         * expected first copy of the same operator action arriving from another
         * server (ids are not comparable across connections, so there is nothing
         * to compare it against). */
        bool deliberate_retry = false;
        if (this->active && t_server_command_id != 0)
        {
            auto seen = this->last_id_by_connection.find (t_connection_key);
            if (seen != this->last_id_by_connection.end () && seen->second != t_server_command_id)
            {
                deliberate_retry = true;
            }
        }

        if (this->active && same_logical_command && !deliberate_retry && withinTolerance (t_timestamp))
        {
            if (t_server_command_id != 0)
            {
                this->last_id_by_connection[t_connection_key] = t_server_command_id;
            }
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

        if (this->active && !same_logical_command && !deliberate_retry && t_timestamp < this->timestamp)
        {
            /* Older, different command (different type or different payload): the
             * newer one is already in effect. This is deliberately NOT bounded by
             * `tolerance_ms`: that window exists only to recognise repeat
             * deliveries of the SAME logical command across redundant servers,
             * not to cap how stale a *different* one has to be before it stops
             * being stale. A delayed/replayed delivery of an old command — worst
             * case an old terminate, which latches permanently — must never be
             * actuated as new just because the gap to the current command exceeds
             * the dedup window, whether that gap is 61 seconds or several hours.
             */
            result.disposition = Disposition::stale_superseded;
            return result;
        }

        /* A new logical command supersedes the current group — either a different
         * command/payload/newer timestamp, or the same command/payload but a
         * fresh id from a connection that already reported a different one for
         * the active group, i.e. a deliberate operator retry. Hand back any
         * copies of the old command that never resolved so the caller acks them
         * as superseded, then open a fresh group with this copy as its first
         * member. */
        result.superseded = std::move (this->pending);
        this->active = true;
        this->epoch++;
        this->command = t_command;
        this->payload = t_payload;
        this->timestamp = t_timestamp;
        this->pending.clear ();
        this->pending.push_back (std::move (copy));
        this->resolution.reset ();
        this->last_id_by_connection.clear ();
        if (t_server_command_id != 0)
        {
            this->last_id_by_connection[t_connection_key] = t_server_command_id;
        }
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
    CommandPayload payload{};
    uint64_t timestamp{ 0 };
    std::vector<Target> pending{};
    std::optional<FSSCommandResolution> resolution{};
    /* Last server_command_id reported by each connection (keyed by the caller's
     * opaque `t_connection_key`) for the currently active group. Reset whenever
     * a new logical command opens, since ids from a superseded group are no
     * longer meaningful. Only ever compared within the same key — never across
     * keys — because ids are not comparable across connections. */
    std::unordered_map<std::string, uint64_t> last_id_by_connection{};
};
