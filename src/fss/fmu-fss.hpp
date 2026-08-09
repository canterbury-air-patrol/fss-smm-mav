#pragma once
#include "fmu-fss-types.hpp"
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <variant>

#include "../fmu-types.hpp"
#include "ifss.hpp"
#include "internal.hpp"

class FSS : public IFSSReporter
{
    /* Test-only accessor: lets the send-worker tests peek task_queue under
     * queue_lock, since the only production path to it is enqueue(). */
    friend struct FSSTestAccess;

  private:
    std::shared_ptr<fss_client_ssl> ssl_client{ nullptr };

  protected:
    /* Outbound FSS sends bottom out in a blocking send() on a blocking socket
     * (flight-safety-system transport.cpp / transport-ssl.cpp). A half-dead FSS
     * peer fills the kernel send buffer and send() blocks until it drains or the
     * connection's TCP_USER_TIMEOUT (30s, set in the transport) errors it out.
     * Doing these on the event-loop thread would therefore stall queued
     * rtl/terminate commands behind a hung peer for up to ~30s. The worker below
     * takes the same treatment SMM got (todo/33): the public report/postAck
     * methods only enqueue a task and return, and this thread does the blocking
     * send.
     *
     * The four paths are no longer alike upstream. position/reached/battery fan
     * out through fss_client::sendMsgAll(), which as of the client library's
     * todo/66 (docs/decisions/66-67-client-outbound-fanout.md) packs once and
     * hands the frame to each server's own outbound worker — non-blocking, with
     * a bounded drop-oldest queue per server. The second-phase command ack is a
     * *per-connection* sendMsg() on the originating connection, which that
     * decision deliberately keeps inline, so it stays blocking. The floor is
     * now fss-client-ssl >= 1.3.0, which is that release — so the ack is the
     * path that still earns this worker, and it is the one that closes the loop
     * on an operator's rtl/terminate. */
    struct FssPositionTask
    {
        double lat{ 0.0 };
        double lng{ 0.0 };
        int16_t alt{ 0 };
        uint16_t heading{ 0 };
        uint16_t hor_vel{ 0 };
        int16_t ver_vel{ 0 };
        /* Whether lat/lng are backed by a valid GPS fix (todo/79); forwarded to
         * the wire report's valid_fields so FSS-Web can show "no fix" instead of
         * a stale position re-reported as current. */
        bool fix_valid{ true };
    };
    struct FssReachedTask
    {
        int point{ 0 };
        int total_points{ 0 };
    };
    struct FssBatteryTask
    {
        int8_t remaining{ 0 };
        int32_t consumed{ 0 };
        double voltage{ 0.0 };
    };
    /* The resolved second-phase command ack. The responder closes over the
     * originating connection (weak) and command group; running it on the worker
     * just moves its blocking send off the event loop. */
    struct FssAckTask
    {
        fss_command_ack_responder ack{};
        FSSCommandResolution res{};
    };

  private:
    using FssTask = std::variant<FssPositionTask, FssReachedTask, FssBatteryTask, FssAckTask>;

    std::thread worker_thread{};
    std::mutex queue_lock{};
    std::condition_variable queue_cv{};
    std::deque<FssTask> task_queue{};
    bool worker_running{ true };

    void enqueue (FssTask task);
    void workerLoop ();

  protected:
    /* Seams over the blocking sends the worker makes, so a test can make one
     * slow and observable without a live peer (the same shape as SMM's
     * fetchSearch/reportPositionToSmm seams, todo/33). Defaults call straight
     * through. Each guards ssl_client itself rather than the dispatch guarding
     * it once: sendAck runs a plain closure and has nothing to do with the
     * client, so it must still fire when there is no client at all. */
    virtual void sendPosition (const FssPositionTask &t);
    virtual void sendReached (const FssReachedTask &t);
    virtual void sendBattery (const FssBatteryTask &t);
    virtual void sendAck (const FssAckTask &t);

    /* Drain the queue, then stop and join the worker. Idempotent. ~FSS calls
     * it, but a subclass that overrides the seams above MUST also call it from
     * its own destructor: the worker calls the seams, and a derived object is
     * destroyed derived-part-first, so a worker still running by the time ~FSS
     * gets control would be calling into an already-destroyed subclass. The
     * base class cannot join early enough on its own — it does not get control
     * until the derived destructor has finished. */
    void stopWorker ();

  public:
    explicit FSS (const std::string &config_file);
    FSS (FSS &) = delete;
    FSS (FSS &&) = delete;
    auto operator= (FSS &) -> FSS & = delete;
    auto operator= (FSS &&) -> FSS & = delete;
    ~FSS () override;
    auto getAssetName () -> std::string;
    void reportPosition (PositionData pd) override;
    void registerCommandCB (notify_fss_command_cb cb);
    void registerCommsStatusCB (notify_fss_comms_cb cb);
    void registerSMMSettingsCB (notify_smm_settings_cb cb);
    void registerPositionDataCB (notify_position_cb cb);
    void reachedPoint (int point, int total_points) override;
    void reportBatteryStatus (BatteryData bd) override;
    /* Enqueue the resolved second-phase command ack for the worker to send. */
    void postAck (const fss_command_ack_responder &ack, const FSSCommandResolution &res) override;
    void reconnectAll ();
};
