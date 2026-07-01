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
#include "internal.hpp"

class FSS
{
  private:
    std::shared_ptr<fss_client_ssl> ssl_client{ nullptr };

    /* Every outbound FSS send the event loop makes (position/reached/battery
     * reports and the second-phase command ack) bottoms out in a blocking send()
     * on a blocking socket (flight-safety-system transport.cpp / transport-ssl.cpp).
     * A half-dead FSS peer fills the kernel send buffer and send() blocks until it
     * drains or the connection's TCP_USER_TIMEOUT (30s, set in the transport)
     * errors it out. Doing these on the event-loop thread would therefore stall
     * queued rtl/terminate commands behind a hung peer for up to ~30s. The worker
     * below takes the same treatment SMM got (todo/33): the public report/postAck
     * methods only enqueue a task and return, and this thread does the blocking
     * send. A tighter, configurable send timeout (so even the worker can't wedge
     * for the full 30s) is tracked upstream in flight-safety-system. */
    struct FssPositionTask
    {
        double lat{ 0.0 };
        double lng{ 0.0 };
        int16_t alt{ 0 };
        uint16_t heading{ 0 };
        uint16_t hor_vel{ 0 };
        int16_t ver_vel{ 0 };
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
    using FssTask = std::variant<FssPositionTask, FssReachedTask, FssBatteryTask, FssAckTask>;

    std::thread worker_thread{};
    std::mutex queue_lock{};
    std::condition_variable queue_cv{};
    std::deque<FssTask> task_queue{};
    bool worker_running{ true };

    void enqueue (FssTask task);
    void workerLoop ();

  public:
    explicit FSS (const std::string &config_file);
    FSS (FSS &) = delete;
    FSS (FSS &&) = delete;
    auto operator= (FSS &) -> FSS & = delete;
    auto operator= (FSS &&) -> FSS & = delete;
    ~FSS ();
    auto getAssetName () -> std::string;
    void reportPosition (PositionData pd);
    void registerCommandCB (notify_fss_command_cb cb);
    void registerCommsStatusCB (notify_fss_comms_cb cb);
    void registerSMMSettingsCB (notify_smm_settings_cb cb);
    void registerPositionDataCB (notify_position_cb cb);
    void reachedPoint (int point, int total_points);
    void reportBatteryStatus (BatteryData bd);
    /* Enqueue the resolved second-phase command ack for the worker to send. */
    void postAck (const fss_command_ack_responder &ack, const FSSCommandResolution &res);
    void reconnectAll ();
};
