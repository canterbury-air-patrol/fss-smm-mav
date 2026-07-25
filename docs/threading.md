# Threading and lock-ordering model

This is the consolidated statement of cap-fmu's concurrency design: the
thread inventory, which thread owns which data, the locks involved, and why
the whole thing is deadlock-free. Individual call sites carry local
rationale in comments; this document is the cross-thread picture those
comments point back to instead of restating (todo/62).

## Thread inventory

| Thread | Started in | Role |
|---|---|---|
| Event loop | `App::run()` (the process's main thread) | Dequeues `event`s and calls `EventDispatcher::dispatch()`, which drives `FMUStateMachine`. The sole arbiter of FMU state and priority (todo/33). |
| MAV recv | `mav_connection::connect_to_mav()` | Performs blocking reads and parses inbound MAVLink; invokes the position/battery/reached/comms callbacks that enqueue events for the event loop. |
| MAV heartbeat | `mav_connection` constructor path / `start()` | Sends periodic `HEARTBEAT`s and is the single authority for edge-triggering the MAV comms-up/down callback (`heartbeat_loop()`). |
| Signal waiter | `App::signal_waiter()` | Blocks in `sigwait()` for SIGINT/SIGTERM, then flips `App::running` and wakes the event loop and reconnector. |
| FSS reconnector | `App::fss_reconnector()` | Periodically calls `fss->reconnectAll()`, `mav->attemptReconnect()`, and `smm->retryPendingSearch()`. |
| SMM worker | `SMM` constructor / `workerLoop()` | Runs all blocking `smm_asset_*` HTTP I/O (connect, position report, search acquire/accept) off the event loop (todo/33). |
| FSS send worker | `FSS` constructor / `workerLoop()` | Runs all blocking FSS `send()`s (position/reached/battery reports, command acks) off the event loop, so a hung FSS peer cannot stall queued commands. |

That is 7 threads, matching the count referenced in todo/62 and elsewhere.

## Data ownership and locks

| Lock / atomic | Guards | Held by |
|---|---|---|
| `App::main_lock` + `main_cv` | `App::event_queue` | Event loop (consumer), every callback that calls `enqueue_event` (producers: MAV recv/heartbeat threads, SMM worker via its callbacks, FSS send worker, signal waiter, FSS reconnector). |
| `App::reconnect_lock` + `reconnect_cv` | Only the reconnector's own wait timer | FSS reconnector thread, woken early by the signal waiter. |
| `FMUStateMachine::lock` | `fss_command`, `smm_command`, `fss_command_target`, `low_battery`, `low_battery_count`, `altitude_breach`, `altitude_breach_count`, `altitude_clear_count`, `current_state`, `fss_comms_lost`, `mav_comms_lost`, `pending_replay_state` | Event loop only (todo/52's `assert_event_loop_thread()` enforces this in debug builds) — see "Single-event-loop-thread invariant" below. |
| `mav_connection::send_lock` | The socket fd's lifetime (open/close) and the per-channel MAVLink pack/transmit state (the generated `*_pack_chan()` calls mutate global per-channel sequence/status, so packing and sending must be serialised) | Any thread that sends: event loop (via `MAV`/`IMAV` action methods), MAV heartbeat thread, MAV recv thread (mission handshake replies, ADS-B rebroadcast). |
| `mav_connection::state_lock` | `last_position`, `search`, `search_loaded`, `search_loading`, `goto_active`, `goto_position`, `goto_ack_pending`, `pending_mode_command`, `retry_count`, `last_tried` | MAV recv thread (parses inbound MAVLink and updates upload/search state), event loop (issues commands), FSS reconnector (`attemptReconnect`). |
| `mav_connection::heartbeat_mutex` + `heartbeat_cv` | Only the heartbeat loop's own 1-second wait timer | MAV heartbeat thread, woken early by `stopping`. |
| `SMM::queue_lock` + `queue_cv` | `SMM::task_queue` | SMM worker (consumer); event loop and FSS reconnector (producers, via the public methods that call `enqueue`). |
| `SMM::state_lock` | `current_search`, `asset` | SMM worker (publishes), event loop (`currentSearchPoints()` reads the atomic mirror instead, see below), test-only injection (`SMMTestAccess`). |
| `FSS::queue_lock` + `queue_cv` | `FSS::task_queue` | FSS send worker (consumer); event loop (producer, via `reportPosition`/`reachedPoint`/`reportBatteryStatus`/`postAck`). |
| Atomics (`App::running`, `mav_connection::fd`/`mav_comms_ok`/`last_heartbeat_ts`/`broken`/`stopping`/`started`, `SMM::search_active`/`current_search_points`) | Single flags or counters read across threads without a critical section spanning more than the one load/store | Various — each is documented at its declaration; called out here because they are part of the concurrency picture even though they need no mutex. |

`Logger` has its own internal mutex and is safe to call from any thread
(subsystem diagnostics routed through `ILogger`, todo/59, run on whichever
thread produced them — e.g. MAV's heartbeat thread, SMM's worker thread).

## Lock acquisition order

**No function in this codebase holds two of the locks above at the same
time.** Every critical section is a single `std::lock_guard`/`std::unique_lock`
in its own scope; a function that needs two locks (e.g.
`mav_connection::commandGoto()`, which touches both `state_lock` and
`send_lock`) always fully releases the first before acquiring the second,
never nests them. This sidesteps the lock-ordering question that a
multi-lock codebase would otherwise have to answer: there is no order to get
wrong because there is never more than one lock in play at a given moment on
a given thread.

Cross-object calls follow the same rule at a coarser grain: `FMUStateMachine`
calls into `IMAV`/`ISMM` (`actionState()`) *before* taking `this->lock` to
record `pending_replay_state`, never while holding it; `SMM::retryPendingSearch()`
reads `MAV::getCurrentPosition()` (which takes `mav_connection::state_lock`
internally) before calling `SMM::enqueue()` (which takes `SMM::queue_lock`),
never both at once.

## Single-event-loop-thread invariant

Every state-mutating `FMUStateMachine` method (`FSSNewCommand`,
`SMMNewCommand`, `setLowBattery`, `setCommsFailure`, `setMavCommsFailure`,
`setCurrentAltitude`) and the work they drive (`updateState`/`actionState`, which touch `mav`,
`smm`, and `state_change_cb`) must run on the event-loop thread. All
external inputs — FSS/SMM/MAV callbacks, which fire on their own threads —
are funnelled through `App`'s event queue and applied on the event-loop
thread instead of being actioned directly from the callback. This is what
lets the priority arbitration and `FMUStateMachine`'s state fields be
reasoned about with only `FMUStateMachine::lock` (guarding the small
publish-to-worker-threads surface, `pending_replay_state`) rather than a
lock around every access. `FMUStateMachine::assert_event_loop_thread()`
defends this in debug builds (todo/52); calling a state-mutating method from
another thread would otherwise be a silent data race.

## Mechanical backstop

The invariants above are checked, not just documented: the `sanitizers` CI
job (`.github/workflows/build.yml`) runs the full test suite under
ThreadSanitizer, which would flag a real data race or an actual lock-order
inversion regardless of whether this document stays accurate. Treat this
document as the map for reasoning about a change *before* running TSan, not
a replacement for it.
