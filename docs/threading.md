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
| MAV recv | `mav_connection::connect_to_mav()` | Performs blocking reads and parses inbound MAVLink; invokes the position/battery/reached/autopilot-restart callbacks that enqueue events for the event loop. |
| MAV heartbeat | `mav_connection` constructor path / `start()` | Sends periodic `HEARTBEAT`s and is the single authority for edge-triggering the MAV comms-up/down callback (`heartbeat_loop()`). |
| Signal waiter | `App::signal_waiter()` | Blocks in `sigwait()` for SIGINT/SIGTERM, then flips `App::running` and wakes the event loop and reconnector. |
| FSS reconnector | `App::fss_reconnector()` | Periodically calls `fss->reconnectAll()`, `mav->attemptReconnect()`, and `smm->retryPendingSearch()`. |
| SMM worker | `SMM` constructor / `workerLoop()` | Runs all blocking `smm_asset_*` HTTP I/O (connect, position report, search acquire/accept) off the event loop (todo/33). |
| FSS send worker | `FSS` constructor / `workerLoop()` | Runs all blocking FSS `send()`s (position/reached/battery reports, command acks) off the event loop, so a hung FSS peer cannot stall queued commands. |
| Log writer | `Logger` constructor / `workerLoop()` | Drains the log queue and does the disk I/O, so a slow or blocking write never stalls the thread that produced the message. Every other thread here is a producer. |
| FSS recv (library-owned, one per connected FSS server) | `flight_safety_system::transport::fss_connection`'s `recv_thread`, not started by cap-fmu | Parses inbound FSS traffic and fires *every* inbound FSS callback — `registerCommandCB`, `registerPositionDataCB`, `registerCommsStatusCB`, `registerSMMSettingsCB`. Those callbacks enqueue events for the event loop rather than acting directly; the one exception is `known_aircraft`, which the position callback touches on this thread (see below). |

That is 8 threads cap-fmu itself starts, plus the FSS library's per-connection
recv threads. (Older references — todo/62 and elsewhere — say 7: they predate
the log writer, which moved the Logger's disk I/O off the event loop.) The
library threads are listed because they hold cap-fmu locks and produce cap-fmu
events, even though their lifetime is the library's concern, not ours.

## Data ownership and locks

| Lock / atomic | Guards | Held by |
|---|---|---|
| `App::main_lock` + `main_cv` | `App::event_queue` | Event loop (consumer), every callback that calls `enqueue_event` (producers: MAV recv/heartbeat threads, FSS recv threads, SMM worker via its callbacks, FSS send worker, signal waiter, FSS reconnector). |
| `App::reconnect_lock` + `reconnect_cv` | Only the reconnector's own wait timer | FSS reconnector thread, woken early by the signal waiter. |
| `FMUStateMachine::lock` | `fss_command`, `smm_command`, `fss_command_target`, `low_battery`, `low_battery_count`, `terminated`, `altitude_breach`, `altitude_breach_count`, `altitude_clear_count`, `current_state`, `fss_comms_lost`, `mav_comms_lost` | Event loop only (todo/52's `assert_event_loop_thread()` enforces this in debug builds) — see "Single-event-loop-thread invariant" below. |
| `mav_connection::send_lock` | The socket fd's lifetime (open/close) and the per-channel MAVLink pack/transmit state (the generated `*_pack_chan()` calls mutate global per-channel sequence/status, so packing and sending must be serialised) | Any thread that sends: event loop (via `MAV`/`IMAV` action methods), MAV heartbeat thread, MAV recv thread (mission handshake replies, ADS-B rebroadcast). |
| `mav_connection::state_lock` | `last_position`, `search`, `search_loaded`, `search_loading`, `goto_active`, `goto_position`, `goto_ack_pending`, `pending_mode_command`, `retry_count`, `last_tried` | MAV recv thread (parses inbound MAVLink and updates upload/search state), event loop (issues commands), FSS reconnector (`attemptReconnect`). |
| `mav_connection::heartbeat_mutex` + `heartbeat_cv` | Only the heartbeat loop's own 1-second wait timer | MAV heartbeat thread, woken early by `stopping`. |
| `mav_systems::lock` | `mav_systems::systems` (the per-sysid `mav_sys` list, and through it each system's component list) | MAV recv thread, which is the only one that grows the list (`findSystem`'s find-or-create, from `processMavLinkMsg`) and which also walks it to re-arm the setup latches on a detected autopilot restart (`resetAllSetup()`, todo/108); FSS reconnector thread, which walks it the same way from `disconnect_from_mav()`; the event-loop command paths read it via `findExistingSystem()`, which never mutates, so dispatching a command cannot race a concurrent recv-thread insertion. Each `mav_sys`'s own published state (`autopilot_type`, `flight_mode`, `setup`, `failsafe_checked`, `failsafe_checked_type`) is atomic and read without this lock once the `shared_ptr` is in hand. |
| `SMM::queue_lock` + `queue_cv` | `SMM::task_queue`, `SMM::worker_running` (the shutdown flag the `queue_cv` predicate reads) | SMM worker (consumer); event loop and FSS reconnector (producers, via the public methods that call `enqueue`); the destructor clears `worker_running`. |
| `SMM::state_lock` | `current_search`, `asset` | SMM worker (publishes), event loop (`currentSearchPoints()` reads the atomic mirror instead, see below), test-only injection (`SMMTestAccess`). |
| `FSS::queue_lock` + `queue_cv` | `FSS::task_queue`, `FSS::worker_running` (the shutdown flag the `queue_cv` predicate reads) | FSS send worker (consumer); event loop (producer, via `reportPosition`/`reachedPoint`/`reportBatteryStatus`/`postAck`); the destructor clears `worker_running`. |
| `CommandAckGroup::mtx` (`fss_client_ssl::command_group`) | The in-flight command-dedup group: `active`, `epoch`, `command`, `payload`, `timestamp`, `pending`, `resolution`, `last_id_by_connection` | FSS recv threads (`onDelivery()`, as the same logical command arrives on each connected server) and the FSS send worker (`resolve()`, from the phase-2 ack responder). The acks themselves are sent *outside* the lock, so a recv thread adding a late duplicate never waits on the wire. |
| `known_aircraft::lock` | `lastAllocatedICAO` and the `aircraft` map (including the stale-entry eviction sweep, `evictStaleLocked()`) | FSS recv threads only, via `App`'s `registerPositionDataCB` — the one inbound FSS callback that does work before enqueuing rather than enqueuing straight away, because the synthetic ICAO it allocates has to be stamped onto the `PositionData` the event carries. With more than one FSS server connected there is more than one such thread, which is why the map is locked rather than thread-owned. |
| Atomics (`App::running`, `mav_connection::fd`/`mav_comms_ok`/`last_heartbeat_ts`/`broken`/`stopping`/`started`, `mav_sys::autopilot_type`/`flight_mode`/`setup`/`failsafe_checked`/`failsafe_checked_type`, `SMM::search_active`/`current_search_points`) | Single flags or counters read across threads without a critical section spanning more than the one load/store | Various — each is documented at its declaration; called out here because they are part of the concurrency picture even though they need no mutex. |

The "Guards" column is the inventory: a field held under one of these locks
belongs in it, and a field named in it must still exist. Two categories are
deliberately *not* listed, so their absence is not read as drift:

- **Immutable after construction.** Config values such as
  `FMUStateMachine::low_battery_latch_count`/`altitude_breach_latch_count`/
  `altitude_cap_m` and `mav_connection::goto_altitude_m`/`altitude_floor_m`/
  `altitude_cap_m`/`position_stream_interval_us`/`battery_stream_interval_us`
  are read inside critical sections, but they are written once by the
  constructor and never again, so no lock is what makes them safe.
- **Thread-owned.** Fields written and read on exactly one thread, e.g.
  `mav_connection::gps_fix_type` (recv thread only, see todo/79) and
  `mav_connection::last_time_boot_ms` (recv thread only, see todo/108).

One genuine asymmetry is worth naming rather than hiding:
`FMUStateMachine::state_change_cb` is *written* under `FMUStateMachine::lock`
(`setStateChangeCB`) but *read* without it (`actionState`). That is safe only
because `EventDispatcher`'s constructor installs it on the event-loop thread
before any other thread can reach the state machine — it is a set-once field,
not a genuinely concurrent one. If it ever becomes settable at runtime, the
read side needs the lock too.

`Logger` has its own internal mutex (`queue_lock`, with `queue_cv` and
`idle_cv`, guarding the message queue between every producer thread and the log
writer thread) and is safe to call from any thread — subsystem diagnostics
routed through `ILogger`, todo/59, run on whichever thread produced them, e.g.
MAV's heartbeat thread or SMM's worker thread. It is kept out of the table
above because every thread in the inventory is a producer, so it carries no
ownership information.

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

The same holds for the two locks a command path can meet in sequence:
`mav_connection::setResolvedMode()` resolves the target system through
`mav_systems::findExistingSystem()` (which takes and releases
`mav_systems::lock`) *before* it touches `state_lock` or `send_lock`, and
`processMavLinkMsg()` likewise finishes its `findSystem()` lookup before any
per-message handler takes a lock. `handleAutopilotRestart()` (todo/108) keeps
that shape: `resetAllSetup()` takes and releases `mav_systems::lock`, and only
then does it take `state_lock` to drop the loaded-mission belief.
`disconnect_from_mav()` calls `resetAllSetup()` after its `send_lock` region has
closed, for the same reason — `mav_systems::lock` never nests with either.

Cross-object calls follow the same rule at a coarser grain: `FMUStateMachine`
calls into `IMAV`/`ISMM` (`actionState()`) *before* taking `this->lock`, never
while holding it (`setMavCommsFailure()` and `reassertState()` both read
`current_state` under the lock, release it, and only then action); `SMM::retryPendingSearch()`
reads `MAV::getCurrentPosition()` (which takes `mav_connection::state_lock`
internally) before calling `SMM::enqueue()` (which takes `SMM::queue_lock`),
never both at once.

## Single-event-loop-thread invariant

Every state-mutating `FMUStateMachine` method (`FSSNewCommand`,
`SMMNewCommand`, `setLowBattery`, `setCommsFailure`, `setMavCommsFailure`,
`setCurrentAltitude`, `reassertState`) and the work they drive (`updateState`/`actionState`, which touch `mav`,
`smm`, and `state_change_cb`) must run on the event-loop thread. All
external inputs — FSS/SMM/MAV callbacks, which fire on their own threads —
are funnelled through `App`'s event queue and applied on the event-loop
thread instead of being actioned directly from the callback. This is what
lets the priority arbitration and `FMUStateMachine`'s state fields be
reasoned about with only `FMUStateMachine::lock` rather than a lock around
every access. `FMUStateMachine::assert_event_loop_thread()`
defends this in debug builds (todo/52); calling a state-mutating method from
another thread would otherwise be a silent data race.

## Mechanical backstop

The invariants above are checked, not just documented: the `sanitizers` CI
job (`.github/workflows/build.yml`) runs the full test suite under
ThreadSanitizer, which would flag a real data race or an actual lock-order
inversion regardless of whether this document stays accurate. Treat this
document as the map for reasoning about a change *before* running TSan, not
a replacement for it.
