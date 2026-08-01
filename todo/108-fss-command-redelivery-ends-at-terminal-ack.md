# 108 — FSS stops re-sending a command once the FMU acks it terminally

**Status (2026-08-01): DONE.** Both questions answered below; the gap question 2
named is closed on this end.

- **Q1 — no, the FMU does not rely on the 10-second redelivery.** No server-side
  slow re-assert is wanted. Evidence and pinning below.
- **Q2 — cap-fmu re-applies commanded state itself**, on a MAV-link recovery
  edge and on a detected autopilot restart. Implemented here, not upstream.

Filed 2026-07-31 from `flight-safety-system` todo/68. This was a **notice of a
server-side behaviour change plus two questions only this end can answer**, not
a defect on this end.

## What changed on the server

`fss_client::sendCommand()` used to re-dispatch an asset's newest command every
10 seconds for as long as the aircraft stayed connected, rewriting the command
row's dispatch id each time and nulling `ack_state`/`ack_timestamp`/
`ack_superseded_by` with it — destroying and re-establishing the stored record
of what the aircraft did with a command every 10 seconds, forever, on the
healthy path.

Three changes address it; only the third is visible here:

1. Acks are matched on the command row's primary key, translated in-session from
   the `acked_command_id` this end echoes. **No wire change**, and every
   delivery stays ackable, including resends.
2. A resend no longer records a dispatch, so the ack columns are written once
   per delivery and never cleared underneath a settled outcome.
3. **Once the FMU acks a command terminally — `actioned`, `superseded`,
   `rejected` or `noop`, i.e. anything but `received` — the server stops
   re-sending that command on that connection.** A newer command still
   dispatches immediately.

The stop is scoped to the **connection**, not the command row: the server builds
a new `fss_client` for every accepted TCP connection, so an FMU restart, a
server bounce, a network drop or a duplicate-identity eviction all reconnect into
a session with no memory of the old ack, and the redelivery loop runs again until
*that* connection sees its own terminal ack. So the FMU is never assumed to still
hold a command it acked on an earlier connection, and needs no persistent storage
for this to be correct.

## Q1 — the FMU does not use redelivery as a keep-alive

Nothing in cap-fmu re-arms on a repeated delivery, and nothing takes effect only
on a second one:

- A redelivery carries the same `server_command_id`, so
  `CommandAckGroup::onDelivery()` (`src/fss/command-ack-group.hpp`) resolves it
  as `already_resolved` and replays the cached ack without the command
  re-entering the state machine at all. Only a *fresh* `server_command_id` — a
  deliberate operator retry, a new DB row — re-actuates.
- Even when a command does reach `FSSNewCommand()`, `updateState()` returns
  `nullopt` when the resulting state is unchanged, and `actionState()` — the sole
  path to any MAVLink send — runs only on an actual transition. A redelivered
  identical command has never re-commanded the autopilot, on any code path.
- No watchdog is fed by command delivery. MAV liveness is `HEARTBEAT` +
  `heartbeat_loop()`; FSS liveness is the server's own 1 Hz RTT request, which
  refreshes `last_message_received_time` regardless of commands.
- A failed MAVLink send is re-driven by cap-fmu's own recovery rule (below), not
  by a server resend.

Pinned by `tests/fmu_test.cpp`: "a same-command redelivery with the same server
command id still dedups", the several "does not re-action on repeated FSS …"
cases, and the `[state_machine][replay]` group.

## Q2 — cap-fmu re-applies commanded state after an autopilot restart

Answering this honestly meant fixing it: an autopilot reboot **did** silently
lose commanded state, and the 10-second redelivery was not covering that either
(see Q1 — a redelivery produced no MAVLink traffic). Three holes, all now
closed:

1. **A recovery edge only re-sent a command whose original send had failed.**
   `setMavCommsFailure(false)` consulted `pending_replay_state` (todo/46), so a
   low-battery RTL or a terminate that *had* reached the autopilot before the
   reboot was never re-sent — a non-clearing latch holds `current_state`
   constant, so there was no transition to ride either. Now every genuine
   down→up edge re-commands `current_state` unconditionally.
   `pending_replay_state` is gone: this rule subsumes it. A link blip costs one
   redundant `SET_MODE`; a reboot that is never re-commanded costs the aircraft.
2. **A typical reboot produces no comms edge at all.** ArduPilot is back inside
   ~3s, under `heartbeat_loop()`'s 5000 ms timeout, so the link never reads
   down. `autopilot_restarted()` (`src/mav/mav-comms.hpp`) now watches
   `time_boot_ms` from `GLOBAL_POSITION_INT` and `SYSTEM_TIME` (previously
   discarded) and reports a backwards jump past a 3s margin as a restart, which
   reaches the state machine as a `MavAutopilotRestart` event and calls
   `FMUStateMachine::reassertState()`.
3. **The restart also invalidated things this end still believed.** The
   `SET_MESSAGE_INTERVAL` stream requests and the failsafe-config check were
   one-shot for the whole process — never re-issued after a reboot, nor even
   after a full TCP reconnect — and `search_loaded`/`goto_active` still claimed a
   mission the reboot had wiped, which would have made a re-applied
   `fmu_state_searching` a no-op. A restart now re-arms the setup latches and
   drops the loaded-mission belief; so does `disconnect_from_mav()`.

Known and accepted: `time_boot_ms` is `uint32` milliseconds and wraps at ~49.7
days of autopilot uptime, which reads here as a restart. The cost is one
redundant re-assert and a stream re-request.

**Files:** `src/mav/mav-comms.hpp`, `src/mav/internal.hpp`,
`src/mav/mavlink.cpp`, `src/mav/mav-sys.cpp`, `src/mav/imav.hpp`,
`src/mav/mav.cpp`, `src/mav/mav.hpp`, `src/fmu-types.hpp`, `src/event.hpp`,
`src/event-dispatcher.cpp`, `src/main.cpp`, `src/fmu.cpp`, `src/fmu.hpp`,
`tests/fmu_test.cpp`, `tests/mav_io_test.cpp`, `docs/threading.md`.

## Where this is pinned upstream

`flight-safety-system/docs/decisions/68-command-redelivery-and-ack-keying.md`,
plus unit cases in `tests/server_session_test.cpp` (a terminal ack stops the
resend; a `received` ack does not; a new dbid dispatches regardless; a fresh
session resends despite the previous session's terminal ack) and an e2e case
that kills and restarts the client mid-command. That document's "Alternatives
deliberately not taken" lists a periodic re-assert, to be revisited only "if
cap-fmu's answer to todo/108 is that they cannot re-apply" — it can. The
autopilot-reboot gap it names as cap-fmu's to handle is handled.
