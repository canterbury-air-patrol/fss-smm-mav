# cap-fmu

This is the Flight Management Unit that [Canterbury Air Patrol](https://canterburyairpatrol.org/) uses to integrate [ArduPilot](https://www.ardupilot.org) with [Flight Safety System](https://github.com/canterbury-air-patrol/flight-safety-system/) and [Search Management Map](https://github.com/canterbury-air-patrol/search-management-map/)

It will obey commands set for the asset in flight safety system and when in continue mode will perform a search from search management map as applicable.

There is a built-in RTL on low battery, complete loss of communication with the Flight Safety System, or a sustained breach of the regulatory altitude cap.

The FSS comms-loss failsafe is **armed at startup**, not once FSS has been
reached and then lost: an FMU that has never connected to a server is treated as
one that has lost FSS, and flies the failsafe RTL until a server admits it. The
client library only reports connection status *changes*, and never dials from
its constructor, so a client that has never come up would otherwise report
nothing at all and leave the failsafe unarmed for the whole flight. Two
consequences worth knowing:

- Booting with no reachable FSS server commands RTL. On the ground and disarmed
  that is inert; in the air (an FMU restarted mid-flight) it is the intended
  behaviour. The command is replayed if the MAV link was still down when it was
  first sent.
- Until FSS reports comms okay, every FSS command resolves as *superseded* by
  the failsafe. Commands are still retained, and apply as soon as a server
  admits the FMU.

Once FSS comms are established, an asset with no command set in FSS searches, the
same as one commanded `continue` — "no command received yet" and `continue` map
to the same state. The FSS server dispatches the asset's current command at
identify time, so a cold start with a command already set moves to that command
within milliseconds of connecting.

## Basic Setup
#### Dependencies
Direct dependencies are the client library from [Flight Safety System](https://github.com/canterbury-air-patrol/flight-safety-system/) and [SMM Asset API](https://github.com/canterbury-air-patrol/smm-asset-api/)

You will need to build these and install them somewhere they can be found with pkg-config.

| Package | Supported versions |
|---|---|
| `fss-client-ssl` | `>= 1.3.0` |
| `fss-transport` | `>= 1.3.0` |
| `smm-asset` | `>= 1.1.0` |
| `jsoncpp` | any |
| `catch2-with-main` (or `catch2`) | `>= 3.0`, or `catch2` `>= 2.13` (unit tests only) |

The flight-safety-system floor is a **hard requirement, not a preference**, and
`configure` fails below it. 1.3.0 is the release that gave each FSS server its
own outbound worker, and this source is written against the semantics that
brought, not merely built with them:

- `sendMsgAll()` is non-blocking, so one FSS peer that completes TLS and then
  stops reading sheds only its own telemetry instead of stalling the fan-out to
  the healthy servers.
- `attemptReconnect()` is non-blocking, so a set of unreachable FSS servers no
  longer delays MAV reconnection or the SMM pending-search retry that share the
  reconnector's loop. One consequence to know: a server becomes live on the
  *pass after* the one that dialled it, so bringing a recovered server back
  takes up to one extra `reconnect_interval_s`.
- Connection status counts the servers that have **admitted** this client, not
  the ones it has merely connected to. An FSS server refuses a client while its
  own database fail-safe is degraded, and does so after the TLS handshake — so
  under 1.2.x the FMU could briefly leave its comms-loss failsafe, resume the
  previous command, and re-enter the failsafe when the drop landed.

Building against 1.2.x would silently reinstate the blocking fan-out and the
premature comms-okay. This was an explicit `< 1.3` window while that ABI break
was pending; it is a plain floor again now, and gets a ceiling again when the
next break is announced.

### Build/Install
You can build this package from source:
```
git clone https://github.com/canterbury-air-patrol/fss-smm-mav.git
cd fss-smm-mav
./autogen.sh
./configure
make
make install
```

#### Tests
`make check` builds and runs the Catch2 unit and integration suites in
`tests/`. **Either Catch2 generation works**: 3.x (`catch2-with-main.pc`) is
preferred and is what the suite is written against, and 2.13 or newer
(`catch2.pc`, header-only) is supported as a fallback so that Debian bookworm
— which has only 2.13 in main, and is what the aircraft image is built from —
can still build and run the tests. `configure` picks whichever is installed,
`tests/catch2-compat.hpp` bridges the differences, and Debian's `catch2`
package is the right one on every suite.

Catch2 is the one dependency the FMU binary itself does not need, so a machine
can be missing it — or have only 2.x — and still build the binary. But then
there is no suite to run, and an automake `make check` with no tests in it
exits 0 in silence, which looks exactly like a run that passed. So:

- `configure` warns loudly when Catch2 is missing, as the last thing it prints.
- `make check` in that configuration **fails**, reporting that no tests were
  built, rather than succeeding empty.
- `./configure --disable-tests` is the deliberate opt-out: it skips the Catch2
  probe, and `make check` then says no tests were built and exits 0.
- `./configure --enable-tests` makes Catch2 a hard requirement — `configure`
  fails if it is absent. Use it anywhere the test coverage must not be lost
  quietly, such as CI or a packaging build.

### Running
#### Prerequisites
You will need [MAVProxy](https://ardupilot.org/mavproxy/) running and connected to the autopilot.

You will need at least one [Flight Safety System](https://github.com/canterbury-air-patrol/flight-safety-system/) server running.

You will need a certificate generated from FSS that is signed by the CA, and the ca.public.pem file that your servers are using. The client certificate needs to match the configured 'name' exactly.

Create a client.json file that refers to your server(s):
```
{
        "name": "test-1",
        "ssl": {
                "ca_public_key": "ca.public.pem",
                "client_private_key": "test-1.private.pem",
                "client_public_key": "test-1.public.pem"
        },
        "servers": [
                {
                        "address": "localhost",
                        "port": 20202
                }
        ],
        "tcp_user_timeout_ms": 30000
}
```

`tcp_user_timeout_ms` is optional and read directly by the FSS client
library (it is a top-level key, not part of the `fmu` block below): the
`TCP_USER_TIMEOUT` applied to every FSS server connection, bounding how long
a blocking send can stall into a half-dead FSS peer before the kernel errors
the connection out. Defaults to 30000 (30s, the flight-safety-system
library default) if omitted. A wedged FSS send only blocks the dedicated FSS
worker thread, not the event loop (see the `fmu` block's `mav_send_timeout_s`
for the equivalent MAV-side bound), but a tighter value here still recovers
a dead-peer send sooner.

`clock_offset_ms` is likewise optional and top-level, also read directly by
the FSS client library: a signed millisecond offset applied to every
wall-clock-stamped outbound FSS message (RTT response, position report).
Defaults to 0 (no skew) if omitted. Intended for testing an FMU against a
deliberately skewed idea of wall-clock time without touching the host's real
`CLOCK_REALTIME`, which is a single non-namespaced kernel-global value that
`--cap-add=SYS_TIME` + `date -s` inside a container would skew for every
other container sharing that kernel. Set via the Docker image's
`CLOCK_OFFSET_MS` environment variable (see `docker/generate-config.sh`); the
FMU's own code never touches this key, the FSS client library parses it.

`learned_server_expiry_ms` (default 60000) and `max_learned_servers` (default
16) are two more optional top-level keys the FSS client library parses itself,
new in 1.3.0. They apply only to servers the client *learns* from a server-list
broadcast: one that goes unmentioned for the expiry window is dropped, and the
client refuses to learn more than the cap. Servers listed in `servers` above
are the operator's declared intent — they are never expired and do not count
against the cap, so an outage that empties every broadcast list cannot strand
the FMU with nothing to connect to. Set the expiry to 0 to keep the pre-1.3.0
behaviour of never dropping a learned server.

#### Optional FMU configuration

The `client.json` file itself is required and must be a valid, openable JSON
file — it is also the FSS client's own config, so a missing or malformed
file is a fatal startup error regardless of anything below. Within that
file, the `fmu` block is optional, and every key inside it is optional too,
falling back to the default shown below if omitted (or if the block is
absent entirely):

```
{
        "...": "... name / ssl / servers as above ...",
        "fmu": {
                "mav_address": "127.0.0.1",
                "mav_port": 5760,
                "mav_connect_timeout_s": 5,
                "mav_send_timeout_s": 2,
                "altitude_cap_ft": 400,
                "altitude_breach_latch_count": 5,
                "altitude_floor_ft": 33,
                "goto_altitude_ft": 165,
                "camera_fov_deg": 90.0,
                "lowbat_threshold": 20,
                "low_battery_latch_count": 4,
                "reconnect_interval_s": 10,
                "position_stream_interval_ms": 200,
                "battery_stream_interval_ms": 1000,
                "smm_position_report_interval_ms": 1000,
                "smm_connect_timeout_s": 5,
                "smm_transfer_timeout_s": 10,
                "log_level": "info",
                "log_dir": "/var/log/cap-fmu"
        }
}
```

| Key | Default | Description |
|---|---|---|
| `mav_address` | `127.0.0.1` | Host/IP of the MAVLink autopilot endpoint the FMU connects to (e.g. a mavproxy/SITL TCP endpoint). Unlike the other keys (which warn and fall back to their default), a present-but-invalid value here is **fatal**: an empty/whitespace-only or non-string `mav_address` aborts startup, since silently using the default could connect to the wrong autopilot. Omit the key to use the default. |
| `mav_port` | `5760` | TCP port of the MAVLink autopilot endpoint. Range 1–65535; a present-but-invalid value is **fatal** (see `mav_address`). |
| `mav_connect_timeout_s` | `5` | Upper bound (seconds) on a MAV TCP connect attempt. A black-holed or unreachable autopilot endpoint cannot stall startup or a reconnect attempt beyond this. Range 1–30. |
| `mav_send_timeout_s` | `2` | Upper bound (seconds) on a blocking MAV send (`SO_SNDTIMEO` and, where available, `TCP_USER_TIMEOUT`). A peer that stops reading, or a network path that silently disappears, cannot pin a sender — including the event-loop thread issuing a safety command — beyond this. Kept tighter than `mav_connect_timeout_s`: unlike a slow connect, a blocked send runs on/behind the live event loop and directly extends comms-failure detection latency. Range 1–10. |
| `altitude_cap_m` | `122` | Regulatory ceiling for the derived search altitude, in metres AGL. |
| `altitude_cap_ft` | – | The same ceiling expressed in feet; converted to metres internally. If both `_m` and `_ft` are given, `_ft` wins. |
| `altitude_breach_latch_count` | `5` | Number of consecutive over-/under-cap AGL `GLOBAL_POSITION_INT` readings required to trip, or clear, the continuous altitude-cap enforcement RTL (see intro). Range 1–100. Symmetric: the same count debounces both directions. Unlike the low-battery latch below, this one is self-clearing — once altitude drops back under the cap for this many consecutive readings, control returns to whatever FSS/SMM command is current. |
| `altitude_floor_m` | `10` | Minimum search altitude, in metres AGL. The derived altitude is never flown below this, so a tiny or zero sweep width cannot put the aircraft at ground level. Clamped to be no greater than the altitude cap. |
| `altitude_floor_ft` | – | The floor expressed in feet; converted to metres internally. If both `_m` and `_ft` are given, `_ft` wins. |
| `goto_altitude_m` | `50` | Altitude (metres AGL, relative to home) a `goto` command is flown at. A goto carries only a target position, so the FMU supplies this altitude. Clamped into the `[floor, cap]` range, so a goto can never be flown above the ceiling or into the ground. |
| `goto_altitude_ft` | – | The goto altitude expressed in feet; converted to metres internally. If both `_m` and `_ft` are given, `_ft` wins. |
| `camera_fov_deg` | `90.0` | Camera total cross-track (across-flight) field of view, in degrees. The flight altitude for a search is derived from its sweep width as `altitude = sweep_width / (2 * tan(fov / 2))`, then clamped to the [floor, cap] range. Must be in the open range (0, 180). |
| `lowbat_threshold` | `20` | Battery percentage below which a low-battery RTL is triggered. Range 0–100. The trigger is debounced: the RTL latch only engages after `low_battery_latch_count` *consecutive* readings below the threshold, so a single noisy/spurious sample cannot ground the mission. **This is a deliberate, one-way fail-safe policy, not a defect**: once latched, the RTL is held until the FMU is restarted — a later higher reading, however many, does not release it. A stuck-low or noisy single battery sensor can therefore ground the mission for the rest of the flight with no operator override; that is the intended conservative behaviour (a real low battery must never be waved off by an optimistic post-sag reading), so treat a nuisance latch as an airframe/sensor tuning problem (`lowbat_threshold`, `low_battery_latch_count`, `battery_stream_interval_ms`) rather than something to bypass in flight. |
| `low_battery_latch_count` | `4` | Number of consecutive sub-threshold `BATTERY_STATUS` readings required to engage the low-battery RTL latch (see `lowbat_threshold`). Range 1–100. Its real-world duration is `low_battery_latch_count * battery_stream_interval_ms`, so raising the battery stream rate silently shortens the debounce window unless this is raised to compensate. |
| `reconnect_interval_s` | `10` | Seconds between FSS/MAV reconnection attempts. Range 1–3600. |
| `position_stream_interval_ms` | `200` | Interval (milliseconds) the autopilot is asked to stream `GLOBAL_POSITION_INT` at (200ms = 5Hz). A fixed-wing may want faster updates than a slow rover. Range 50–60000. |
| `battery_stream_interval_ms` | `1000` | Interval (milliseconds) the autopilot is asked to stream `BATTERY_STATUS` at. Range 50–60000. |
| `smm_position_report_interval_ms` | `1000` | Minimum interval (milliseconds) between position reports to the SMM server, throttling them independently of the (faster) MAVLink position stream. Range 100–60000. |
| `smm_connect_timeout_s` | `5` | Connect timeout (seconds) for SMM HTTP requests. SMM I/O runs on a dedicated worker thread (public SMM methods enqueue work and return, so queued FSS commands are never blocked on SMM HTTP); this timeout bounds how long a slow or hung SMM endpoint can pin that worker, delaying shutdown, search-acquire retries, and other queued SMM work. Range 1–30 (the smm-asset library default of 30s is the cap, since a larger value only loosens the bound). |
| `smm_transfer_timeout_s` | `10` | Total transfer timeout (seconds) for SMM HTTP requests; see `smm_connect_timeout_s`. Range 1–60 (capped at the smm-asset library default of 60s). |
| `log_level` | `info` | Logging verbosity: `error`, `warning`, `info`, or `debug`. Each level adds to the one before it, so `error` shows only failures and `warning` adds degraded-but-handled conditions (a link drop, an ignored mission ack, a misconfigured autopilot param). Every log line names its own level between the timestamp and the message. |
| `log_dir` | `/var/log/cap-fmu` | Directory the rotating `fmu.log` is written to. The FMU normally runs as a non-root user, so set this to a path that user can write; the directory is created if missing, and logging is skipped with a warning if it cannot be. |

One interval is deliberately absent from that table: the rate the FMU reports
its own position to the **FSS** servers is fixed at 1 Hz and is not
configurable. Those reports feed the operator display on FSS-Web, and
server-side liveness is judged from RTT responses rather than from them, so
there is nothing per-airframe to tune — unlike the SMM upload
(`smm_position_report_interval_ms`) and the MAVLink stream
(`position_stream_interval_ms`), which are. Running the MAVLink stream faster
does not change it; the surplus is dropped at the FSS throttle.

Then start this client with:
```
cap-fmu --terminate-action=<action> client.json
```

The MAVLink endpoint is taken from the `mav_address` / `mav_port` keys in the
`fmu` block above (defaulting to `127.0.0.1` / `5760`).

`--terminate-action` is required. There is no default — the correct action is airframe-dependent and must be chosen explicitly:

| Action | What it does | Airframe requirements |
|---|---|---|
| `terminate` | Sends `MAV_CMD_DO_FLIGHTTERMINATION` (param1=1) to hard-cut motors or deploy a parachute | Requires `AFS_ENABLE=1` and `AFS_TERM_ACTION` configured on the autopilot |
| `disarm` | Sends `MAV_CMD_COMPONENT_ARM_DISARM` with force-disarm | Only safe on the ground; suitable for ground vehicles or bench testing |
| `none` | Logs a loud warning and falls through to RTL — the safest non-destructive action | No special airframe configuration; use when AFS is not available |

On the first heartbeat, the FMU does a **read-only, advisory** sanity check of
the autopilot config the above depends on: `AFS_ENABLE`/`AFS_TERM_ACTION` when
`terminate` is selected, and in all cases the autopilot's own GCS/telemetry
failsafe — the backstop the comms-loss/low-battery RTL latches rely on while
the MAV link is down. That means more than the enable flag, because an enabled
failsafe can still be configured to carry on flying the mission: the check
covers the failsafe's *action* and `SYSID_MYGCS` too, per family, matching the
table under [Required autopilot configuration](#required-autopilot-configuration)
below. (The timeouts and `SYSID_ENFORCE` are checked only by the ground tool: a
slow backstop is still a backstop.) A mismatch logs a loud warning naming the
parameter, the value read back, and what is wrong with it; it never blocks
flight — the FMU only requests params, it never writes them. The GCS-failsafe
param name is vehicle-firmware dependent:
`FS_GCS_ENABL` for Plane and `FS_GCS_ENABLE` for Copter are confirmed to
exist and answer on ArduPilot 4.6 (read back by name with
`tools/apconfig_check.py` against SITL — the same firmware source and
parameter tables as an airframe, a different HAL). Rover's `FS_GCS_ENABLE`
comes from `Rover/Parameters.cpp` and has never been requested from a
running vehicle. None of that says your aircraft has the parameter set
correctly, only that the name exists in 4.6 — confirm it against your
fleet's actual params if the check doesn't behave as expected; a wrong name
simply means that one check silently doesn't fire, same as not checking at
all.

That check is a last-chance warning, not a gate. Verifying the configuration
is a ground activity — see below.

This requires [MAVProxy](https://ardupilot.org/mavproxy/) on the local device with `--tcpin:127.0.0.1:5760` you can adjust parameters as required to access a remote device.

#### Running in Docker

The `canterburyairpatrol/cap-fmu` image generates `client.json` at container
start from environment variables (`docker/generate-config.sh`), so the file
above is not mounted — it is rebuilt on every start and any hand edit is lost.

Required:

| Variable | Meaning |
|---|---|
| `TERMINATE_ACTION` | `none`, `disarm` or `terminate`; see `--terminate-action` above |
| `NAME` | FSS asset/client name; also selects `/certs/<NAME>.{private,public}.pem` |
| `MAVPROXY_HOST` / `MAVPROXY_PORT` | The MAVLink endpoint (`mav_address` / `mav_port`) |
| `SERVER1_ADDR` / `SERVER1_PORT` | The FSS server |

Optional: `SERVER2_ADDR` / `SERVER2_PORT` (a second, independent FSS server for
redundancy), `LOG_DIR`, `CLOCK_OFFSET_MS`, and `RUN_IN_VALGRIND=yes`.

Every key in the `fmu` block above also has an environment variable, named as
the uppercased key — `ALTITUDE_CAP_FT`, `LOWBAT_THRESHOLD`, `CAMERA_FOV_DEG`,
`LOG_LEVEL`, and so on for the whole table. **Each is omitted from the
generated config unless set**, so a container that sets none of them runs on
exactly the defaults documented above; the defaults live in `FmuConfig`, not
duplicated in the entrypoint.

This matters most for `ALTITUDE_CAP_FT`. The regulatory ceiling is the one
number most likely to differ between jurisdictions, airframes and test flights,
and until these variables existed the only way to change it in a container was
to rebuild the image.

`generate-config.sh` only checks that a value is well-formed (an integer is an
integer) and fails before writing anything if it is not. The **ranges** are
enforced by `loadFmuConfig`, which warns and falls back to the default for an
out-of-range value — one authority for the bounds rather than two that can
drift apart. So a rejected value shows up as a `Config:` warning at startup,
not as a container that refuses to launch.

```
docker run \
  -e TERMINATE_ACTION=none -e NAME=test-1 \
  -e MAVPROXY_HOST=127.0.0.1 -e MAVPROXY_PORT=5760 \
  -e SERVER1_ADDR=fss.example.org -e SERVER1_PORT=20202 \
  -e ALTITUDE_CAP_FT=250 -e LOG_LEVEL=debug \
  -v /path/to/certs:/certs \
  canterburyairpatrol/cap-fmu
```

## Required autopilot configuration

The FMU relies on the autopilot's own GCS/telemetry failsafe as the backstop
for a companion-computer failure: if the FMU stops heartbeating, the aircraft
must return to launch on its own, with no ground intervention. **That backstop
is disabled on a stock airframe and must be configured deliberately.** On
ArduPlane 4.6 defaults it fails three ways at once — the failsafe is off, it
watches the ground station's heartbeat rather than the FMU's, and its action
is to carry on flying the mission.

| Parameter | Value | Applies to | Why |
|---|---|---|---|
| `SYSID_MYGCS` | `200` | all | The FMU's own MAVLink system id. The failsafe must watch *its* heartbeat: MAVProxy runs on the same companion computer and heartbeats as 255, so with the default an FMU process death is invisible to the autopilot. |
| `SYSID_ENFORCE` | `0` | all | `1` would reject every packet not from system 200, including the pilot's own GCS. |
| `FS_GCS_ENABL` | `1` | Plane | Enables the GCS failsafe. (`2` also acceptable; `3` only acts in AUTO, and the FMU also flies GUIDED and LOITER.) |
| `FS_LONG_ACTN` | `1` | Plane | RTL. The default `0` means *continue the mission* in AUTO/GUIDED — the failsafe fires and the aircraft keeps flying. |
| `FS_LONG_TIMEOUT` | `<= 5` | Plane | How long the FMU can be silent before the aircraft returns to launch. |
| `FS_GCS_ENABLE` | `1` or `3` | Copter | RTL, or SmartRTL falling back to RTL. |
| `FS_GCS_TIMEOUT` | `<= 5` | Copter | As `FS_LONG_TIMEOUT`. |
| `FS_OPTIONS` | bit 1 clear | Copter | Bit 1 continues the mission in AUTO on a GCS failsafe — the same trap as `FS_LONG_ACTN=0`. |
| `FS_GCS_ENABLE` | `1` | Rover | `2` continues the mission in Auto. |
| `FS_ACTION` | `1` or `3` | Rover | The default is Hold. |
| `FS_TIMEOUT` | `<= 5` | Rover | As `FS_LONG_TIMEOUT`. |

`SYSID_MYGCS=200` means a restart of the FMU that lasts longer than the
failsafe timeout will trigger an RTL. That is the intended, conservative
direction: the aircraft returns rather than continuing without a control
link.

Every row above except the timeouts and `SYSID_ENFORCE` is also checked in
flight, on the first heartbeat, as an advisory warning (see above). Verify on
the ground regardless — the in-flight check is the last-chance warning for a
configuration error that should already have been caught.

`tools/apconfig_check.py` verifies all of the above against a real aircraft:

```
tools/apconfig_check.py --device udp:127.0.0.1:14550
tools/apconfig_check.py --device /dev/ttyUSB0 --baud 57600 \
    --terminate-action terminate --json evidence.json
```

It reads parameters only, never writes them, reports pass/fail per parameter
with the reason each one matters, and exits non-zero if anything is wrong (or
if the aircraft never answers for a parameter — "not checked" is a failure,
not a pass). `--json` writes a record for the aircraft's change log. It needs
`pymavlink`; the expectations themselves are plain Python and are unit tested
in CI without it.

## License
This project is licensed under GNU GPLv2 see the [LICENSE](LICENSE.md) file for details.