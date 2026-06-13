# fss-smm-mav

This is the Flight Management Unit that [Canterbury Air Patrol](https://canterburyairpatrol.org/) uses to integrate [ArduPilot](https://www.ardupilot.org) with [Flight Safety System](https://github.com/canterbury-air-patrol/flight-safety-system/) and [Search Management Map](https://github.com/canterbury-air-patrol/search-management-map/)

It will obey commands set for the asset in flight safety system and when in continue mode will perform a search from search management map as applicable.

There is a built-in RTL on low battery or complete loss of communication with the Flight Safety System.

## Basic Setup
#### Dependencies
Direct dependencies are the client library from [Flight Safety System](https://github.com/canterbury-air-patrol/flight-safety-system/) and [SMM Asset API](https://github.com/canterbury-air-patrol/smm-asset-api/)

You will need to build these and install them somewhere they can be found with pkg-config.

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
        ]
}
```

#### Optional FMU configuration

An optional `fmu` block configures asset-specific behaviour. Every key is
optional and falls back to the default shown below if omitted (or if the
block is absent entirely):

```
{
        "...": "... name / ssl / servers as above ...",
        "fmu": {
                "altitude_cap_ft": 400,
                "camera_fov_deg": 90.0,
                "lowbat_threshold": 20,
                "reconnect_interval_s": 10,
                "log_level": "info"
        }
}
```

| Key | Default | Description |
|---|---|---|
| `altitude_cap_m` | `122` | Regulatory ceiling for the derived search altitude, in metres AGL. |
| `altitude_cap_ft` | – | The same ceiling expressed in feet; converted to metres internally. If both `_m` and `_ft` are given, `_ft` wins. |
| `camera_fov_deg` | `90.0` | Camera total cross-track (across-flight) field of view, in degrees. The flight altitude for a search is derived from its sweep width as `altitude = sweep_width / (2 * tan(fov / 2))`, then clamped to the altitude cap. Must be in the open range (0, 180). |
| `lowbat_threshold` | `20` | Battery percentage at or below which a low-battery RTL is triggered. Range 0–100. |
| `reconnect_interval_s` | `10` | Seconds between FSS/MAV reconnection attempts. Range 1–3600. |
| `log_level` | `info` | Logging verbosity: `error`, `info`, or `debug`. |

Then start this client with:
```
cap-fmu --terminate-action=<action> client.json 127.0.0.1 5760
```

`--terminate-action` is required. There is no default — the correct action is airframe-dependent and must be chosen explicitly:

| Action | What it does | Airframe requirements |
|---|---|---|
| `terminate` | Sends `MAV_CMD_DO_FLIGHTTERMINATION` (param1=1) to hard-cut motors or deploy a parachute | Requires `AFS_ENABLE=1` and `AFS_TERM_ACTION` configured on the autopilot |
| `disarm` | Sends `MAV_CMD_COMPONENT_ARM_DISARM` with force-disarm | Only safe on the ground; suitable for ground vehicles or bench testing |
| `none` | Logs a loud warning and falls through to RTL — the safest non-destructive action | No special airframe configuration; use when AFS is not available |

This requires [MAVProxy](https://ardupilot.org/mavproxy/) on the local device with `--tcpin:127.0.0.1:5760` you can adjust parameters as required to access a remote device.

## License
This project is licensed under GNU GPLv2 see the [LICENSE](LICENSE.md) file for details.