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

Then start this client with `cap-fmu client.json 127.0.0.1 5760`

This requires [MAVProxy](https://ardupilot.org/mavproxy/) on the local device with `--tcpin:127.0.0.1:5760` you can adjust parameters as required to access a remote device.

## License
This project is licensed under GNU GPLv2 see the [LICENSE](LICENSE.md) file for details.