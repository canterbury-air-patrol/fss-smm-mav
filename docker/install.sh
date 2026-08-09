#!/bin/bash -ex

SETUP_PATH=`pwd`/docker

# Install the scripts to run it (generate-config.sh is called by cap-fmu.sh
# from the same directory, so both must be installed together).
su - autopilot -c "mkdir -p /home/autopilot/scripts"
su - autopilot -c "cp ${SETUP_PATH}/cap-fmu.sh ${SETUP_PATH}/generate-config.sh /home/autopilot/scripts/"

# Create the config file
su - autopilot -c "mkdir -p /home/autopilot/config"
su - autopilot -c "touch /home/autopilot/config/fmu-client.json"

# Create the default log directory. Unlike the two directories
# above (under /home/autopilot, already autopilot-owned), /var/log itself is
# root-owned, so the non-root autopilot user can't mkdir under it at
# runtime -- Logger then silently disables logging (see README). Create it
# here as root (this script runs as root; only the two commands above
# deliberately run as autopilot via `su`) and hand it to the runtime user.
mkdir -p /var/log/cap-fmu
chown autopilot:autopilot /var/log/cap-fmu
