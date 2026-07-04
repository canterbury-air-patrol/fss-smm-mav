#!/bin/bash -ex

SETUP_PATH=`pwd`/docker

# Install the scripts to run it (generate-config.sh is called by cap-fmu.sh
# from the same directory, so both must be installed together).
su - autopilot -c "mkdir -p /home/autopilot/scripts"
su - autopilot -c "cp ${SETUP_PATH}/cap-fmu.sh ${SETUP_PATH}/generate-config.sh /home/autopilot/scripts/"

# Create the config file
su - autopilot -c "mkdir -p /home/autopilot/config"
su - autopilot -c "touch /home/autopilot/config/fmu-client.json"
