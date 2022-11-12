#!/bin/bash -ex

SETUP_PATH=`pwd`/docker

# Install the script to run it
su - autopilot -c "mkdir -p /home/autopilot/scripts"
su - autopilot -c "cp ${SETUP_PATH}/cap-fmu.sh /home/autopilot/scripts/"

# Create the config file
su - autopilot -c "mkdir -p /home/autopilot/config"
su - autopilot -c "touch /home/autopilot/config/fmu-client.json"
