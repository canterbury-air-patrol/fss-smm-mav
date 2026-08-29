#!/bin/bash -ex

# Runtime stage only, and despite the name it does not install the binary: that
# is `make install' in the builder stage (docker/build.sh), staged under
# /staging and copied in by the Dockerfile. What is left here is the runtime
# user, the two entrypoint scripts, and the directories they write to.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Create the autopilot user. The two stages share no filesystem, so this
# belongs in the stage that has the home directory it owns.
useradd -ms /bin/bash autopilot

# Install the scripts to run it (generate-config.sh is called by cap-fmu.sh
# from the same directory, so both must be installed together).
su - autopilot -c "mkdir -p /home/autopilot/scripts"
su - autopilot -c "cp ${SCRIPT_DIR}/cap-fmu.sh ${SCRIPT_DIR}/generate-config.sh /home/autopilot/scripts/"

# Create the config file
su - autopilot -c "mkdir -p /home/autopilot/config"
su - autopilot -c "touch /home/autopilot/config/fmu-client.json"

# Create the default log directory. Unlike the two directories
# above (under /home/autopilot, already autopilot-owned), /var/log itself is
# root-owned, so the non-root autopilot user can't mkdir under it at
# runtime -- Logger then silently disables logging (see README). Create it
# here as root (this script runs as root; only the commands above
# deliberately run as autopilot via `su`) and hand it to the runtime user.
mkdir -p /var/log/cap-fmu
chown autopilot:autopilot /var/log/cap-fmu
