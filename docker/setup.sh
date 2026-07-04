#!/bin/bash -ex

# Update the database
apt update

# Make sure lsb-release is present
apt install -y lsb-release

# Install the CAP repo (dearmored keyring over HTTPS, matching CI)
apt install -y ca-certificates curl gnupg
echo "deb https://apt.canterburyairpatrol.org/apt/$(lsb_release -sc)/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/cap.list
# --batch --yes: the Dockerfile layer already created cap.gpg, and without
# them gpg's overwrite confirmation tries to prompt on /dev/tty (absent in
# docker build), failing the build.
curl -fsSL https://apt.canterburyairpatrol.org/repository.key | gpg --batch --yes --dearmor -o /etc/apt/trusted.gpg.d/cap.gpg

# Update the database
apt update && apt upgrade -y

# Install dependencies
apt install -y libfss-client-ssl

# Create the autopilot user
useradd -ms /bin/bash autopilot
