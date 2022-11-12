#!/bin/bash -ex

# Update the database
apt update

# Make sure lsb-release is present
apt install -y lsb-release

# Install the CAP repo
apt install -y ca-certificates curl gnupg
echo "deb http://apt.canterburyairpatrol.org/apt/$(lsb_release -sc)/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/cap.list
curl https://apt.canterburyairpatrol.org/repository.key -o repo.key
apt-key add repo.key

# Update the database
apt update && apt upgrade -y

# Install dependencies
apt install -y libfss-client-ssl

# Create the autopilot user
useradd -ms /bin/bash autopilot
