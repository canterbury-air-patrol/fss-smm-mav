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
#
# No `apt upgrade -y' here or in the Dockerfile any more. It was what made the
# digest-pinned base ineffective: pinning FROM to a digest fixes the base layer
# and then upgrading every package in it immediately undoes that, so two builds
# of one SHA a week apart still produced different software. The pinned digest
# is now the whole statement about what the base contains, and moving to a newer
# one is an edit to the Dockerfile rather than a side effect of the build date.
# Security updates therefore arrive when the digest is bumped -- see
# docs/decisions.md.
apt update

# Install the flight-safety-system and smm-asset libraries, bounded by the same
# version window configure.ac enforces (docker/dep-constraints.sh derives the
# terms from it, so there is one copy of the bounds, not two).
#
# `apt-get satisfy' rather than `apt install': the constraint is enforced by the
# solver, so a repository publishing a release outside the window fails this
# step instead of installing it and leaving configure to notice -- or, past the
# floor, not to notice. The FMU's comms-loss failsafe semantics are the
# library's, so an unbounded install here is an unbounded change to the
# failsafe.
mapfile -t constraints < <("$(dirname "${BASH_SOURCE[0]}")/dep-constraints.sh")
apt-get satisfy -y "${constraints[@]}"

# Create the autopilot user
useradd -ms /bin/bash autopilot
