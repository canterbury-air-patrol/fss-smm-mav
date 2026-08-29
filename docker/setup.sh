#!/bin/bash -ex

# Builder stage only. Everything installed here is toolchain and headers, and
# none of it reaches the flight image: the runtime stage (docker/Dockerfile,
# docker/runtime-setup.sh) starts from the same base again and installs only the
# shared libraries the binary loads.

# Update the database
apt update

# Make sure lsb-release is present
apt install -y lsb-release

# Install the CAP repo (dearmored keyring over HTTPS, matching CI). The runtime
# stage copies the two files this writes out of the builder rather than
# repeating this: that is how it reaches the same repository without shipping
# curl and gnupg to an airborne node.
apt install -y ca-certificates curl gnupg
echo "deb https://apt.canterburyairpatrol.org/apt/$(lsb_release -sc)/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/cap.list
# --batch --yes: gpg's overwrite confirmation tries to prompt on /dev/tty
# (absent in docker build) if the file is already there, which would fail the
# build on a rerun of this layer.
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

# Build dependencies from Debian's own archive, matching debian/control
# Build-Depends. libgnutls28-dev: the FSS ssl libraries link -lgnutls/-lgnutlsxx.
#
# These live here rather than in docker/build.sh so that every apt install in
# the builder is in one layer that a source-only change does not invalidate --
# the Dockerfile copies only docker/ and configure.ac before running this.
apt install -y build-essential libtool automake pkg-config libjsoncpp-dev libgnutls28-dev libcurl4-openssl-dev libtidy-dev libjansson-dev

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

# Record what the compile is about to link against, for the runtime stage to
# check its own install against. The repository is rolling and publishes one
# version of each package, so a release landing between the two stages of a
# single build would otherwise ship a library the binary was never built with --
# silently, and inside the version window, so no bound would catch it.
"$(dirname "${BASH_SOURCE[0]}")/dep-constraints.sh" --versions > /build-versions
cat /build-versions
