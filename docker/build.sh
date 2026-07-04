#!/bin/bash -ex

# Build dependencies, matching debian/control Build-Depends
# (libfss-client and libsmm-asset are stale names no longer published).
# libgnutls28-dev: the FSS ssl libraries link -lgnutls/-lgnutlsxx.
apt install -y libfss-client-ssl libsmm-asset-dev libjsoncpp-dev libgnutls28-dev

cd /src
./autogen.sh
./configure
make

