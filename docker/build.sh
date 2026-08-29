#!/bin/bash -ex

# Build dependencies from Debian's own archive, matching debian/control
# Build-Depends. libgnutls28-dev: the FSS ssl libraries link -lgnutls/-lgnutlsxx.
#
# The Canterbury Air Patrol libraries (libfss-client-ssl, libfss-transport,
# libsmm-asset-dev -- the stale libfss-client and libsmm-asset names are no
# longer published) are deliberately *not* installed here. They come from
# docker/setup.sh, which runs first and installs them under the version bounds
# derived from configure.ac; installing them again here unconstrained would
# quietly reopen the window that step exists to close.
apt install -y libjsoncpp-dev libgnutls28-dev

cd /src
./autogen.sh
./configure
make

