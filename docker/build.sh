#!/bin/bash -ex

# Builder stage only. Compiles the tree and stages `make install' output under
# /staging, which is the one thing the runtime stage copies out of here: the
# object files, the autotools artifacts, the vendored mavlink submodule and the
# toolchain that produced them all stay in the builder.
#
# Build dependencies are installed by docker/setup.sh, not here. In particular
# the Canterbury Air Patrol libraries (libfss-client-ssl, libfss-transport,
# libsmm-asset-dev) are installed there under the version bounds derived from
# configure.ac; installing them again here unconstrained would quietly reopen
# the window that step exists to close.

cd /src
./autogen.sh
# --prefix=/usr, matching debian/rules' dh_auto_configure: the container and the
# .deb then put cap-fmu at the same path, so a note that says
# `/usr/bin/cap-fmu' is true of both.
./configure --prefix=/usr
make
make install DESTDIR=/staging

# Everything the runtime stage will inherit, listed while the build log is the
# place to look for it.
find /staging -type f
