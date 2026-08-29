#!/bin/bash -ex

# Runtime stage only. Installs what the *flight* image needs and nothing else.
#
# The builder stage installs a compiler, headers and the -dev dependency chain;
# none of that is here, and none of it is copied over. What arrives from the
# builder is /staging (the `make install' tree) plus the two files that point
# apt at apt.canterburyairpatrol.org -- so this stage reaches the same
# repository without shipping curl and gnupg on a network-facing airborne node.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Debian's own archive first: the base image has no ca-certificates, so the CAP
# repository (HTTPS) is not usable until this has run. That is why the two apt
# files copied from the builder are staged next to this script rather than
# dropped straight into /etc/apt -- an `apt update' with cap.list already in
# place and no CA store fails.
apt update
# ca-certificates is not just for apt: libsmm-asset verifies SMM's TLS chain
# with it at runtime, so it ships either way.
# jq generates the FSS client config from environment variables at container
# start (docker/generate-config.sh), so string values are properly
# JSON-escaped instead of hand-built with echo.
apt install -y --no-install-recommends ca-certificates jq

# valgrind is a debug tool, and RUN_IN_VALGRIND=yes is the only thing that ever
# used it. Shipping it by default put a memory-debugging toolchain on the
# aircraft to serve a path nobody takes in flight; build with
# --build-arg INCLUDE_VALGRIND=yes for an image that has it, and
# docker/cap-fmu.sh says so when it is asked for and absent.
if [ "${INCLUDE_VALGRIND:-no}" == "yes" ]; then
	apt install -y --no-install-recommends valgrind
fi

install -m 0644 "${SCRIPT_DIR}/cap.list" /etc/apt/sources.list.d/cap.list
install -m 0644 "${SCRIPT_DIR}/cap.gpg" /etc/apt/trusted.gpg.d/cap.gpg
apt update

# The same bounds, from the same place, as the build stage -- but naming the
# shared libraries rather than the -dev packages (docker/dep-constraints.sh's
# --runtime mapping). An unconstrained `apt install' here would reopen the
# version window todo/113 closed, one stage later and out of sight of
# configure.ac, and the binary would then run against a library no build ever
# saw.
mapfile -t constraints < <("${SCRIPT_DIR}/dep-constraints.sh" --runtime)
apt-get satisfy -y --no-install-recommends "${constraints[@]}"

# Bounds are not equality. apt.canterburyairpatrol.org is rolling, so a release
# published between the builder's install and this one is inside the window,
# installs cleanly, and leaves the flight image running a library the binary was
# never compiled or linked against -- the exact failure the version window
# exists to prevent, arriving through the one gap a window cannot see. The two
# stages must resolve to the same versions, so compare them and fail here.
#
# Keyed by pkg-config module because the package names differ by stage
# (libsmm-asset-dev in the builder, libsmmasset0 here); the two are published
# from one source at one version, so their version strings are comparable.
"${SCRIPT_DIR}/dep-constraints.sh" --versions --runtime > /tmp/runtime-versions
if ! diff -u "${SCRIPT_DIR}/build-versions" /tmp/runtime-versions; then
	echo "runtime-setup.sh: the runtime stage resolved different library versions" >&2
	echo "                  than the build stage linked against (left: builder," >&2
	echo "                  right: runtime). The repository published a release" >&2
	echo "                  mid-build; rebuild so both stages take the new one." >&2
	exit 1
fi
rm -f /tmp/runtime-versions

# The package lists are ~40MB of index that nothing at runtime reads, and every
# `apt install' layer above retains its own copy of them.
apt clean
rm -rf /var/lib/apt/lists/*
