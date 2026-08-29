#!/bin/bash -e

# Record what actually went into this image, at /etc/cap-fmu-manifest.
#
# The image tag answers "which source flew". Nothing answered "which libraries
# flew" -- and for this program that is the sharper question, because the
# comms-loss failsafe's connection semantics live in libfss-client-ssl rather
# than in this source. An incident review that has to rebuild to find out has
# already lost the answer: the rolling repository the image installs from
# publishes exactly one version of each package, so a rebuild resolves to
# whatever is current, not to what was current on build day.
#
# So it is written into the image at build time and read back with
#     docker run --rm --entrypoint cat <image> /etc/cap-fmu-manifest
# with no rebuild, no network and no access to the repository's history. CI
# also extracts it as a build artifact next to the traceability report.
#
# It runs in the *runtime* stage, after that stage's installs and after the
# binary has been copied in. Left in the builder it would describe the builder's
# package set -- a compiler toolchain and -dev packages that are not in the
# shipped image -- while looking exactly as authoritative as a correct manifest.
# `build-essential' or any `-dev' entry appearing below is the tell that it has
# drifted back; .github/workflows/docker-build.yml checks for exactly that.
#
# The two halves are separated deliberately. Everything under `# packages' is
# the reproducibility claim -- diff that section between two builds of one SHA
# and the difference *is* the drift. The header above it is provenance, and its
# build timestamp differs on every build by construction; comparing it would
# report drift that is not there.

MANIFEST=/etc/cap-fmu-manifest
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The binary's own idea of its version, from the binary that will actually run
# -- not from dpkg, which does not know it: the image builds cap-fmu from source
# rather than installing a .deb of it.
#
# Run, not merely inspected, and captured before the manifest is opened so a
# failure stops the build. It is the one point where the runtime stage's
# libraries meet the builder's binary, so a shared library the runtime install
# missed surfaces here as a link failure rather than at first launch on an
# aircraft.
if ! cap_fmu_version="$(/usr/bin/cap-fmu --version 2>&1)"; then
	echo "manifest.sh: /usr/bin/cap-fmu --version failed in the runtime stage:" >&2
	echo "${cap_fmu_version}" >&2
	exit 1
fi
cap_fmu_version="$(printf '%s\n' "${cap_fmu_version}" | head -n 1)"

# The runtime package names, from the same mapping the runtime install used, so
# this cannot list a package the stage does not have (libsmm-asset-dev, say).
mapfile -t constrained < <("${SCRIPT_DIR}/dep-constraints.sh" --packages --runtime)

{
	echo "# cap-fmu image manifest"
	echo "built-at: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
	echo "base-image: ${BASE_IMAGE:-unknown}"
	echo "source-revision: ${SOURCE_REVISION:-unknown}"
	echo "cap-fmu-version: ${cap_fmu_version}"
	echo
	# Named first because these are the ones the version bounds in
	# configure.ac guard, and the ones an incident review asks for. They
	# appear again in the full list below; the duplication is so the
	# interesting three do not have to be grepped out of ~300 lines.
	#
	# ${Package}, not ${binary:Package}: the latter appends the arch to
	# multi-arch packages (`libsmm-asset-dev:amd64'), so the first column
	# would not be the package name for some rows and not others --- which
	# breaks grepping the manifest for a name, the one thing it is for.
	# The architecture is its own column.
	echo "# constrained dependencies"
	dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' \
		"${constrained[@]}"
	echo
	echo "# packages"
	dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' | sort
} >"$MANIFEST"

chmod 0444 "$MANIFEST"

cat "$MANIFEST"
