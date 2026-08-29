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
# The two halves are separated deliberately. Everything under `# packages' is
# the reproducibility claim -- diff that section between two builds of one SHA
# and the difference *is* the drift. The header above it is provenance, and its
# build timestamp differs on every build by construction; comparing it would
# report drift that is not there.

MANIFEST=/etc/cap-fmu-manifest

{
	echo "# cap-fmu image manifest"
	echo "built-at: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
	echo "base-image: ${BASE_IMAGE:-unknown}"
	echo "source-revision: ${SOURCE_REVISION:-unknown}"
	# The binary's own idea of its version, from the tree that was just
	# compiled -- not from dpkg, which does not know it: the image builds
	# cap-fmu from source rather than installing a .deb of it.
	echo "cap-fmu-version: $(/src/src/cap-fmu --version 2>&1 | head -n 1)"
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
		libfss-client-ssl libfss-transport libsmm-asset-dev
	echo
	echo "# packages"
	dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' | sort
} >"$MANIFEST"

chmod 0444 "$MANIFEST"

cat "$MANIFEST"
