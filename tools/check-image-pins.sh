#!/usr/bin/env bash

set -euo pipefail

# The aircraft image and every CI job must be built on the same base, pinned to
# the same digest.
#
# Both halves of that have already gone wrong once. The bases drifted apart
# (docker/Dockerfile on bookworm while CI ran trixie), so the gates were green
# for a toolchain and library set the shipped binary had never been built
# with -- caught by review, not by a check, because "must track CI" was only
# ever a comment. And the tag was floating, so even once they agreed on
# `debian:trixie' the two were only the same image on the days Debian had not
# pushed a point release between them.
#
# A digest in one file and a tag in the other looks fine and is exactly the
# first failure again, one Debian point release later, so this compares the
# whole reference including the digest rather than just the suite name.
#
# Run by check-code.sh, so a bump to one file and not the other fails before the
# commit rather than after the flight.

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dockerfile="${repo_root}/docker/Dockerfile"
verify_yml="${repo_root}/.github/workflows/verify.yml"

base="$(sed -n 's/^ARG BASE_IMAGE=\(.*\)$/\1/p' "$dockerfile")"

if [ -z "$base" ]; then
	echo "check-image-pins.sh: no 'ARG BASE_IMAGE=' line in ${dockerfile}" >&2
	exit 1
fi
if [ "$(printf '%s\n' "$base" | wc -l)" -ne 1 ]; then
	echo "check-image-pins.sh: expected exactly one 'ARG BASE_IMAGE=' line with a value in ${dockerfile}" >&2
	exit 1
fi
# A pin that is not a digest is the floating tag back again, whatever it names.
if [[ "$base" != *"@sha256:"* ]]; then
	echo "check-image-pins.sh: BASE_IMAGE '${base}' is not digest-pinned (expected image@sha256:...)" >&2
	exit 1
fi

# The Dockerfile is multi-stage, and only one of its stages ships. A `FROM' that
# names an image directly instead of ${BASE_IMAGE} would leave everything below
# checking a base the flight image is not built on -- the same class of drift as
# the bookworm/trixie split, but now invisible because the pinned line is still
# there and still agrees with CI. Stages that build on an earlier stage by name
# are how a multi-stage build works and are not bases, so they are exempt.
mapfile -t stage_names < <(sed -n 's/^FROM .* [Aa][Ss] \([A-Za-z0-9_.-]*\).*$/\1/p' "$dockerfile")

stage_status=0
while read -r image; do
	[ -n "$image" ] || continue
	if [ "$image" = '${BASE_IMAGE}' ]; then
		continue
	fi
	is_stage=0
	for stage in ${stage_names[@]+"${stage_names[@]}"}; do
		if [ "$image" = "$stage" ]; then
			is_stage=1
		fi
	done
	if [ "$is_stage" -eq 0 ]; then
		echo "check-image-pins.sh: ${dockerfile} has 'FROM ${image}'" >&2
		echo "                    every stage must build on \${BASE_IMAGE}" >&2
		stage_status=1
	fi
done < <(sed -n 's/^FROM \([^ ]*\).*$/\1/p' "$dockerfile")

if [ "$stage_status" -ne 0 ]; then
	exit "$stage_status"
fi

mapfile -t containers < <(sed -n 's/^ *container: *\(.*\)$/\1/p' "$verify_yml")

if [ "${#containers[@]}" -eq 0 ]; then
	echo "check-image-pins.sh: no 'container:' lines found in ${verify_yml}" >&2
	exit 1
fi

status=0
for container in "${containers[@]}"; do
	if [ "$container" != "$base" ]; then
		echo "check-image-pins.sh: ${verify_yml} runs on '${container}'" >&2
		echo "                    but docker/Dockerfile builds on '${base}'" >&2
		status=1
	fi
done

if [ "$status" -ne 0 ]; then
	echo "check-image-pins.sh: CI must run on the base the aircraft image is built from; bump both together" >&2
	exit "$status"
fi

echo "check-image-pins.sh: ${#containers[@]} CI job(s) and docker/Dockerfile agree on ${base}"
