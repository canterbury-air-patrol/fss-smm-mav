# The base is pinned to a digest, not a tag, and the digest is written exactly
# once -- here. Two things depend on that:
#
#   * Reproducibility. `debian:trixie' is a floating tag whose contents change
#     under it on every Debian point release, so two builds of one git SHA
#     produced different software and nothing recorded which one flew. A digest
#     is a specific set of bytes; `apt upgrade -y' (which used to run here and
#     in docker/setup.sh, and would have undone the pin) is gone for the same
#     reason. Security updates now arrive when this line is bumped, which is a
#     reviewable commit rather than a side effect of the build date. See
#     docs/decisions.md.
#
#   * Tracking CI. This must stay the base every job in
#     .github/workflows/verify.yml runs on -- tools/check-image-pins.sh enforces
#     that mechanically, and check-code.sh runs it. While this was bookworm and
#     CI was trixie, the binary that actually flew was compiled by a different
#     toolchain against different library versions than anything CI had ever
#     exercised: the gates were green for a build nobody shipped. That drift was
#     only ever prevented by this comment, which is why it happened.
#
# Both stages below build on it, and check-image-pins.sh requires that too: a
# runtime stage on some other base would be a flight image built on something
# neither CI nor the compile ever saw, with this file still looking pinned.
#
# It is a multi-arch index digest, so it still resolves on the arm64/armhf nodes
# the FMU flies on. To bump it, resolve the tag and update both files:
#   docker buildx imagetools inspect debian:trixie | head -2
ARG BASE_IMAGE=debian:trixie@sha256:f324c7ff54321e8d9c588493a20244965938ce0aa50bbd1022d38010e9ffc4b1

# ---------------------------------------------------------------------------
# Builder: toolchain, headers, and the compile. None of it ships.
# ---------------------------------------------------------------------------
FROM ${BASE_IMAGE} AS builder

# Only what docker/setup.sh reads, before the source: the apt work below is the
# expensive layer, and copying the whole tree first would re-run every install
# on any source change.
COPY docker /src/docker
COPY configure.ac /src/configure.ac
WORKDIR /src
RUN ./docker/setup.sh

COPY . /src
RUN ./docker/build.sh

# ---------------------------------------------------------------------------
# Runtime: the flight image. The same base again, the shared libraries the
# binary loads, and the installed binary -- no compiler, no -dev package, no
# source tree, no apt lists.
# ---------------------------------------------------------------------------
FROM ${BASE_IMAGE} AS runtime

# Re-declared with no value so the global default above comes into scope inside
# the stage: docker/manifest.sh records the base the image was actually built
# from, which is what makes it recoverable from a pushed tag without a rebuild.
ARG BASE_IMAGE
# The git SHA being built. CI passes it; a local build leaves it unknown rather
# than guessing, since there is no .git in the build context to ask.
ARG SOURCE_REVISION=unknown
# Debug images only. valgrind exists in this image for RUN_IN_VALGRIND=yes and
# nothing else, so the flight image does not carry it; docker/cap-fmu.sh names
# this arg when it is asked for and absent.
ARG INCLUDE_VALGRIND=no

# The build-time scripts and the two inputs they read: configure.ac (the version
# bounds docker/dep-constraints.sh derives from) and, from the builder, the apt
# repository files and the versions the compile linked against. /build is
# removed once the last of them has run.
COPY docker /build/docker
COPY configure.ac /build/configure.ac
COPY --from=builder /etc/apt/sources.list.d/cap.list /build/docker/cap.list
COPY --from=builder /etc/apt/trusted.gpg.d/cap.gpg /build/docker/cap.gpg
COPY --from=builder /build-versions /build/docker/build-versions
RUN /build/docker/runtime-setup.sh

# `make install' output only -- /staging/usr/bin/cap-fmu, and nothing else the
# builder touched.
COPY --from=builder /staging/ /

# One layer, so /build does not outlive the scripts in it. manifest.sh last, so
# it records the tree as it ships: every install above has run, and the versions
# it captures are the ones the binary will actually load.
RUN /build/docker/install.sh && /build/docker/manifest.sh && rm -rf /build

USER autopilot
WORKDIR /home/autopilot

ENTRYPOINT ["/home/autopilot/scripts/cap-fmu.sh"]
