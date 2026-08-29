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
# It is a multi-arch index digest, so it still resolves on the arm64/armhf nodes
# the FMU flies on. To bump it, resolve the tag and update both files:
#   docker buildx imagetools inspect debian:trixie | head -2
ARG BASE_IMAGE=debian:trixie@sha256:f324c7ff54321e8d9c588493a20244965938ce0aa50bbd1022d38010e9ffc4b1
FROM ${BASE_IMAGE}

# Re-declared with no value so the global default above comes into scope inside
# the stage: docker/manifest.sh records the base the image was actually built
# from, which is what makes it recoverable from a pushed tag without a rebuild.
ARG BASE_IMAGE
# The git SHA being built. CI passes it; a local build leaves it unknown rather
# than guessing, since there is no .git in the build context to ask.
ARG SOURCE_REVISION=unknown

# Update the database
RUN apt update

# Make sure lsb-release is present
RUN apt install -y lsb-release

RUN apt install -y ca-certificates curl gnupg

# CAP repo: dearmored keyring over HTTPS, matching CI (no deprecated apt-key)
RUN echo "deb https://apt.canterburyairpatrol.org/apt/$(lsb_release -sc)/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/cap.list
RUN curl -fsSL https://apt.canterburyairpatrol.org/repository.key | gpg --dearmor -o /etc/apt/trusted.gpg.d/cap.gpg

# Update the database
RUN apt update

RUN apt install -y valgrind

# jq generates the FSS client config from environment variables at container
# start (docker/generate-config.sh), so string values are properly
# JSON-escaped instead of hand-built with echo.
RUN apt install -y jq

RUN apt install -y build-essential libtool automake pkg-config libcurl4-openssl-dev libtidy-dev libjansson-dev

COPY . /src

WORKDIR /src

RUN ./docker/setup.sh
RUN ./docker/build.sh
RUN ./docker/install.sh

# Last, so it records the tree as it ships: every apt layer above has run, and
# the versions it captures are the ones the built binary linked against.
RUN ./docker/manifest.sh

USER autopilot
WORKDIR /home/autopilot

ENTRYPOINT ["/home/autopilot/scripts/cap-fmu.sh"]
