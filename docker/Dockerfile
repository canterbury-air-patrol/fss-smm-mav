# Must track the base image every job in .github/workflows/verify.yml runs on.
# While this was bookworm and CI was trixie, the binary that actually flew was
# compiled by a different toolchain against different library versions than
# anything CI had ever exercised --- the gates were green for a build nobody
# shipped. (bullseye -> bookworm in Jan 2025 was following the then-current
# stable; CI moved on to trixie afterwards and this did not.)
FROM debian:trixie

# Update the database
RUN apt update

# Make sure lsb-release is present
RUN apt install -y lsb-release

RUN apt install -y ca-certificates curl gnupg

# CAP repo: dearmored keyring over HTTPS, matching CI (no deprecated apt-key)
RUN echo "deb https://apt.canterburyairpatrol.org/apt/$(lsb_release -sc)/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/cap.list
RUN curl -fsSL https://apt.canterburyairpatrol.org/repository.key | gpg --dearmor -o /etc/apt/trusted.gpg.d/cap.gpg

# Update the database
RUN apt update && apt upgrade -y

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

USER autopilot
WORKDIR /home/autopilot

ENTRYPOINT ["/home/autopilot/scripts/cap-fmu.sh"]

