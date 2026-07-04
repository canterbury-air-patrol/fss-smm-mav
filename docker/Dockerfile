FROM debian:bookworm

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
# JSON-escaped instead of hand-built with echo (todo/56).
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

