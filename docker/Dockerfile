FROM debian:bookworm

# Update the database
RUN apt update

# Make sure lsb-release is present
RUN apt install -y lsb-release

RUN apt install -y ca-certificates curl gnupg

RUN echo "deb http://apt.canterburyairpatrol.org/apt/$(lsb_release -sc)/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/cap.list
RUN curl https://apt.canterburyairpatrol.org/repository.key -o repo.key
RUN apt-key add repo.key

# Update the database
RUN apt update && apt upgrade -y

RUN apt install -y valgrind

RUN apt install -y build-essential libtool automake pkg-config libcurl4-openssl-dev libtidy-dev libjansson-dev

COPY . /src

WORKDIR /src

RUN ./docker/setup.sh
RUN ./docker/build.sh
RUN ./docker/install.sh

USER autopilot
WORKDIR /home/autopilot

ENTRYPOINT ["/home/autopilot/scripts/cap-fmu.sh"]

