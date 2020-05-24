# Copyright (C) 2020  Christian Berger
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

#  First stage for building the software:
FROM ubuntu:18.04 as builder
MAINTAINER Christian Berger "christian.berger@gu.se"

ENV DEBIAN_FRONTEND noninteractive

# Upgrade the Ubuntu 18.04 LTS base image
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y


RUN apt-get update -y
#New code for ARM
RUN apt-get upgrade -y
RUN apt-get install -y wget
RUN apt-get install -y qemu-user-static
#RUN docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
#RUN docker run --rm -t arm32v6/alpine uname -m


RUN mkdir -p ~/.docker/cli-plugins
RUN cd ~/.docker/cli-plugins
RUN wget https://github.com/docker/buildx/releases/download/v0.3.1/buildx-v0.3.1.linux-amd64
RUN mv buildx-v0.3.1.linux-amd64 docker-buildx 
RUN chmod 755 docker-buildx

#RUN DOCKER_CLI_EXPERIMENTAL=enabled docker buildx create --name platformbuilder --use
#RUN DOCKER_CLI_EXPERIMENTAL=enabled docker buildx inspect --bootstrap
#RUN cd ~


#End of new code.  Delete if it sucks 

# Install the development libraries for OpenCV
RUN apt-get install -y --no-install-recommends \
        ca-certificates \
        cmake \
        build-essential \
        libopencv-dev

# Include this source tree and compile the sources
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp .. && \
    make && make install


# Second stage for packaging the software into a software bundle:
FROM ubuntu:18.04
MAINTAINER Nicholas Fisher "gusfishnir@student.gu.se"

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y
    
RUN apt-get update

RUN apt-get install -y --no-install-recommends \
        libopencv-core3.2 \
        libopencv-highgui3.2 \
        libopencv-imgproc3.2 

WORKDIR /usr/bin
COPY --from=builder /tmp/bin/steering-service .
# This is the entrypoint when starting the Docker container; hence, this Docker image is automatically starting our software on its creation
ENTRYPOINT ["/usr/bin/steering-service"]

