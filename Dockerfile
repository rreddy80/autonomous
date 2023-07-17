# Use the official Ubuntu 18.04 base image
FROM ubuntu:18.04
LABEL maintainer vramesh.reddy@gmail.com
ARG DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    git \
    python3 \
    python3-pip

# Install Gazebo
RUN apt-get install -y \
    gazebo9 \
    libgazebo9-dev

# Install PX4 dependencies
RUN apt-get install -y \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    libxml2-utils

# Install ArduPilot dependencies
RUN apt-get install -y \
    libtool \
    autoconf \
    automake \
    astyle \
    build-essential \
    coreutils \
    cmake \
    cppcheck \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	libfuse2 \
	libxml2-dev \
	libxml2-utils \
	make \
	ninja-build \
    python3-dev \
    python3-opencv \
    python3-pip \
    python3-setuptools \
    python3-serial \
    python3-wxgtk4.0 \
    python3-matplotlib \
    python3-lxml \
    python3-markupsafe \
    rsync \
	shellcheck \
	unzip \
	zip

# Set environment variables for ArduPilot
ENV PATH=$PATH:/ardupilot/Tools/autotest
ENV PATH=/ardupilot:$PATH
ENV PATH=/ardupilot/Tools/autotest:$PATH
ENV PYTHONPATH=/ardupilot/modules:$PYTHONPATH

# Clone PX4 and ArduPilot repositories
RUN git clone https://github.com/PX4/PX4-Autopilot.git /px4
RUN git clone https://github.com/ArduPilot/ardupilot.git /ardupilot

# Install Python packages
RUN pip3 install kconfiglib
RUN pip3 install --user empy jinja2
RUN pip3 install --user pyros-genmsg
RUN pip3 install --user pyyaml

# RUN /usr/local/bin/python3 -m pip install future --user
# RUN /usr/local/bin/python3 -m pip install numpy --user

# Build PX4
RUN cd /px4 && make px4_sitl_default

# Build ArduPilot
RUN cd /ardupilot && git submodule update --init --recursive
RUN cd /ardupilot && ./waf configure --board sitl && ./waf build --target bin/arducopter

# Install Python packages
RUN pip3 install dronekit dronekit-sitl

# Set the entry point
ENTRYPOINT ["/bin/bash"]
