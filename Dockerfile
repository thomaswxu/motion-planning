FROM ubuntu:22.04

LABEL description="Simple Linux image with required packages."
LABEL version="1.0"
LABEL maintainer="TWX"

# Install Required Packages
RUN apt-get update \
 && apt-get install -y build-essential cmake git vim python3 python3-pip \
 && apt-get clean

WORKDIR /motion-planning

# Install Unit Testing Framework
RUN git clone https://github.com/catchorg/Catch2.git \
 && cd Catch2 \
 && git checkout tags/v3.3.2 \
 && cmake -Bbuild -H. -DCMAKE_CXX_STANDARD=17 -DCATCH_BUILD_STATIC_LIBRARY=ON -DBUILD_TESTING=OFF -DCATCH_INSTALL_DOCS=OFF \
 && cmake --build build/ --target install \
 && cd .. \
 && rm -rf Catch2

# Install JSON parser
RUN git clone https://github.com/nlohmann/json.git \
 && mv json/single_include/nlohmann /usr/include/ \
 && rm -rf json

# Install Python packages
ARG PIP_PACKAGES="\
    numpy \
    matplotlib \
    typing \
    pytest"

RUN python3 -m pip install --upgrade pip \
 && python3 -m pip install ${PIP_PACKAGES}