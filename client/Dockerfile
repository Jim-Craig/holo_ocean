# This container is used to package and generate worlds automatically
FROM ghcr.io/epicgames/unreal-engine:runtime

USER root
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
# RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl git libglib2.0-dev software-properties-common

# OpenCV's runtime dependencies (and other dependencies)
RUN apt-get install -y libglib2.0-0 libsm6 libxrender-dev libxext6

# Install all python versions to test on
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y python3-dev python3-pip \
    python3.9-dev python3.9-distutils \
    python3.10-dev python3.10-distutils \
    python3.11-dev python3.11-distutils \
    python3.12-dev \
    python3.13-dev 

RUN pip3 install setuptools wheel tox posix_ipc numpy

# Setup user
USER ue4

CMD /bin/bash
