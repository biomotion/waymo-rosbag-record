FROM ros:melodic

RUN apt-get update && apt-get install -y \
    ros-melodic-tf2-msgs \
    ros-melodic-cv-bridge \
    python-pip \
    python-tk \
    build-essential \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade tensorflow==2.0.0a0 \
    matplotlib \
    pptk \
    pyyaml \
    transformations \
    opencv-python

SHELL ["/bin/bash","-c"]

# setup entrypoint
# COPY ./entrypoint.sh /
# ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]