FROM ros:jazzy

# dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        curl \
        gnupg2 \
        lsb-release \
        python3-colcon-common-extensions \
        gdb \
        ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# setup ROS in bashrc
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

CMD ["bash"]
