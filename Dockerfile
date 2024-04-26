FROM nvidia/cuda:12.4.1-devel-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    vim \
    git \
    net-tools

RUN apt install -y \
    python3 \
    python3-pip \
    curl

# for opengl
RUN apt install -y mesa-utils

# install ros

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update && apt install -y ros-noetic-desktop-full

RUN apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

RUN apt install python3-rosdep

RUN rosdep init && rosdep update

# grpc install

RUN apt install -y build-essential autoconf libtool pkg-config

RUN git config --global http.lowSpeedLimit 0 && git config --global http.lowSpeedTime 999999

RUN cd /root/ && git clone --recurse-submodules -b v1.62.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc

RUN mkdir -p /root/.local
ENV PATH="/root/.local/bin:$PATH"

RUN cd /root/grpc && \
    mkdir -p cmake/build && \
    cd cmake/build && \
    cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX=/root/.local \
      ../.. && \
    make -j16 && \
    make install

# fix grpc version issue
RUN cp /root/.local/include/google/protobuf/any.h /root/.local/include/google/protobuf/Any.h && \
    cp /root/.local/include/google/protobuf/any.pb.h /root/.local/include/google/protobuf/Any.pb.h && \
    cp /root/.local/include/google/protobuf/any.proto /root/.local/include/google/protobuf/Any.proto

RUN cd /root/ && git clone https://github.com/peichunhuang-1/core.git

RUN cd /root/core && mkdir build && cd build && \
    cmake .. -DCMAKE_PREFIX_PATH=${HOME}/.local -DCMAKE_INSTALL_PREFIX=${HOME}/.local && \
    make -j12 && \
    make install

RUN echo "export PATH="${HOME}/.local/bin:$PATH"" >> ~/.bashrc && \
    echo "export CORE_LOCAL_IP="127.0.0.1"" >> ~/.bashrc && \
    echo "export CORE_MASTER_ADDR="127.0.0.1:10010"" >> ~/.bashrc && \
    echo "export PROTO_PATH=/${HOME}/.local/include" >> ~/.bashrc

# install webots
RUN apt install wget
RUN mkdir -p /etc/apt/keyrings && \
    cd /etc/apt/keyrings && \
    wget -q https://cyberbotics.com/Cyberbotics.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list && \
    apt update && apt install -y webots

ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME /usr/local/webots
ENV PATH /usr/local/webots:${PATH}

# Enable OpenGL capabilities
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

# Set a user name to fix a warning
ENV USER root

# Set the locales
RUN apt install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'
# ---

# install simulation packages
# RUN cd ~ && git clone --recurse-submodules https://github.com/yisyuanshen/corgi_ros_ws.git

# install catkintools
RUN sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y python3-catkin-tools

# build corgi ros workspace
# WORKDIR /root/corgi_ros_ws/
# RUN catkin config --extend /opt/ros/noetic && catkin build -DLOCAL_PACKAGE_PATH=$HOME/.local -j12

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/corgi_ros_ws/devel/setup.bash" >> /root/.bashrc

RUN apt install -y nvtop
RUN pip3 install scipy osqp matplotlib

CMD ["/bin/bash"]
