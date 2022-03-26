ARG BASE_IMAGE="dustynv/ros:galactic-pytorch-l4t-r32.7.1"
FROM ${BASE_IMAGE}


RUN apt update && apt install -y --no-install-recommends \
        git \
		curl \
        cmake \
        wget \
		libboost-all-dev \
		python3-scipy \
		qt5-default \
        python3-pip \
        libprotobuf-dev \ 
        libleveldb-dev \
        libsnappy-dev \
        libhdf5-serial-dev \
        liblmdb-dev \
        protobuf-compiler \
        software-properties-common \
	&& rm -rf /var/lib/apt/lists/*;

RUN pip3 install Jetson.GPIO

#install realsense libs
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt update && apt install -y --no-install-recommends \
        librealsense2-utils \
        librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*;

#install YLIDAR
WORKDIR /opt/
RUN git clone https://github.com/YDLIDAR/ydlidar_ros2
WORKDIR /opt/ydlidar_ros2
RUN source "/opt/ros/galactic/install/setup.bash" && colcon build
WORKDIR /opt/ydlidar_ros2/startup
RUN chmod 755 initenv.sh
RUN sh initenv.sh

#install realsense dependencies
WORKDIR /opt
RUN git clone -b galactic https://github.com/ros/diagnostics
WORKDIR /opt/diagnostics
RUN source "/opt/ros/galactic/install/setup.bash" && colcon build
WORKDIR /opt
RUN git clone -b ros2 https://github.com/ros/xacro.git
WORKDIR /opt/xacro
RUN source "/opt/ros/galactic/install/setup.bash" && colcon build

#install realsense ros2 wrapper
RUN mkdir -p /opt/ros_realsense/src
WORKDIR /opt/ros_realsense/src
RUN git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git
WORKDIR /opt/ros_realsense/
#RUN rosdep init
RUN /bin/bash -c 'source "/opt/ros/galactic/install/setup.bash" && \
    source "/opt/diagnostics/install/local_setup.bash" && \
    source "/opt/xacro/install/local_setup.bash" && \
    rosdep update && rosdep install -i --from-path src --rosdistro "galactic" --os=ubuntu:galactic --skip-keys="librealsense2 libopencv-dev" -y && \
    colcon build'

#install cudnn
#WORKDIR /opt
#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/arm64/cuda-ubuntu1804.pin
#RUN mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
#RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/arm64/7fa2af80.pub
#RUN add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/arm64/ /"
#RUN apt-get update
#RUN apt-get install libcudnn8=8.2.4.15-1+cuda10.2
#RUN apt-get install libcudnn8-dev=8.2.4.15-1+cuda10.2
#RUN rm -rf /var/lib/apt/lists/*

#install cafe
#WORKDIR /opt
#RUN git clone https://github.com/BVLC/caffe.git
#RUN mkdir -p /opt/caffe/build
#WORKDIR /opt/caffe/build
#RUN cmake .. && make all && make install

#install jetson-inference
#WORKDIR /opt
#RUN git clone --recursive https://github.com/dusty-nv/jetson-inference
#WORKDIR /opt/jetson-inference
#RUN mkdir build
#WORKDIR /opt/jetson-inference/build
#RUN cmake ../
#RUN make -j$(nproc)
#RUN make install
#RUN ldconfig

#install ros_deep_learning packages
WORKDIR  /opt
RUN git clone https://github.com/dusty-nv/ros_deep_learning
WORKDIR  /opt/ros_deep_learning
RUN /bin/bash -c 'source "/opt/ros/galactic/install/setup.bash" && \
    source "/opt/diagnostics/install/local_setup.bash" && \
    source "/opt/xacro/install/local_setup.bash" && \
    colcon build'

# generate entrypoint
COPY ./ros_entrypoint.sh /.
RUN chmod 755 /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
