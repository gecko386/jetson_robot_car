ARG BASE_IMAGE="ros:galactic-pytorch-l4t-r32.6.1"
FROM ${BASE_IMAGE}


RUN apt update && apt install -y --no-install-recommends \
        git \
		curl \
        cmake \
		libboost-all-dev \
		python3-scipy \
		qt5-default \
        python3-pip \
	&& rm -rf /var/lib/apt/lists/*;

RUN pip3 install Jetson.GPIO

#install realsense libs
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt update && apt install -y --no-install-recommends \
        librealsense2-utils \
        librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*;

#install realsense ros2 wrapper
RUN mkdir -p /opt/ros_realsense/src
WORKDIR /opt/ros_realsense/src
RUN git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git
WORKDIR /opt/ros_realsense/
#RUN rosdep init
RUN source "/opt/ros/galactic/install/setup.bash" && rosdep update && rosdep install -i --from-path src --rosdistro "galactic" --skip-keys=librealsense2 -y
RUN source "/opt/ros/galactic/install/setup.bash" && colcon build

#install jetson-inference
WORKDIR /opt
RUN git clone --recursive https://github.com/dusty-nv/jetson-inference
WORKDIR /opt/jetson-inference
RUN mkdir build
WORKDIR /opt/jetson-inference/build
RUN cmake ../
RUN make -j$(nproc)
RUN make install
RUN ldconfig

#install ros_deep_learning packages
RUN mkdir -p /opt/ros_deep_learning/src
WORKDIR  /opt/ros_deep_learning/src
RUN git clone https://github.com/dusty-nv/ros_deep_learning
WORKDIR  /opt/ros_deep_learning/
RUN colcon build

# generate entrypoint
COPY ./ros_entrypoint.sh /.
RUN chmod 755 /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
