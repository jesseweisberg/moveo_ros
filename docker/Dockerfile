FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04
#######################################################
### nvidia docker settings
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#######################################################
### Setup basic dev environment
RUN apt-get update && apt-get install --no-install-recommends -y \
      build-essential \
      xauth \
      dirmngr \
      gnupg2 \
      lsb-release \
      curl \
      wget \
      software-properties-common \
      dialog \
      apt-utils \
      dialog \
      locales \
      sudo \
      python3-pip \
      python-pip \
      && apt-get clean

# setup environment
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install devs tools
RUN apt-get update && apt-get install --no-install-recommends -y \
      tmux \
      vim \
      cmake \
      gdb \
      valgrind \
      && apt-get clean

# Set user
ARG USERNAME=docker
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID $USERNAME
RUN useradd -m -u $UID -g $GID -s /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo
RUN echo "\ndocker    ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && /usr/sbin/visudo -c

#######################################################
### Install ROS Kinetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install --no-install-recommends -y \
        ros-kinetic-desktop-full \
        python-rosinstall \
        python-rosinstall \
        python-vcstools \
        python-rosinstall-generator \
        build-essential \
        && apt-get clean
RUN pip install catkin_tools

# Install project dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
	python-genshi \
	python-lxml-dbg \
	python-lxml-doc \
  	libccd-dev \
	libccd2 \
	libfcl-0.5-dev \
	libfcl0.5 \
	python-bs4 \
	python-chardet \
	python-html5lib \
	python-lxml \
	ros-kinetic-moveit-core \
	ros-kinetic-moveit-msgs \
	ros-kinetic-object-recognition-msgs \
	ros-kinetic-octomap-msgs \
	ros-kinetic-srdfdom \
	ros-kinetic-urdfdom-py \
	libglew-dev \
	libglew1.13 \
	ros-kinetic-chomp-motion-planner \
	ros-kinetic-moveit-experimental \
	ros-kinetic-moveit-fake-controller-manager \
	ros-kinetic-moveit-ros-perception \
	ros-kinetic-moveit-ros-planning \
	ros-kinetic-moveit-kinematics \
	ros-kinetic-moveit-ros-manipulation \
	ros-kinetic-moveit-ros-move-group \
	ros-kinetic-moveit-ros-planning-interface \
	ros-kinetic-moveit-ros-robot-interaction \
	ros-kinetic-moveit-ros-visualization \
	ros-kinetic-moveit-ros-warehouse \
	ros-kinetic-warehouse-ros \
	ros-kinetic-graph-msgs \
	ros-kinetic-moveit-visual-tools \
	ros-kinetic-rviz-visual-tools \
	ros-kinetic-moveit-planners-ompl \
	ros-kinetic-ompl \
	python-serial \
	ros-kinetic-rosserial \
	ros-kinetic-rosserial-client \
	ros-kinetic-rosserial-msgs \
	ros-kinetic-rosserial-python \
        ros-kinetic-rosserial-arduino \
        && apt-get clean

# Initialize rosdep
RUN rosdep init

USER docker
RUN rosdep update

# nvidia-container-runtime
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV QT_X11_NO_MITSHM 1

EXPOSE 11345

ENTRYPOINT [""] # this will make docker try to exec '' (the empty string)
