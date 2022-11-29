
ARG VARIANT="bionic"
ARG ROS_VERSION="melodic"
FROM ros:${ROS_VERSION}-ros-base-${VARIANT}

SHELL [ "/bin/bash", "-c" ]

# Update the APT repos to a local mirror.
ARG LOCAL_MIRROR_URI=nz.archive.ubuntu.com
RUN sed -i -e s/archive.ubuntu.com/${LOCAL_MIRROR_URI}/g /etc/apt/sources.list

# Install Kitware APT repo.
# This repo contains an updated CMake package.
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        wget \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && wget -qq https://apt.kitware.com/kitware-archive.sh \
    && chmod +x kitware-archive.sh \
    && ./kitware-archive.sh \
    && rm ./kitware-archive.sh

# Install APT depenencies
RUN apt-get update \
    && apt-get install --no-install-recommends --yes \
        build-essential \
        cmake \
        gdb \
        libpcl-dev \
        python-catkin-tools \
        python-rosdep \
        python-rosinstall \
        python-vcstool \
        ros-melodic-pcl-ros \
        ssh \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create non-root user for development.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(ALL\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# Install Livox-SDK.
WORKDIR /tmp
ARG LIVOX_SDK_BRANCH=master
ENV LIVOX_SDK_BRANCH ${LIVOX_SDK_BRANCH}
ARG LIVOX_SDK_BUILD_TYPE=Release
RUN git clone -b ${LIVOX_SDK_BRANCH} https://github.com/oxin-ros/Livox-SDK.git src/livox-sdk \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && catkin_make \
        -DCMAKE_BUILD_TYPE=${LIVOX_SDK_BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=/opt/ros/${ROS_DISTRO} \
    && cd build \
    && sudo make install \
    && rm -rf /tmp/src
WORKDIR /workspace

# Automatically source ros in each terminal.
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/${USERNAME}/.bashrc

# Setup entrypoint.
COPY ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]
CMD [ "/bin/bash" ]
