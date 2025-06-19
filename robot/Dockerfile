FROM --platform=linux/arm64/v8 ros:humble-ros-core

SHELL ["/bin/bash", "-c"]


ENV DEBIAN_FRONTEND noninteractive

COPY ./dependencies /app/dep

COPY ./debs /debs

RUN apt install software-properties-common -y

RUN apt install -y curl gnupg2 lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor --batch --yes -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository for Jazzy (Ubuntu 24.04 is "noble")
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list


RUN --mount=type=cache,target=/var/cache/apt apt update -y


RUN apt install git ninja-build meson udev python3-ply python3-jinja2 libglib2.0-dev libgstreamer-plugins-base1.0-dev libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev  -y

#RUN --mount=type=cache,target=/root/.cache/pip xargs pip3 install < /app/dep/python.txt

RUN mkdir -p /app/src

RUN mkdir -p /app/cmd

RUN mkdir -p /app/src/workspace

RUN apt remove -y meson
RUN pip3 install --upgrade meson

WORKDIR /

RUN git clone https://github.com/raspberrypi/libcamera.git

WORKDIR /libcamera

RUN meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled

RUN ninja -C build

RUN ninja -C build install

WORKDIR /

RUN echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list

RUN curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -

RUN apt update

RUN apt install devscripts debhelper dkms  -y

RUN apt install /debs/libedgetpu-dev.deb /debs/libedgetpu1-std.deb -y

RUN git clone https://github.com/google/gasket-driver.git

WORKDIR /gasket-driver

RUN debuild -us -uc -tc -b

WORKDIR /

RUN dpkg -i gasket-dkms_1.0-18_all.deb

RUN ldconfig

RUN chmod -R a+x /app/cmd

RUN apt install -y tini

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD [ "/entrypoint.sh" ]