FROM osrf/ros:melodic-desktop-full

RUN apt-get update && apt-get upgrade -y && \
	apt-get install -y lsb-core g++ openssh-server gedit vim

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

RUN apt-get update && apt-get install -y gdb gnupg2 apt-transport-https

# WORKDIR /catkin_ws
# COPY . /src/ERC_2021_simulator
# RUN cd ..

# RUN mkdir -p /catkin_ws/src
# RUN cd /catkin_ws/src
# WORKDIR /catkin_ws/src
COPY . /catkin_ws/src/ERC_2021_simulator
WORKDIR /
RUN cd /catkin_ws/src
RUN apt install ros-melodic-industrial-core -y

RUN rosdep update

# RUN rosdep install --from-paths /src/ERC_2021_simulator --ignore-src --rosdistro melodic -r -y

RUN rosdep install --from-paths /catkin_ws/src/ --ignore-src --rosdistro melodic -r -y

#teleop
RUN apt install ros-melodic-teleop* -y

#joystick
RUN apt install ros-melodic-joy* -y

#aruco
RUN apt install ros-melodic-aruco-ros* -y

#controller
RUN apt install ros-melodic-ros-controllers* -y

#pip2
RUN apt install -y python-pip

#pip3
RUN apt install -y python3-pip

RUN apt install -y ros-melodic-moveit 
RUN apt install -y ros-melodic-aruco-detect  
RUN pip install numpy 
RUN pip install pandas 

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make'
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; '
# RUN cd /catkin_ws
# RUN catkin_make

RUN echo "source /opt/ros/melodic/setup.bash" >> /etc/bash.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

COPY ./start.sh /
RUN chmod +x start.sh
CMD ["/start.sh"]
