FROM ros:noetic-ros-core-focal

RUN apt-get update -y\
   && apt-get upgrade -y\
   && apt-get install -y python3-wstool

RUN apt-get install -y --no-install-recommends \
    gazebo11 \
    build-essential \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init
    
RUN rosdep update

COPY catkin_ws/src /home/catkin_ws/src
COPY catkin_ws/src/simple_scene/worlds /usr/share/gazebo-11/worlds
COPY catkin_ws/src/simple_scene/models /root/.gazebo/models

RUN cd /home/catkin_ws/src\
   && wstool init .\
   && wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall \
   && wstool remove  moveit_tutorials \
   && wstool update -t . \

RUN apt-get update
RUN . /opt/ros/noetic/setup.sh\
	&& apt install ros-noetic-moveit -y\
	&& cd /home/catkin_ws \
	&& rosdep install --rosdistro=noetic --from-path ./src -y --ignore-src\
	&& catkin build
	
RUN apt-get install ros-noetic-trac-ik -y

RUN echo "source /opt/ros/noetic/setup.sh" >> ~/.bashrc
RUN echo "source /home/catkin_ws/devel/setup.sh" >> ~/.bashrc

  
WORKDIR /home/catkin_ws/

