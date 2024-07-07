FROM osrf/ros:humble-desktop-full

WORKDIR /droid_remote
COPY cyclonedds.xml /tmp/cyclonedds.xml
COPY .tmux.conf /root
COPY Packages /droid_remote/Packages

#Setup for cyclondds middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/tmp/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

#Install packages
RUN apt-get -y update
RUN apt-get install -y iputils-ping
RUN apt-get install -y python3-pip 
RUN python3 -m pip install -r ./Packages/requirements.txt
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
RUN apt-get install -y tmux 
RUN apt-get install -y vim

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 
RUN echo "source /droid_remote/ws/install/setup.bash" >> /root/.bashrc 

CMD ["/bin/bash"] 
