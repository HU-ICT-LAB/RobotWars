# Create a layer from the 'dustynv/ros:foxy-ros-base-l4t-r32.6.1' image
FROM dustynv/ros:foxy-ros-base-l4t-r32.6.1

# Update the package information from al configured sources
RUN sudo apt update

#Install python3 
RUN sudo apt-get install python3

#Update pip in order to install correct robomaster package version
RUN sudo pip3 install --upgrade pip

#Install robomaster python-library
RUN python3 -m pip install robomaster






