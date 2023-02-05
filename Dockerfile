# This is an auto generated Dockerfile for ros:perception
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-perception=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*


# source ros setup script to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc