# Use the official ROS 2 Humble base image
FROM ros:humble

# Set environment variables
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (skip init if already initialized)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update

# Create workspace directory
WORKDIR /workspace

# Ensure the src directory exists
RUN mkdir -p /workspace/src

# Copy everything into the container
COPY . /workspace/

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN colcon build --symlink-install

# Source the workspace setup
RUN echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Default command to run a shell
CMD ["bash"]
