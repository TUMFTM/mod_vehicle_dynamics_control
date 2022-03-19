FROM gitlab.lrz.de:5005/iac/docker_iac:latest

# install boost
RUN apt-get update
RUN apt-get install -y -q --no-install-recommends \
    libboost-all-dev

# Copy repository into container
COPY ros2_wrapper /dev_ws/src/mod_control
COPY osqp /dev_ws/lib/osqp

# Build and test workspace
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
             colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release' --packages-select can_dbc_parser tum_tmpc_control tum_raptor_emulation && \
             colcon test"

# Set entrypoint by sourcing overlay workspace
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. /dev_ws/install/local_setup.bash\nexec "$@"' >> /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
