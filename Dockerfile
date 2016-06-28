FROM ros:indigo

# using bash instead of sh to be able to source
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && \
    apt-get install -y curl && \
    curl -L bit.ly/dev-machine | bash -s -- -w /workspace/shadow_robot/base

# Cleaning up, deleting all sources
RUN cd /workspace/shadow_robot/base_deps/src && \
    wstool set sandbox -y --git https://github.com/shadow-robot/sandbox.git --version="hydro-devel" && \
    wstool set industrial_moveit -y --git https://github.com/ros-industrial/industrial_moveit.git --version="indigo-devel" && \
    wstool set sr_benchmarking -y --git https://github.com/shadow-robot/sr_benchmarking && \
    wstool up -j 3 industrial_moveit sandbox sr_benchmarking && \
    source /opt/ros/indigo/setup.bash && \
    cd .. && \
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y && \
    catkin_make && \
    pip install tabulate

RUN cd /workspace/shadow_robot/base/src && \
    wstool set -y sr_interface --version-new "F#64_planner_test_suite_ugo" && \
    wstool up sr_interface && \
    cd ../../base_deps/src && \
    wstool up sr_benchmarking && \
    source /workspace/shadow_robot/base_deps/devel/setup.bash && \
    cd /workspace/shadow_robot/base && \
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y && \
    catkin_make

RUN mkdir -p {/data,/results} && \
    ln -s /workspace/shadow_robot/base/src/sr_interface/sr_multi_moveit/sr_moveit_planner_benchmarking/data /data/planners_benchmark

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
