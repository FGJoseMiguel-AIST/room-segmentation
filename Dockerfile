FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic

# Herramientas base
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata dirmngr gnupg2 curl ca-certificates git nano build-essential \
    lsb-release \
 && rm -rf /var/lib/apt/lists/*

# Repos ROS1
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ROS desktop-full + herramientas de bootstrap
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-vcstools python3-catkin-tools \
 && rm -rf /var/lib/apt/lists/*

# Dependencias específicas que usaste en la build
RUN apt-get update && apt-get install -y --no-install-recommends \
    libdlib-dev ros-noetic-libdlib \
    ros-noetic-opengm \
    ros-noetic-cob-map-accessibility-analysis \
    ros-noetic-move-base-msgs \
    libomp-dev \
    coinor-libcoinutils-dev coinor-libosi-dev coinor-libclp-dev coinor-libcbc-dev \
 && rm -rf /var/lib/apt/lists/*

# rosdep
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# Workspace único
RUN mkdir -p /ws/src
WORKDIR /ws/src

# Clonar código
# ipa_coverage_planning (rama noetic_dev)
RUN git clone --depth=1 -b noetic_dev https://github.com/ipa320/ipa_coverage_planning

# Tu paquete (cliente/launch)
RUN git clone --depth=1 https://github.com/FGJoseMiguel-AIST/room-segmentation ipa_room_segmentation_tools

# Parches mínimos que aplicaste (quedan documentados)
# 1) include <ros/ros.h> en morphological_segmentation.cpp
RUN bash -lc 'f=/ws/src/ipa_coverage_planning/ipa_room_segmentation/common/src/morphological_segmentation.cpp; \
              grep -q "ros/ros.h" "$f" || sed -i "1i #include <ros/ros.h>" "$f"'

# 2) OpenMP en CMakeLists si hace falta (silenciar warnings y enlazar)
RUN bash -lc 'cmakefile=/ws/src/ipa_coverage_planning/ipa_room_segmentation/CMakeLists.txt; \
              grep -q "find_package(OpenMP" "$cmakefile" || cat >> "$cmakefile" <<'"'"'EOF'"'"'

# ---- Optional OpenMP link (silence DEPENDS OpenMP warning) ----
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found: enabling OpenMP for ipa_room_segmentation")
  if(TARGET ipa_room_segmentation)
    target_link_libraries(ipa_room_segmentation PUBLIC OpenMP::OpenMP_CXX)
  endif()
endif()
# ----------------------------------------------------------------
EOF'

# Resolver deps y compilar para validar la imagen (como en tus logs)
WORKDIR /ws
RUN /bin/bash -lc "source /opt/ros/$ROS_DISTRO/setup.bash && \
                   rosdep install --from-paths src --ignore-src -r -y && \
                   catkin_make -DCMAKE_BUILD_TYPE=Release"

# Entrypoint ROS
COPY ./thirdparty/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
