# Dockerized ROS Noetic (desktop-full) + Room Segmentation

This repository provides a **Dockerized** ROS Noetic environment (desktop-full) plus everything required to run the **room segmentation** server and client.

- Base: `ubuntu:focal` + `ros-noetic-desktop-full`
- Runtime workspace: bind-mounted at **`/ws`** (from `./ws` on host)
- Sources are cloned during the Docker build (no sources committed under `ws/src`)

---

## Requirements

- Docker (Linux): `sudo apt-get update && sudo apt-get install -y docker.io`
- Optional: X11 access for GUI tools (`xhost +local:`)

---

## Quickstart

```bash
# 1) Build the image
make build

# 2) Run the container in "permanent" mode (keepalive)
make run

# 3) Open a shell inside the container
make shell

Inside the container:

source /opt/ros/noetic/setup.bash
cd /ws
# Build only if you changed sources under /ws/src
catkin_make -DCMAKE_BUILD_TYPE=Release

Make targets

    make build – Build the Docker image (ros-noetic-full:local).

    make run – Create (if needed) and start the container in keepalive mode.

    make start – Start an existing container.

    make stop – Stop the container.

    make clean – Remove /ws/build and /ws/devel inside the container.

    make shell – Open an interactive terminal inside the container.

    make room-segmentation – Launch server and client automatically.

    Variables útiles:
    IMAGE, CONTAINER, GPU=1, TZ, DISPLAY, SSH_AUTH_SOCK (ver Makefile).

How to use the room segmentator
Option A: Using Make

# ensure the container is running
make run

# run the end-to-end smoke test (server + client)
make room-segmentation

You should see logs like:

    Server: “Action Server for room segmentation has been initialized…”

    Client: “¡Segmentación OK! Rooms: X”

Stop with Ctrl+C.
Option B: roslaunch commands

Open a shell:

make shell
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash

Run the server:

roslaunch ipa_room_segmentation room_segmentation_action_server.launch

In another terminal (or after the server is up), run the client:

roslaunch ipa_room_segmentation_tools map_to_room_seg_client.launch

Notes

    The Dockerfile:

        Configures ROS Noetic apt sources and rosdep.

        Installs required packages:

            git, nano, build-essential, python3-rosdep, python3-rosinstall, python3-vcstools

            python3-catkin-tools (via ros-shadow-fixed)

            libdlib-dev, ros-noetic-libdlib, ros-noetic-opengm

            ros-noetic-cob-map-accessibility-analysis, ros-noetic-move-base-msgs

            COIN-OR libs: coinor-libcoinutils-dev, coinor-libosi-dev, coinor-libclp-dev, coinor-libcbc-dev

            libomp-dev

        Clones:

            ipa320/ipa_coverage_planning (-b noetic_dev)

            FGJoseMiguel-AIST/room-segmentation (-b main) → package ipa_room_segmentation_tools

        Applies small compatibility tweaks and compiles once to validate.

    Runtime:

        Only one workspace is used: host ./ws ↔ container /ws.

        If you edit anything under ./ws/src, rebuild inside the container:

        make shell
        source /opt/ros/noetic/setup.bash
        cd /ws && rosdep install --from-paths src --ignore-src -r -y
        catkin_make -DCMAKE_BUILD_TYPE=Release

Troubleshooting

    xhost:

xhost +local:

rosdep permissions:
Run rosdep without sudo inside the container. If you ever see a warning about running as root, it’s safe to ignore inside the container, or fix with:

    rosdep fix-permissions
    rosdep update

License

MIT (unless upstream packages state otherwise).


---

## 3) Asegura que el Makefile tenga los targets clave

Si ya tienes el Makefile que te propuse, solo añade/asegura estos dos:

```make
clean:
	docker exec -it $(CONTAINER) bash -lc 'rm -rf /ws/build /ws/devel && echo "Limpieza /ws OK"'

room-segmentation:
	# lansa el server en background (en el contenedor)
	- docker exec -d $(CONTAINER) bash -lc 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; \
	  roslaunch ipa_room_segmentation room_segmentation_action_server.launch'
	# pequeña espera para que el server arranque
	sleep 2
	# cliente en primer plano
	docker exec -it $(CONTAINER) bash -lc 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; \
	  roslaunch ipa_room_segmentation_tools map_to_room_seg_client.launch'