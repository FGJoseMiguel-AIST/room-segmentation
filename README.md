Here’s a cleaned-up, fully English README you can drop in as `README.md`:

---

# Dockerized ROS Noetic (desktop-full) + Room Segmentation

This repository provides a **Dockerized** ROS Noetic (desktop-full) environment with everything needed to run the **room segmentation** server and client.

* Base: `ubuntu:focal` + `ros-noetic-desktop-full`
* Runtime workspace: bind-mounted at **`/ws`** from the host’s `./ws`
* Sources are cloned during the Docker build (no sources committed under `ws/src`)

---

## Requirements

* Docker (Linux): `sudo apt-get update && sudo apt-get install -y docker.io`
* Optional (for GUI/X11 apps): `xhost +local:`

---

## Quickstart

### 1) Build the image

```bash
make build
```

### 2) Start the container in keepalive mode

```bash
make run
```

### 3) Open a shell inside the container

```bash
make shell
```

Inside the container:

```bash
source /opt/ros/noetic/setup.bash
cd /ws
# Build only if you changed sources under /ws/src
catkin_make -DCMAKE_BUILD_TYPE=Release
```

---

## Make targets

```text
make build            # Build the Docker image (ros-noetic-full:local)
make run              # Create (if needed) and start the container in keepalive mode
make start            # Start an existing container
make stop             # Stop the container
make clean            # Remove /ws/build and /ws/devel inside the container
make shell            # Open an interactive terminal inside the container
make room-segmentation# Launch server and client automatically (smoke test)
```

Useful variables (see `Makefile`): `IMAGE`, `CONTAINER`, `GPU=1`, `TZ`, `DISPLAY`, `SSH_AUTH_SOCK`.

---

## How to use the room segmentation

### Option A — Using `make`

Ensure the container is running:

```bash
make run
```

Run the end-to-end smoke test (server + client):

```bash
make room-segmentation
```

You should see logs similar to:

* Server: `Action Server for room segmentation has been initialized...`
* Client: `Segmentation OK! Rooms: X`

Stop with `Ctrl+C`.

### Option B — Using `roslaunch` directly

Open a shell and source the environment:

```bash
make shell
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
```

Run the server:

```bash
roslaunch ipa_room_segmentation room_segmentation_action_server.launch
```

In another terminal (after the server is up), run the client:

```bash
roslaunch ipa_room_segmentation_tools map_to_room_seg_client.launch
```

---

## Notes

### What the Dockerfile does

* Configures ROS Noetic apt sources and **rosdep**.
* Installs required packages:

  * Core tooling: `git`, `nano`, `build-essential`, `python3-rosdep`, `python3-rosinstall`, `python3-vcstools`
  * (Optional) `python3-catkin-tools` — you can build with `catkin_make`, so this is not strictly required
  * Libraries: `libdlib-dev`, `ros-noetic-libdlib`, `ros-noetic-opengm`
  * ROS packages: `ros-noetic-cob-map-accessibility-analysis`, `ros-noetic-move-base-msgs`
  * COIN-OR libs: `coinor-libcoinutils-dev`, `coinor-libosi-dev`, `coinor-libclp-dev`, `coinor-libcbc-dev`
  * OpenMP: `libomp-dev`
* Clones:

  * `ipa320/ipa_coverage_planning` (`-b noetic_dev`)
  * `FGJoseMiguel-AIST/room-segmentation` (`-b main`) → package `ipa_room_segmentation_tools`
* Applies small compatibility tweaks and performs a one-time build to validate.

### Runtime model

* Single workspace only: host `./ws` ↔ container `/ws`.
* If you edit anything under `./ws/src`, rebuild **inside** the container:

  ```bash
  make shell
  source /opt/ros/noetic/setup.bash
  cd /ws && rosdep install --from-paths src --ignore-src -r -y
  catkin_make -DCMAKE_BUILD_TYPE=Release
  ```

---

## Troubleshooting

### X11 access

```bash
xhost +local:
```

(Enables X11 for local Docker containers.)

### `rosdep` permissions warning

Inside the container you may see a warning about running as root; it’s safe to ignore. If you want to silence it:

```bash
rosdep fix-permissions
rosdep update
```

---

## Git hygiene

* This repo keeps `ws/src` empty in Git. Use:

  ```
  ws/build/
  ws/devel/
  ```

  in your `.gitignore` (already provided), plus a `.gitkeep` if you want to keep the folder tracked.

---

## Makefile snippet (reference)

These targets are already included; shown here for clarity:

```make
clean:
	docker exec -it $(CONTAINER) bash -lc 'rm -rf /ws/build /ws/devel && echo "Cleaned /ws"'

room-segmentation:
	# launch server in background
	- docker exec -d $(CONTAINER) bash -lc 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; \
	  roslaunch ipa_room_segmentation room_segmentation_action_server.launch'
	# give the server a moment to start
	sleep 2
	# run client in foreground
	docker exec -it $(CONTAINER) bash -lc 'source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash; \
	  roslaunch ipa_room_segmentation_tools map_to_room_seg_client.launch'
```

---
