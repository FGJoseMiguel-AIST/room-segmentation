# --------- Config ----------
IMAGE        ?= ros-noetic-full:local
CONTAINER    ?= ros_noetic_dev
WORKDIR      ?= $(PWD)

# GPU: usa "make run GPU=1" para habilitar GPU (si hay soporte)
GPU          ?= 0
ifeq ($(GPU),1)
  GPU_FLAGS  := --gpus all
else
  GPU_FLAGS  :=
endif

# X11 (Linux)
DISPLAY_HOST ?= $(DISPLAY)
XSOCK        ?= /tmp/.X11-unix
XAUTH        ?= $(HOME)/.Xauthority

# Zona horaria
TZ           ?= Asia/Tokyo

# Comando keepalive (permanente)
KEEPALIVE ?= bash -lc "source /opt/ros/noetic/setup.bash; \
  if [ -f /ws/devel/setup.bash ]; then source /ws/devel/setup.bash; fi; \
  while sleep 3600; do :; done"

# Flags de ejecución (solo /ws)
RUN_FLAGS := \
	--name $(CONTAINER) \
	--privileged \
	--net=host \
	--ipc=host \
	--pid=host \
	--restart unless-stopped \
	--security-opt seccomp=unconfined \
	--security-opt apparmor=unconfined \
	--add-host=host.docker.internal:host-gateway \
	-e TZ=$(TZ) \
	-e DISPLAY=$(DISPLAY_HOST) \
	-e XAUTHORITY=/root/.Xauthority \
	-v $(XSOCK):/tmp/.X11-unix:rw \
	$(if $(wildcard $(XAUTH)),-v $(XAUTH):/root/.Xauthority:rw,) \
	-v $(WORKDIR)/ws:/ws:rw \
	-v /dev:/dev \
	-v /run/dbus:/run/dbus \
	-v /etc/localtime:/etc/localtime:ro \
	$(if $(SSH_AUTH_SOCK),-e SSH_AUTH_SOCK=/ssh-agent -v $(SSH_AUTH_SOCK):/ssh-agent,) \
	-v $(HOME)/.gitconfig:/root/.gitconfig:ro \
	-v $(HOME)/.ssh/known_hosts:/root/.ssh/known_hosts:rw

# --------- Helpers ----------
_exists_container = $(shell docker ps -a --format '{{.Names}}' | grep -w $(CONTAINER) || true)
_is_running       = $(shell docker ps --format '{{.Names}}' | grep -w $(CONTAINER) || true)

.PHONY: help build run create start stop clean shell room-segmentation logs rm rmi ps xhost-allow xhost-revoke

help:
	@echo "Targets:"
	@echo "  make build            - Construye la imagen ($(IMAGE))"
	@echo "  make run              - Crea (si no existe) e inicia el contenedor (permanente)"
	@echo "  make start            - Inicia el contenedor ya creado"
	@echo "  make stop             - Detiene el contenedor"
	@echo "  make clean            - Limpia /ws/build y /ws/devel dentro del contenedor"
	@echo "  make shell            - Abre una shell dentro del contenedor"
	@echo "  make room-segmentation- Lanza server y luego client de room segmentation"
	@echo "  make logs             - Muestra logs del contenedor"
	@echo "  make rm               - Elimina el contenedor"
	@echo "  make rmi              - Borra la imagen"
	@echo "  -- Variables útiles: IMAGE, CONTAINER, GPU=1"

build:
	docker build -t $(IMAGE) .

run: create start
	@echo "Contenedor '$(CONTAINER)' en ejecución (modo permanente)."

create:
ifneq ($(_exists_container),)
	@echo "El contenedor '$(CONTAINER)' ya existe. Omitiendo create."
else
	docker create $(RUN_FLAGS) $(GPU_FLAGS) $(IMAGE) $(KEEPALIVE)
	@echo "Contenedor '$(CONTAINER)' creado."
endif

start:
ifneq ($(_is_running),)
	@echo "El contenedor '$(CONTAINER)' ya está en ejecución."
else
	docker start $(CONTAINER)
	@echo "Contenedor '$(CONTAINER)' iniciado."
endif

stop:
ifneq ($(_is_running),)
	docker stop $(CONTAINER)
	@echo "Contenedor '$(CONTAINER)' detenido."
else
	@echo "El contenedor '$(CONTAINER)' no está ejecutándose."
endif

clean:
ifneq ($(_is_running),)
	docker exec -it $(CONTAINER) bash -lc 'rm -rf /ws/build /ws/devel && echo "Limpio /ws/build y /ws/devel"'
else
	@echo "Arranca el contenedor primero (make run)."
endif

shell:
	docker exec -it $(CONTAINER) bash

room-segmentation:
	# Arranca el servidor en background dentro del contenedor
	docker exec -d $(CONTAINER) bash -lc 'source /opt/ros/noetic/setup.bash && \
	  [ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash || true; \
	  roslaunch ipa_room_segmentation room_segmentation_action_server.launch ns:=/room_segmentation'
	# Pequeña espera y luego cliente en la consola actual
	sleep 2
	docker exec -it $(CONTAINER) bash -lc 'source /opt/ros/noetic/setup.bash && \
	  [ -f /ws/devel/setup.bash ] && source /ws/devel/setup.bash || true; \
	  roslaunch ipa_room_segmentation_tools map_to_room_seg_client.launch'

logs:
	docker logs -f --tail=200 $(CONTAINER)

rm:
ifneq ($(_exists_container),)
	docker rm -f $(CONTAINER)
	@echo "Contenedor '$(CONTAINER)' eliminado."
else
	@echo "No existe el contenedor '$(CONTAINER)'."
endif

rmi:
	docker rmi $(IMAGE) || true

ps:
	docker ps -a --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"

# Permisos X11 (opcional)
xhost-allow:
	xhost +local:

xhost-revoke:
	xhost -local:
