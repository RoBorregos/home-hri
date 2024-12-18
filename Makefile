# ----------------------------------------------------------------------
#  Robocup@Home ROS Noetic Docker Development
# ----------------------------------------------------------------------

#: Builds a Docker image with the corresponding Dockerfile file


# ----------------------------BUILD------------------------------------
# ---------hri----------

# NOTE: if the docker container is created with a non-root user, the following command should be executed 
# in the host machine to provide access (after every machine reboot):

# sudo usermod -aG audio $USER # Make sure current user has access to audio resources.
# sudo chmod 777 /dev/snd/* # Allow access to audio devices.

DISPLAY_NAME := "home-hri-display"

# No GPU
hri.build:
	@./docker/scripts/build.bash --area=hri
	
hri.build.ros2:
	@./docker/scripts/build.bash --area=hri --ros2
# CUDA 11.8 x86_64
hri.build.cuda:
	@./docker/scripts/build.bash --area=hri --use-cuda

# Jetson devices
hri.build.jetson:
	@./docker/scripts/build.bash --area=hri --jetson-l4t=35.4.1

# Display
hri.build.display:
	@./docker/scripts/build.bash --area=hri --is-display

# ----------------------------CREATE------------------------------------

hri.create:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@./docker/scripts/speech.bash
	@./docker/scripts/run.bash --area=hri --is-speech --volumes=$(volumes) --name=$(name)

hri.create.cuda:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@./docker/scripts/speech.bash
	@./docker/scripts/run.bash --area=hri --use-cuda --is-speech --volumes=$(volumes) --name=$(name)

# For jetpack version 35.4.1, jetson images are special in the sense that they are specific to the jetpack version
hri.create.jetson:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@./docker/scripts/speech.bash
	@./docker/scripts/run.bash --area=hri --jetson-l4t=35.4.1 --is-speech --volumes=$(volumes) --name=$(name)

hri.create.display:
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@./docker/scripts/run.bash --area=hri --is-display --volumes=$(volumes) --name=$(DISPLAY_NAME)


# ----------------------------START------------------------------------
# Start containers
hri.up:
	@./docker/scripts/speech.bash
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@docker start home-hri

hri.up.jetson:
	@./docker/scripts/speech.bash
	@(if [ ! -z ${DISPLAY} ]; then xhost +; fi)
	@docker start home-hri

hri.up.display:
	@docker start $(DISPLAY_NAME)

# ----------------------------STOP------------------------------------
# Stop containers
hri.down:
	@docker stop home-hri 

hri.down.display:
	@docker stop $(DISPLAY_NAME) 

# ----------------------------RESTART------------------------------------
# Restart containers
hri.restart:
	@docker restart home-hri 

hri.restart.display:
	@docker restart $(DISPLAY_NAME)

# ----------------------------LOGS------------------------------------
# Logs of the container
hri.logs:
	@docker logs --tail 50 home-hri

hri.logs.display:
	@docker logs --tail 50 $(DISPLAY_NAME)

# ----------------------------SHELL------------------------------------
# Fires up a bash session inside the container
hri.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) home-hri bash

hri.shell.display:
	@docker exec -it --user $(shell id -u):$(shell id -g) $(DISPLAY_NAME) bash

hri.execute.display:
	@docker exec -it $(DISPLAY_NAME) npm --prefix "/workspace/display/display" run start

# ----------------------------REMOVE------------------------------------
# Remove container
hri.remove:
	@docker container rm home-hri

hri.remove.display:
	@docker container rm $(DISPLAY_NAME)

# ----------------------------------------------------------------------
#  General Docker Utilities

#: Show a list of images.
list-images:
	@docker image ls

#: Show a list of containers.
list-containers:
	@docker container ls -as
