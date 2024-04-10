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

# No GPU
hri.build:
	@./docker/scripts/build.bash --area=hri

# CUDA 11.8 x86_64
hri.build.cuda:
	@./docker/scripts/build.bash --area=hri --use-cuda

# Jetson devices
hri.build.jetson:
	@./docker/scripts/build.bash --area=hri --jetson-l4t=35.4.1

# ----------------------------CREATE------------------------------------

hri.create:
	@./docker/scripts/speech.bash
	@./docker/scripts/run.bash --area=hri --volumes=$(volumes) --name=$(name)

hri.create.cuda:
	@./docker/scripts/speech.bash
	@./docker/scripts/run.bash --area=hri --use-cuda --volumes=$(volumes) --name=$(name)

# For jetpack version 35.4.1, jetson images are special in the sense that they are specific to the jetpack version
hri.create.jetson:
	@./docker/scripts/run.bash --area=hri --jetson-l4t=35.4.1 --volumes=$(volumes) --name=$(name)

# ----------------------------START------------------------------------
# Start containers
hri.up:
	@./docker/scripts/speech.bash
	@xhost +
	@docker start home-hri

# ----------------------------STOP------------------------------------
# Stop containers
hri.down:
	@docker stop home-hri 

# ----------------------------RESTART------------------------------------
# Restart containers
hri.restart:
	@docker restart home-hri 

# ----------------------------LOGS------------------------------------
# Logs of the container
hri.logs:
	@docker logs --tail 50 home-hri

# ----------------------------SHELL------------------------------------
# Fires up a bash session inside the container
hri.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) home-hri bash

# ----------------------------REMOVE------------------------------------
# Remove container
hri.remove:
	@docker container rm home-hri

# ----------------------------------------------------------------------
#  General Docker Utilities

#: Show a list of images.
list-images:
	@docker image ls

#: Show a list of containers.
list-containers:
	@docker container ls -as
