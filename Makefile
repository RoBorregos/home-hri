# General commands to interact with HRI container

# Note: if the docker container is instantiated with a non-root user, the following command should be executed in the host machine to provide access:
# sudo usermod -aG audio $USER # Make sure current user has access to audio resources.

UID := $(shell id -u)
GID := $(shell id -g)

hri.build:
	@docker build -f docker/Dockerfile.hri -t roborregos/home:hri-base .

hri.build.cuda:
	@docker build -f docker/Dockerfile.hri.cuda -t roborregos/home:hri-base .

hri.create:
	@docker run -it --name home-hri --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$(DISPLAY) -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 --device /dev/snd:/dev/snd --user $(UID):$(GID) -v ${PWD}:/workspace --env-file .env roborregos/home:hri-base2 bash

hri.create.cuda:
	@docker run -it --name home-hri --gpus all --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$(DISPLAY) -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 --device /dev/snd:/dev/snd --user $(UID):$(GID) -v ${PWD}:/workspace --env-file .env roborregos/home:hri-base bash

hri.stop:
	@docker stop home-hri

hri.start:
	@docker start home-hri

hri.enter:
	@docker exec --user $(UID):$(GID) -it home-hri /bin/bash

hri.rm:
	@docker rm home-hri

hri.rmi:
	@docker rmi roborregos/home:hri-base
