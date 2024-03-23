## Init NLP setup

In the root folder (`hri`), create a `.env` file based on `.env.example` and add the credentials. 

Build this image, must be in root directory:

```bash
docker build -f docker/Dockerfile.hri -t home/hri:base .
```

Run the image, the `--env-file` passes the environment variables:

```bash
docker run -it --name home-hri --net=host --privileged --env="QT_X11_NO_MITSHM=1" -e DISPLAY=$DISPLAY -eQT_DEBUG_PLUGINS=1 -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/video0:/dev/video0 --user $(id -u):$(id -g) -v $(pwd):/workspace --env-file .env home/hri:base bash
```

Test the initial setup:
```bash
python3 nlp/test-setup.py
```

The output should be: `go kitchen, grab spoon`

**Useful command found**

Remove unused docker images:

```bash
docker rmi $(docker images --filter "dangling=true" -q --no-trunc)
```