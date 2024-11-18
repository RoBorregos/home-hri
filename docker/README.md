# Using Docker compose to run the HRI stack

## Setting up the environment

There are several environment files and variables that must be set up before running the stack. The following environment files are used:
`network.env`: contains `ROS_MASTER_URI` and `ROS_IP`. Needed when using several devices.
`.env`: contains the same content as .env.example. This is used to setup the user to run the containers. The reason to use this file is that the variables are accessible from the docker-compose files, and that these files can use commands to get the needed ids (e.g. LOCAL_USER_ID: $(id -u)).
`.env`(in parent folder): must contain `OPENAI_API_KEY` and `ACCESS_KEY`. This file is placed here to maintain compatibility with the previous docker setup. May be migrated in the future.
`x-speech-devices` (in devices.yaml): specify the characteristics of the devices to use.

## Launch devices

To launch the devices, you can use the following command:

```bash
docker compose -f devices.yaml up
```

## Launch processing

To launch HRI processing, you can use the following command:

```bash
docker compose -f processing.yaml up
```

## Stopping

Use the following command to stop the containers:

```bash
docker compose -f devices.yaml stop
docker compose -f processing.yaml stop
```

```bash
docker compose -f devices.yaml down
docker compose -f processing.yaml down
```

## Debugging

To run both devices and processing, you can use the following command:
(When running this, the default command excludes the use of respeaker)

```bash
docker compose -f debug.yaml up
```

If no output is shown, check that network.env is set up correctly.
If there are issues with audio, debug using the commented commands (TestMic.py, TestSpeaker.py) in `devices.yaml`.
If further issues persist, check that pulseaudio is set up [correctly](https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio).
