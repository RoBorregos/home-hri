# Create pulse audio socket to share with docker container

# Ref: https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio

# Create pulseaudio socket.
pactl load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket

# Create pulseaudio clients config.
echo 'default-server = unix:/tmp/pulseaudio.socket
# Prevent a server running in the container
autospawn = yes
daemon-binary = /bin/true
# Prevent the use of shared memory
enable-shm = false' > /tmp/pulseaudio.client.conf
