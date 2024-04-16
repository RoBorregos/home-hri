# Script to set up permissions for ReSpeaker
# Note: if the usb is detected in another subsystem that isn't usb, the following command could be executed
# sudo echo 'ATTRS{idVendor}=="2886", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-usb-permissions.rules

# Allow usb access to respeaker without sudo in python script
sudo echo '"SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-usb-permissions.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger