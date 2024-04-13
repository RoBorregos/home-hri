# Script to set up permissions for ReSpeaker

# Allow usb access to respeaker without sudo in python script
sudo echo '"SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-usb-permissions.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger