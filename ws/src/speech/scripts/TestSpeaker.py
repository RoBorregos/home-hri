# List available devices:
# python3 -m sounddevice

from WavUtils import WavUtils

import sounddevice as sd
def get_devices():
    devices = sd.query_devices()
    num_dev = 0
    for device_info in devices:
        print(f"Device {num_dev}: {device_info['name']}, {device_info['max_input_channels']} input channels, {device_info['max_output_channels']} output channels")
        num_dev = num_dev + 1 
    
    # print("Original order:")
    # print(devices)

if __name__ == "__main__":
    # mp3_path = "say.mp3"
    # WavUtils.play_mp3(mp3_path, device_index=11)
    get_devices()

    