# List available devices:
# python3 -m sounddevice

from WavUtils import WavUtils
        
if __name__ == "__main__":
    mp3_path = "say.mp3"
    WavUtils.play_mp3(mp3_path, device_index=11)

    