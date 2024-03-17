# Porcupine demo


Run demo using pvporcupinedemo
```bash
    python3 -m venv venv # Create env in cwd
    source venv/bin/activate # activate venv
    sudo pip3 install pvporcupinedemo # Install quick demo

    export ACCESS_KEY="your_access_key"
    export KEYWORD_FILE_PATH="path to custom keyword"

    # Run demo with custom keyword
    porcupine_demo_mic --access_key $ACCESS_KEY --keyword_paths $KEYWORD_FILE_PATH

    # Run demo with predefined keywords
    porcupine_demo_mic --access_key $ACCESS_KEY --keywords picovoice porcupine

    # Show audio devices
    porcupine_demo_mic --show_audio_devices

    # Specify audio device
    porcupine_demo_mic --access_key ${ACCESS_KEY} --keywords picovoice --audio_device_index 0

```

Run demo using demo.py
```bash
    python3 -m venv venv # Create env in cwd
    source venv/bin/activate # activate venv
    pip install -r requirements
    # Make sure to create an .env (see .env.example)
    python3 demo.py # Run demo
```