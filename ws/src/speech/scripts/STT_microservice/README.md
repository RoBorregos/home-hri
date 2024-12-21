# Speech To Text Microservice

Contains two gRPC servers, one uses whisper and the other faster-whisper for improved speed and precision. To run with cuda on l4t a docker image was generated with CTranslate2. You can pull it by running:

```bash
docker pull tacolon27/faster-whisper-l4t-cuda
```

Next, navigate to ws/src/speech/scripts/ and build a container by running:

```bash
docker run -it -p 8888:8888 --name fast-whisper -v .:/workspace tacolon27/faster-whisper-l4t-cuda
```

Finally, to start a server just run the perferred script.

```bash
python3 Faster-whisper.py
python3 Whisper.py
```