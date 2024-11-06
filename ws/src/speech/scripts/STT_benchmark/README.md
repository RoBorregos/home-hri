# Speech To Text benchmarks

_faster-whisper.py_ and _whisper_test.py_ are two scripts which translate the 10 .wav audio files found in recordings.

To run with cuda a docker image was generated with CTranslate2 compiled with support. You can pull it by running:

```
docker pull tacolon27/faster-whisper-l4t-cuda
```

The results for the tests were that faster-whisper runs consistently in half the time than normal whisper transcribing audios of 10s in just half a second. To view results check _whispers benchmark.csv_.
