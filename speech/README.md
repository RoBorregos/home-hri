# Speech @Home




## TO-DO
### Models to test

#### MDTC -> con HeySnips Dataset
- [PaddleSpeech](https://github.com/PaddlePaddle/PaddleSpeech/tree/develop/examples/hey_snips/kws0)
- [Dataset HeySnips](https://github.com/sonos/keyword-spotting-research-datasets)
- [Paper](https://arxiv.org/pdf/2102.13552.pdf) mdtc model

#### Data2vec
- https://github.com/holgerbovbjerg/data2vec-kws

## Other

### Run PaddleSpeech

```bash
# Pull images
docker run --name dev --runtime=nvidia -v $PWD:/mnt -p 8888:8888 -it paddlecloud/paddlespeech:develop-gpu-cuda10.2-cudnn7-fb4d25 /bin/bash
# Run jupyter notebook
jupyter lab --ip=0.0.0.0 --port=8888 --allow-root --notebook-dir=/home
```
