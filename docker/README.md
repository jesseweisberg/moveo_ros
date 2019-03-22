# Docker image for Kinetic

## How to install docker-nvidia2

Add nvidia docker repository as it is explained [here](https://nvidia.github.io/nvidia-docker/).

```
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
```

Install nvidia-docker and reload docker daemon as it is explained [here](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-2.0)).

```
sudo apt-get install nvidia-docker2
sudo pkill -SIGHUP dockerd
```

## How to build the image

```
docker build --force-rm --build-arg UID=$(id -u) --build-arg GID=$(id -g) --tag moveo_kinetic .
```


