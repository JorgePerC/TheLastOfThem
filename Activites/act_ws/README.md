# Dockerizing ROS

So... You assuming you know what Docker is, you'll realize that there is a problem to directly running a Jetson conainer on your machine. This is because CPU architectures. Your computer is likely to be AMDx64, but the Jetson Nano is ARM_64. So... You'll need to install extra things 

## Steps Fedora

1. Install Docker

1. Download a Docker Image for the Jetson Nano from [here](https://developer.nvidia.com/embedded/learn/tutorials/jetson-container)


    sudo dnf install qemu-user-binfmt


## How I learned this:

[Multi-platform Docker](https://docs.docker.com/build/building/multi-platform/#:~:text=Docker%20Desktop%20provides%20binfmt_misc%20multi,the%20Docker%20for%20Mac%20VM.)

[Docker on different CPU arachitectures](https://medium.com/@Smartcow_ai/building-arm64-based-docker-containers-for-nvidia-jetson-devices-on-an-x86-based-host-d72cfa535786)

https://developer.nvidia.com/embedded/downloads

https://collabnix.com/building-your-first-jetson-container/


