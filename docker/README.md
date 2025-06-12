# VESC Packages build container
This is a Docker based environment to build your VESC packages.

If you don't know what Docker is look here: [Docker in 100 seconds](https://www.youtube.com/watch?v=Gjnup-PuquQ)

The Docker container contains all the packages and compiler that you need to build.

If you do not have Docker installed, look here: [Install Docker](#install-docker-engine-in-ubuntu) 

If you are running Windows, look here [Install WSL](#install-wsl)

## Make the Docker image
Using the Dockerfile in this folder you can make an image
```
sudo docker build -t vesc-package-builder:latest .
```
This will build the docker image in about 2 minutes.

## Spin up a container
You can then make a container of this image using the command:
```
sudo docker run -it -v ~/vesc:/home/vesc vesc-package-builder:latest bash
```
You are now inside the container.

_This assumes that you have or want to have VESC related software in a `vesc` folder in your home directory_

(You can use docker-compose or make a Portainer stack in stead of using the `docker run` command above. Google it...)

## Clone source code and build
If you haven't already cloned a VESC package you can do that from inside the container. (Using Refloat as example)
```
cd /home/vesc
mkdir vesc_pkg
cd vesc_pkg
git clone https://github.com/lukash/refloat.git
```
You now have a copy of the source code of the package.

If you want to do any changes to the code you can do that now. Since the `/home/vesc` folder inside the container is shared with the host operating system, you can do the changes both on the host and in the container. It's probably more convinient to do it on the host.

To build the package you will need a copy of `VESC Tool`. You have to download the linux version from here: https://vesc-project.com/vesc_tool, and put it inside the `vesc` folder. 

The different VESC packages will probably have different make configs and requirements so refer to the package doc for building. Example for building Refloat using VESC Tool v6.05:

```
cd /home/vesc/vesc_pkg/refloat
make clean
make VESC_TOOL=/home/vesc/vesc_tool_6.05 OLDVT=1
```
The build process will take some time, and create
a `[packagename].vescpkg` file

You can download this file to the device that are running VESC Tool (either PC/Mac using USB or Android/iOS using Bluetooth) and load it to your VESC as a custom package.

# Appendix

## Install Docker Engine in Ubuntu
To install Docker Engine in Ubuntu Linux follow these steps:
https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository


## Install WSL
Windows Subsystem for Linux (WSL) is a way to have Linux running inside Windows. Ubuntu is the default Linux distro. To install WSL look here: 
https://learn.microsoft.com/en-us/windows/wsl/install#install-wsl-command

Once installed you can log into Ubuntu using one simple command:
```
wsl 
```
The first time you start you are prompted to create a user

Now [install Docker](#install-docker-engine-in-ubuntu)

The C drive of Windows is available under `/mnt/c`

_Tip:_ If you clone the VESC package here: `C:\Users\username\vesc\vesc_pkg\packagename` the command to start the Docker container would be: 
```
sudo docker run -it -v /mnt/c/Users/username/vesc:/home/vesc vesc-package-builder:latest bash
```