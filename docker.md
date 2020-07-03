# Docker Container

1. Install docker engine [https://docs.docker.com](https://docs.docker.com/engine/install/ubuntu/#installation-methods) 
1. run
```bash
docker volume create summer-school
docker run --rm -p 7000:7000 -p 8888:8888 -v summer-school:/home/student -it memoryofmotion/summer-school
```
1. And open the link with `127.0.0.1` in your web browser. Say Hi to Talos!

## Update the notebooks

The VirtualBox or Docker images might not contain the latest version of the tutorials so please pull the latest modifications before each tutorial.

