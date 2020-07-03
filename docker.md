# Docker Container

1. Install docker engine [https://docs.docker.com](https://docs.docker.com/engine/install/ubuntu/#installation-methods) 
1. run
```bash
docker run --rm -p 7000:7000 -p 8888:8888 \
-v summer-school:/home/student \
-it memoryofmotion/summer-school
```
1. And open the link with `127.0.0.1` in your web browser. You can then run ~/load_talos.ipynb to say hi to Talos!
1. Notebooks for the tutorials are in ~/summer-school/tutorials.


## Update the notebooks

The VirtualBox or Docker images might not contain the latest version of the notebooks so please pull the latest modifications before the tutorials. To do that :
1. run
```bash
docker run --rm -v summer-school:/home/student \
-it memoryofmotion/summer-school \
git -C summer-school pull --rebase --recurse-submodules --ff-only
```
