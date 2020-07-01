# 2020 Summer School software setup

This setup is provided as a [Docker](https://www.docker.com/) image and a [VirtualBox](https://www.virtualbox.org/)
image. We expect better performances with Docker, but VirtualBox is easier to use and more bullet-proof.

Please choose one :)

*If you feel adventurous, you can also try to setup the softwares directly on your computer, but you're on your own: due
to limited time, no support will be provided by the Summer School, you'll have to ask for help in the project's issue
trackers, without guarantees*

## Start the image and say hi to TALOS

### with Docker

```bash
docker run --rm -p 7000:7000 -p 8888:8888 -it memoryofmotion/summer-school
```

And open the link with `127.0.0.1` in your web browser

### with VirtualBox

Start the VirtualBox Image.

## Build

:warning: this section is not required for students. It is just here for documentation purposes.

### Docker image

```bash
docker build -t memoryofmotion/summer-school .
docker push memoryofmotion/summer-school
```

## VirtualBox Image

1. Start a new VM on a [Ubuntu 18.04 ISO](https://releases.ubuntu.com/18.04/ubuntu-18.04.4-desktop-amd64.iso)
2. Choose a minimal installation with all defaults
3. Create a `student` main user with a `student` password
4. disable Screen Lock (settings / privacy)
5. `sudo apt update && sudo apt install -qqy git && git clone https://github.com/memory-of-motion/summer-school.git`
6. `cd summer-school/setup`
7. `sudo ./setup.sh`
8. `bash`
9. `jupyter notebook`
