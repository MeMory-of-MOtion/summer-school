FROM gepetto/buildfarm:robotpkg-bin-18.04

RUN apt-get update -qqy \
 && apt-get install -qqy \
    python3-matplotlib \
    python3-pip \
    robotpkg-py3\*-crocoddyl \
    robotpkg-py3\*-tsid \
 && pip3 install --no-cache-dir \
    jupyter \
    meshcat \
 && useradd -m student \
 && echo /opt/openrobots/lib/python3.6/site-packages > /usr/lib/python3/dist-packages/robotpkg.pth

ENV PATH=/opt/openrobots/bin:$PATH LD_LIBRARY_PATH=/opt/openrobots/lib

USER student
WORKDIR /home/student

EXPOSE 6000 7000 8888
CMD jupyter notebook --no-browser --ip='*'

ADD *.ipynb ./
