FROM ubuntu:18.04

ADD setup.sh robotpkg.key /

RUN /setup.sh
RUN useradd -m student

USER student
WORKDIR /home/student

ENV PATH=/opt/openrobots/bin:$PATH \
    LD_LIBRARY_PATH=/opt/openrobots/lib

EXPOSE 6000 7000 8888
CMD jupyter notebook --no-browser --ip='*'

VOLUME /home/student

# (in)validate docker build cache with the last commit from github
ADD https://api.github.com/repos/memory-of-motion/summer-school/commits/master /
RUN git clone --recursive https://github.com/memory-of-motion/summer-school.git \
 && ln -s summer-school/setup/load_talos.ipynb
