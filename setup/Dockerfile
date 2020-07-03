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

ADD *.ipynb ./
VOLUME /home/student
RUN git clone --recursive https://github.com/memory-of-motion/summer-school.git
