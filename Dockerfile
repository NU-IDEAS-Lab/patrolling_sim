# syntax = edrevo/dockerfile-plus

INCLUDE+ ./src/mas_simulation/Dockerfile

RUN apt-get update; apt-get install -y python3-pip && pip install networkx[default] numpy matplotlib zarr torch gymnasium