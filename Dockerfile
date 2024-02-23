# syntax = devthefuture/dockerfile-x

INCLUDE ./src/mas_simulation/Dockerfile

RUN apt-get update; apt-get install -y python3-pip && pip install networkx[default] numpy matplotlib zarr torch gymnasium

RUN pip install wandb pettingzoo setproctitle absl-py