#!/usr/bin/env sh

BASE="mecademic:latest"

# Build image
# docker build -t $BASE .

# Run container
docker run --rm -it \
    -v `pwd`:/input \
    $BASE python /input/test.py
