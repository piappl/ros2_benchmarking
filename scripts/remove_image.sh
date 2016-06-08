#!/bin/bash

if [ 1 -ne `docker images test:$1 | wc -l` ]
then
    docker rmi test:$1
fi
