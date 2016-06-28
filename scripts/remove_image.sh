#!/bin/bash

if [ 1 -ne `docker images $1 | wc -l` ]
then
    docker rmi $1
fi
