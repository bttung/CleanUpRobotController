#!/bin/bash

for i in `ps aux | grep sigserver | grep $(whoami) | awk '{print $2}'`; do
    echo process id: $i
    kill -9 $i
done
