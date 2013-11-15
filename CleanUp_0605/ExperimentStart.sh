#!/bin/sh
# This is a comment
echo "Release Port"
sh sigkill.sh
echo "Start Controller"
sh sigserver.sh  -w ./CleanUp.xml
