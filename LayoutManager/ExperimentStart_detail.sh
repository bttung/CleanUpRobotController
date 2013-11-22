#!/bin/sh
# This is a comment
echo "Release Port"
sh sigkill.sh
echo "Start Controller Using port: "
echo $1
sh sigserver.sh  -w ./CleanUp.xml -p $1
