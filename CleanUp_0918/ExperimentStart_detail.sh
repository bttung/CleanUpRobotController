#!/bin/sh
# This is a comment
echo "Release Port"
sh sigkill.sh
echo "Start Controller Using port: "
echo $1
sh sigserver.sh  -w ./Room0928_ObjDetect.xml -p $1
