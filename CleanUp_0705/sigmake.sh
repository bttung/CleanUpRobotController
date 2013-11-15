#################################
# compile SIGVerse controller
# e.g.
# $ ./sigmake.sh Controller.cpp
# $ ./sigmake.sh clean
#################################
#!/bin/sh

# make clean
arg1=$1
if [ "${arg1}" = "clean" ]; then
make clean

# set OBJS
else
for arg in $@
do
case ${arg} in 
*\.cpp)
export OBJS="$OBJS ${arg%.cpp}.so";
esac
done

# complile
make
fi
