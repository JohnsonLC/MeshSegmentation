#/bin/bash

# 默认是小恐龙
cmd="./bin/mesh_segment"
model="../model/dinosaur.2k.obj"
k="5"
eposilon="0.05"
mode=$2

if [ $1 = "cube" ]
then
    model="../model/cube.obj"
    k="2"
    eposilon="0.1"
elif [ $1 = "block" ]
then 
    model="../model/block.obj"
    k="3"
    eposilon="0.01"
fi

$cmd $model $k $eposilon $mode