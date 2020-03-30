#!/bin/bash

function usage()
{
    echo "To run the docker for waymo-viewer:"
    echo ""
    echo "./docker_run.sh [PATH_TO_DATA_SEGMENT] [OUTPUT_PATH]"
}


BASH_OPTION="roslaunch waymo_viewer recorder.launch"
if [ -z "$1" ]; then
    usage
    exit
else
    TF_PATH=$1
    BAG_PATH=$2
    echo "TF_PATH = $TF_PATH"
    echo "BAG_PATH = $BAG_PATH"
fi

BASH_OPTION=bash

echo "running $BASH_OPTION in docker"

docker run -it --rm \
        --network host \
        --privileged \
        --name waymo-viewer \
        -e DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -v "$XAUTH:$XAUTH" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix" \
        -e ROS_MASTER_URI=$ROS_MASTER_URI \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v "$TF_PATH:/tf_files" \
        -v "$BAG_PATH:/bag_files" \
        -v "${PWD}/catkin_ws:/waymo_ws" \
        -v "${PWD}/waymo-od:/waymo-od" \
	    -w /waymo_ws \
        biomotion/waymo-viewer:1.0 $BASH_OPTION