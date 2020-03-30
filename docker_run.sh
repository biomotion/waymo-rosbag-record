#!/bin/bash

function usage()
{
    echo "To run the docker for waymo-viewer:"
    echo ""
    echo "./run.sh [PATH_TO_DATA_SEGMENT] [OUTPUT_PATH]"
}

docker_runtime=runc
# if [ "$(docker info|grep nvidia)" == "" ]; then
#     echo "no nvidia-docker available"
# else
# 	echo "nvidia-docker found"
# 	docker_runtime=nvidia
# fi

# XAUTH=/tmp/.docker.xauth
# if [ ! -f $XAUTH ]
# then
#     xauth_list=$(xauth nlist $DISPLAY)
#     xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
#     if [ ! -z "$xauth_list" ]
#     then
#         echo "$xauth_list" | xauth -f $XAUTH nmerge -
#     else
#         touch $XAUTH
#     fi
#     chmod a+r $XAUTH
# fi

# # Prevent executing "docker run" when xauth failed.
# if [ ! -f $XAUTH ]; then
#     echo "[$XAUTH] was not properly created. Exiting..."
#     exit 1
# fi

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
        --runtime=$docker_runtime \
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
