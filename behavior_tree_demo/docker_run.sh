#!/bin/bash

XAUTH=/tmp/.docker.xauth
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
	echo $xauth_list | xauth -f $XAUTH nmerge -
    else
	touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker pull osrf/ros:humble-desktop-full

docker run \
       --privileged \
       --name behavior_tree_demo \
       --entrypoint bash -it \
       --gpus all \
       --rm \
       --network=host \
       --env "ACCEPT_EULA=Y" \
       --env "PRIVACY_CONSENT=Y" \
       --env="DISPLAY=$DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --env="XAUTHORITY=$XAUTH" \
       -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
       -v $XAUTH:$XAUTH \
       -v $SCRIPT_DIR/workspace:/workspace:rw \
       -v $SCRIPT_DIR/extras/.bashrc:/root/.bashrc \
       -v $SCRIPT_DIR/extras/.bash_history:/root/.bash_history \
       -v $SCRIPT_DIR/extras/inputrc:/etc/inputrc \
       osrf/ros:humble-desktop-full
