 docker run  -it --rm \
        --env DISPLAY=$DISPLAY \
        --network host \
        --device=/dev/input/js0 \
        --name remote_droid_bot \
        --volume ./ros2_ws:/droid_remote/ws \
        remote_droid


