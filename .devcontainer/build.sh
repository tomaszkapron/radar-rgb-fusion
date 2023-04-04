DOCKER_BUILDKIT=1 docker build --network=host \
    --build-arg WORKSPACE=mgr_ws \
    -t mgr/moveit:humble .
