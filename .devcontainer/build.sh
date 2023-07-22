DOCKER_BUILDKIT=1 docker build --network=host \
    --build-arg WORKSPACE=radar-rgb-fusion \
    -t mgr/mgr:humble .
