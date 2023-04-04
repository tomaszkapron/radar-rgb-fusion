rocker --network host --privileged --nvidia --x11 --user --group-add plugdev --group-add sudo --name ros_container \
    --env "USER" \
    --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --env "ROS_DOMAIN_ID=0" \
    --volume "${PWD}:${HOME}/${PWD##*/}" \
    -- mgr/moveit:humble 
