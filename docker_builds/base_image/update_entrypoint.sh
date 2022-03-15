#!/bin/bash

if [[ ! -f "/docker_entrypoint.sh" ]];
then
	touch /docker_entrypoint.sh
	chmod 755 /docker_entrypoint.sh

    echo '#!/bin/bash
set -e
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"' > /docker_entrypoint.sh

fi

sed -i 's/exec "$@"/source "\/home\/nvidia/\jetson_robot_car\/install\/local_setup.sh"\nexec "$@"/g' /ros_entrypoint.sh