xhost local:root
docker run -it \
	-v $PWD/catkin_ws:/home/catkin_ws/:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	--network="host"\
	--name='control'\
	-e XAUTHORITYS\
	-e DISPLAY\
	--device /dev/dri/\
	an2ancan/ros_dev\
	env TERM=xterm-256color\
	/bin/bash
