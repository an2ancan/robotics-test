xhost local:root
docker run -it \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	--network="host"\
	--name='control'\
	-e XAUTHORITYS\
	-e DISPLAY\
	--device /dev/dri/\
	docker.io/an2ancan/ros_dev:latest\
	env TERM=xterm-256color\
	/bin/bash
