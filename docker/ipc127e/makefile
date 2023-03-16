current_dir = $(shell pwd)

download-sources:
	# mkdir -p /tmp/eduart_robot_ws
	# cp Dockerfile /tmp/eduart_robot_ws
	# cd /tmp/eduart_robot_ws
	git clone --branch develop https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch develop https://github.com/EduArt-Robotik/edu_robot_control.git
	# cd $(current_dir)

build-docker: download-sources
	# cd /tmp/eduart_robot_ws
	docker build -t eduard-ipc127e:0.2.0 .
	# cd $(current_dir)

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"

clean:
	rm -rf edu_robot
	rm -rf edu_robot_control
