current_dir = $(shell pwd)

download-sources:
	# mkdir -p /tmp/eduart_robot_ws
	# cp Dockerfile /tmp/eduart_robot_ws
	# cd /tmp/eduart_robot_ws
	git clone --branch main https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch main https://github.com/EduArt-Robotik/edu_robot_control.git
	# cd $(current_dir)

build-docker: download-sources
	# cd /tmp/eduart_robot_ws
	docker build -t eduard-iotbot:alpha .
	# cd $(current_dir)

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"

clean:
	rm -r edu_robot
	rm -r edu_robot_control
