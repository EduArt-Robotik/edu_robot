current_dir = $(shell pwd)

download-sources:
	# mkdir -p /tmp/eduart_robot_ws
	# cp Dockerfile /tmp/eduart_robot_ws
	# cd /tmp/eduart_robot_ws
	git clone --branch 0.1.0-alpha https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch main https://github.com/EduArt-Robotik/edu_robot_control.git
	# cd $(current_dir)

build-docker: download-sources
	docker build -t eduard-robot-monitoring:alpha .

all: download-sources build-docker
	echo "build edu_robot_control into a docker image"

clean:
	rm -rf edu_robot
	rm -rf edu_robot_control
