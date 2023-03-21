current_dir = $(shell pwd)

download-sources:
	git clone --branch develop https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch develop https://github.com/EduArt-Robotik/edu_robot_control.git

build-docker:
	docker build -t eduard-ipc127e:0.2.0 .

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"

clean:
	rm -rf edu_robot
	rm -rf edu_robot_control
