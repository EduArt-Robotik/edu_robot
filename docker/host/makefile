current_dir = $(shell pwd)

download-sources:
	git clone --branch feature/prepare_release_0.2.0 https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch develop https://github.com/EduArt-Robotik/edu_robot_control.git
	git clone --branch ros2 https://github.com/EduArt-Robotik/joystick_drivers.git

build-docker:
	docker build -t eduard-robot-monitoring:0.2.0 .

all: download-sources build-docker
	echo "build edu_robot edu_robot_control into a docker image"

clean:
	rm -rf edu_robot
	rm -rf edu_robot_control
	rm -rf joystick_drivers
