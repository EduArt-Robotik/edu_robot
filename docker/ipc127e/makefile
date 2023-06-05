current_dir = $(shell pwd)

download-sources:
	git clone --branch 0.2.1 https://github.com/EduArt-Robotik/edu_robot.git

build-docker:
	docker build -t eduard-ipc127e:0.2.1 .

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"

clean:
	rm -rf edu_robot
