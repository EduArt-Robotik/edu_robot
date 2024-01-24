image_name = eduart-robot:0.4.0

download-sources:
	git clone --branch 0.4.0 https://github.com/EduArt-Robotik/edu_robot.git

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

all: download-sources build-docker
	$(info Creating EduArt's robot library Docker image "$(image_name)".)

clean:
	rm -rf edu_robot
