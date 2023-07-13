image_name = eduard-ipc127e:0.3.0

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

all: download-sources build-docker
	$(info Creating Eduard IoT2050 Docker image "$(image_name)".)
