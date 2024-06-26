<!--
Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.

Downloading, reproducing, distributing or otherwise using the SDK Software
is subject to the terms and conditions of the Boston Dynamics Software
Development Kit License (20191101-BDSDK-SL).
-->

# Fire Extinguisher Detector Server

This is an example of a network compute bridge server. It is similar to tensorflow_server.py, but uses Keras Retinanet instead of TensorFlow. The provided model specifically detects fire extinguishers

## Build and Export

This example can be run on a local machine directly, but is easier to use with Docker.

The Docker image can be built and exported with the following commands:

```
# builds the image
sudo docker build -t fire_ext_detector .

# exports the image, uses pigz
sudo docker save fire_ext_detector | pigz > fire_ext_detector.tar.gz
```

To build the image for CORE I/O:

```
# Prerequisites
# Install the pigz and qemu packages
sudo apt-get install qemu binfmt-support qemu-user-static pigz
# This step will execute the registering scripts
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Builds the image
sudo docker build -t fire_ext_detector:l4t -f Dockerfile.l4t .

# Exports the image, uses pigz
sudo docker save fire_ext_detector:l4t | pigz > fire_ext_detector.tar.gz
```

## Execution

To run this example on a Spot CORE, run:

```
./start_server.sh
```

Otherwise, run:

```
sudo docker run -d --name retinanet_server --network host --env BOSDYN_CLIENT_USERNAME --env BOSDYN_CLIENT_PASSWORD --restart unless-stopped fire_ext_detector -d . --port $PORT $ROBOT_IP
```

- `$PORT` is the port to use for the server on the machine the server is running on
- `$ROBOT_IP` is the IP address or hostname of your Spot.
- `BOSDYN_CLIENT_USERNAME` is the environment variable for the username for the robot
- `BOSDYN_CLIENT_PASSWORD` is the environment variable for the password for the robot

To run this example on a CORE I/O, run:

```
# Load the image
sudo docker load -i fire_ext_detector.tar.gz

# Run the container
sudo docker run -d \
    --gpus all \
    --name retinanet_server \
    --network host \
    --restart unless-stopped \
    -v /opt/payload_credentials/payload_guid_and_secret:/opt/payload_credentials/payload_guid_and_secret \
    fire_ext_detector:l4t \
    -d . 192.168.50.3 --payload-credentials-file /opt/payload_credentials/payload_guid_and_secret
```

## Spot Extension for CORE I/O

This example can also be built into a [Spot Extension](../../../../docs/payload/docker_containers.md) using a provided [convenience script](../../extensions/README.md)

```sh
cd {/path/to/python/examples/network_compute_bridge/fire_extinguisher_server/}

python3 ../../extensions/build_extension.py \
    --arm \
    --dockerfile-paths Dockerfile.l4t \
    --build-image-tags fire_ext_detector:l4t \
    -i fire_ext_detector_image.tar.gz \
    --package-dir . \
    --spx fire_extinguisher_detector.spx
```

The output file will be called fire_ext_detector.spx and can be uploaded to a CORE I/O.
