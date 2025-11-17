# PNC Tailsitter Justfile
# build top algorithms image
build-algs:
    docker build -f docker/ap-pnc.dockerfile \
    --network=host \
    -t ap-pnc:latest .

run-algs:
    docker run -it --rm \
        --privileged --net=host \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd)/src/tailsitter_df:/ap-pnc/src/tailsitter_df \
        -v $(pwd)/src/trajectory_server:/ap-pnc/src/trajectory_server \
        -v $(pwd)/scripts/tools:/ap-pnc/scripts/tools \
        -v $(pwd)/temp_data:/ap-pnc/temp_data \
        --name ap-pnc \
        ap-pnc bash

attach:
    docker exec -it ap-pnc bash
a:
    docker exec -it px4-simulator bash

build-px4:
    docker build -f docker/px4-simulator.dockerfile \
    --network=host \
    -t px4-simulator:latest .
    # --build-arg CACHE_BUSTER=$(date +%s) \

px4:
    docker run -it --rm \
        --privileged --net=host \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --name px4-simulator \
        px4-simulator bash

build-qgc:
    docker build -f docker/qgc.dockerfile \
    --network=host \
    -t qground_control:latest .

qgc:
    docker run --rm \
        --privileged --net=host \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
        -v $HOME/.Xauthority:/qgc/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --name qground_control \
        qground_control qgc

# --build-arg CACHE_BUSTER=$(date +%s) \
up:
    docker compose up -d

down:
    docker compose down
