# PNC Tailsitter Justfile

build:
    docker build -f docker/ap-pnc.dockerfile \
    --network=host \
    -t ap-pnc:latest .

run:
    docker run -it --rm \
        --privileged --net=host \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $(pwd)/src/tailsitter_df:/ap-pnc/src/tailsitter_df \
        -v $(pwd)/scripts/tools:/ap-pnc/scripts/tools \
        --name ap-pnc \
        ap-pnc bash

attach:
    docker exec -it ap-pnc bash

build-px4:
    docker build -f docker/px4-simulator.dockerfile \
    --network=host \
    -t px4-simulator:latest .

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
    docker run -it --rm \
        --privileged --net=host \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --name qground_control \
        qground_control qgc

# --build-arg CACHE_BUSTER=$(date +%s) \
