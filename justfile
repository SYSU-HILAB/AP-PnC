# PNC Tailsitter Justfile

build:
    docker build -f docker/ap-pnc.dockerfile \
    --network=host --progress=plain \
    -t ap-pnc:latest .

run:
    docker run -it --rm \
        --privileged --net=host \
        -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        --name ap-pnc \
        ap-pnc bash


