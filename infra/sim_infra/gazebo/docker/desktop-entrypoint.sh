#!/usr/bin/env bash
set -euo pipefail
case "${AP_PNC_DIR:?Set absolute AP_PNC_DIR}" in
    /*) ;;
    *) echo 'AP_PNC_DIR must be absolute' >&2; exit 2 ;;
esac
export HOME="$AP_PNC_DIR/.artifacts/desktop/home"
logs="$AP_PNC_DIR/.artifacts/desktop/log"
mkdir -p "$HOME" "$logs" /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix
# UNIX socket is shared only with the GUI clients; no TCP X11 listener.
Xvfb :99 -screen 0 1280x800x24 -nolisten tcp -ac > "$logs/xvfb.log" 2>&1 &
pids=($!)
cleanup() { kill "${pids[@]}" 2>/dev/null || true; wait || true; }
trap cleanup EXIT
trap 'exit 0' TERM INT
ready=0
for _ in {1..50}; do
    if xdpyinfo -display :99 > /dev/null 2>&1; then ready=1; break; fi
    sleep 0.2
done
[[ "$ready" == 1 ]] || { echo 'Xvfb did not become ready' >&2; exit 1; }
fluxbox > "$logs/fluxbox.log" 2>&1 & pids+=($!)
x11vnc -display :99 -listen 127.0.0.1 -rfbport 5900 -forever -shared -nopw \
    > "$logs/vnc.log" 2>&1 & pids+=($!)
# HTTP is published by Compose on host loopback only. VNC is never published.
websockify --web=/usr/share/novnc 0.0.0.0:6080 127.0.0.1:5900 \
    > "$logs/web.log" 2>&1 & pids+=($!)
wait -n "${pids[@]}"
