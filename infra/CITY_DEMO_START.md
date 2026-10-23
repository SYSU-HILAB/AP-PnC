# Start the live 3D Gaussian Splatting demo

This is the **actual online planner + NMPC + practical dynamics simulator**, not
`flight.json` playback. The browser renders the simulated body pose and two
body-fixed cameras. The demonstrated scene is a **synthetic mirrored MatrixCity
road**, with assumed metric scale and visible reconstruction seams.

The accepted target is Linux ARM64 (including Apple Silicon via Docker).
Commands select the host-native CPU; AMD64 is supported by the build recipe but
has **not** passed this new ring's runtime acceptance. Do not use an AMD64 fallback
on Apple Silicon or reuse caches from a different OS/architecture.

## 1. Required local inputs

Install Docker and uv, clone the repository, and set its **absolute** root:

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
uv sync --project "$AP_PNC_DIR"
uv run --project "$AP_PNC_DIR" ap-pnc gen-nmpc-lib
```

This demo is **not asset-free**. Supply these files yourself:

| File under the project root | Requirement |
|---|---|
| `.artifacts/dev-sim/matrixcity/assets/matrixcity-aerial.ply` | The specific Gaussian checkpoint below; not an arbitrary XYZ cloud |
| `.artifacts/vtol-matlab/profile.yaml` | Authorized, locally extracted practical actuator profile |
| `.artifacts/vtol-matlab/provenance.json` | Its extraction provenance |

Gaussian source: [CityGaussian checkpoints](https://huggingface.co/TeslaYang123/CityGaussian/tree/4f73714cf50bdefe47ff8012e5c7ec71246adb9a),
revision `4f73714cf50bdefe47ff8012e5c7ec71246adb9a`.
Obtain `MatrixCityAeiral_2.zip`, extract the member
`mc_aerial_c36_light_75_vq/point_cloud.ply`, and place it at the path above.
The tested input contains **5,847,405 Gaussians**. The archive is several GB;
allow space for extraction, prepared scenes and Linux build artifacts.

Respect [MatrixCity](https://github.com/city-super/MatrixCity) and
[CityGaussian](https://github.com/Linketic/CityGaussian) attribution and terms.
The checkpoint model card declares **CC-BY-NC-SA-4.0**; this does not relicense
upstream assets. No checkpoint or private actuator profile is redistributed here.
See [media attribution](CITY_DEMO_MEDIA.md) for the public demonstration images.

For authorized practical-profile extraction, see the
[stack guide](OFFLINE_STACK.md). Missing inputs are an error: do not
invent calibration or use the public ideal benchmark as practical acceptance.

## 2. Build and prepare the scene in native Linux Docker

These are shell commands, not host ROS/CMake commands. Start with an empty native
build directory on a new machine; never copy `.artifacts/` from another target.
Run setup before launching a mission, not while a recording is in progress.

```bash
case "$(uname -m)" in
  arm64|aarch64) ARCH=arm64 ;;
  x86_64|amd64) ARCH=amd64 ;;
  *) echo 'Unsupported CPU architecture'; exit 1 ;;
esac

docker build --platform "linux/$ARCH" --target tools \
  -f "$AP_PNC_DIR/core/docker/ros2-base.dockerfile" \
  -t "ap-pnc-tools:$ARCH" "$AP_PNC_DIR"

mkdir -p "$AP_PNC_DIR/.artifacts/dev-sim/city-loop" \
  "$AP_PNC_DIR/.artifacts/dev-sim/city-ring"

docker run --rm --platform "linux/$ARCH" --ipc=host --network none \
  -v "$AP_PNC_DIR:/workspace:ro" \
  -v "$AP_PNC_DIR/.artifacts/dev-sim/city-loop:/workspace/.artifacts/dev-sim/city-loop:rw" \
  -v "$AP_PNC_DIR/.artifacts/dev-sim/city-ring:/workspace/.artifacts/dev-sim/city-ring:rw" \
  -e AP_PNC_DIR=/workspace -e PYTHONPATH=/workspace \
  -e PYTHONDONTWRITEBYTECODE=1 --entrypoint /bin/bash "ap-pnc-tools:$ARCH" -c '
    set -e
    source /opt/ros/humble/setup.bash
    python3 "$AP_PNC_DIR/infra/sim_infra/lidar_sim/scripts/prepare_city_ring.py"
    cmake -S "$AP_PNC_DIR/infra/sim_infra/simple_sim" \
      -B "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/build" \
      -DSIMPLE_SIM_WITH_ROS=ON -DSIMPLE_SIM_WITH_NMPC=ON \
      -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
    cmake --build "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/build" \
      --target city_mission_node -j 3
  '
```

The preparation script transforms Gaussian appearance/covariance and occupancy
consistently; it does not overwrite the original checkpoint. Runtime configuration
uses Z=12 m, overriding the asset generator's earlier 10 m route proposal.

## 3. Prepare and launch a paused mission

Keep the following shell variables in the same terminal for later verification:

```bash
JOB=$(PYTHONPATH="$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -m tooling.city_demo)
JOB_ID=${JOB##*/}
printf 'Prepared job: %s\n' "$JOB"
PYTHONPATH="$AP_PNC_DIR" uv run --project "$AP_PNC_DIR" python -m tooling.city_demo \
  --launch "$JOB"
```

`JOB` is the absolute path emitted by the staging tool; `JOB_ID` is its basename.
The container is named `ap-pnc-city-$JOB_ID`. Keep only one selected mission running.
It uses host IPC, no network, ROS domain 213 and localhost-only DDS. Inputs and
executable are frozen and hashed. A changed configuration requires a new job.
The ring's local roll P/D tuning is explicit in its metadata; global NMPC defaults
and the original practical profile remain unchanged.

## 4. Start the browser viewer

Stage the web application; npm and Vite run inside Node's Linux container:

```bash
mkdir -p "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/web"
cp "$AP_PNC_DIR/infra/sim_infra/lidar_sim/web/"* \
  "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/web/"

docker run --rm --platform "linux/$ARCH" --ipc=host -e AP_PNC_DIR=/workspace \
  -v "$AP_PNC_DIR/.artifacts/dev-sim/city-loop:/workspace/.artifacts/dev-sim/city-loop:rw" \
  node:22-alpine sh -c '
    set -e
    mkdir -p "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/web/public"
    if [ ! -e "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/web/public/assets" ]; then
      ln -s "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/assets" \
        "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/web/public/assets"
    fi
    npm --prefix "$AP_PNC_DIR/.artifacts/dev-sim/city-loop/web" install --no-audit --no-fund
  '

docker run -d --name ap-pnc-city-viewer --platform "linux/$ARCH" --ipc=host \
  -p 127.0.0.1:5178:5178 -e AP_PNC_DIR=/workspace \
  -v "$AP_PNC_DIR/.artifacts/dev-sim/city-loop:/workspace/.artifacts/dev-sim/city-loop:rw" \
  -v "$AP_PNC_DIR/.artifacts/dev-sim/city-ring:/workspace/.artifacts/dev-sim/city-ring:ro" \
  node:22-alpine npm --prefix /workspace/.artifacts/dev-sim/city-loop/web \
    run dev -- /workspace/.artifacts/dev-sim/city-loop/web
```

Open **http://127.0.0.1:5178** on the development machine in a WebGL2-capable
browser. The scene may take time to load. Do not expose the viewer/control API to
the Internet. This localhost address is not a remotely accessible phone link.

- **启动试验 / Start:** 8 s initial hover, acceleration, continuous cruise, then
  return deceleration and terminal hover. No intermediate simulated dwell.
- **暂停 / Pause:** freezes simulation time; **继续试验 / Resume** continues it.
- **重置 / Reset:** available while paused/finished; creates a fresh recording.
- **街区俯瞰 / Overview**, **外部跟随 / Chase**, **自由观察 / Orbit:** external
  observer views only. Neither onboard camera has horizon stabilization.
- Blue is the reference; orange is the actual trajectory. Both camera timestamps
  are committed/interpolated plant time, not independently fabricated playback.

## 5. Independently verify each completed recording

```bash
docker exec -e PYTHONPATH=/workspace -e PYTHONDONTWRITEBYTECODE=1 \
  "ap-pnc-city-$JOB_ID" python3 \
  /workspace/infra/sim_infra/lidar_sim/scripts/verify_city_run.py \
  "/workspace/.artifacts/dev-sim/city-loop/jobs/$JOB_ID"
```

Only after this succeeds does the matching completed run show a green acceptance
banner. Resetting invalidates that banner; reference execution alone is not a
pass. CSV, reference coefficients and receipts remain in the job's `recordings/`.
See [acceptance and limitations](CITY_DEMO.md) for measured errors and tests.

To stop your selected demo (do not target unrelated containers):

```bash
docker stop "ap-pnc-city-$JOB_ID" ap-pnc-city-viewer
```

Use `docker start ap-pnc-city-viewer` to reuse the existing viewer. To recreate it
with changed mounts, remove the **stopped** viewer using
`docker rm ap-pnc-city-viewer`, then rerun step 4. Preparing a new job selects its
stream links; stop the old mission before launching the new one.

### Troubleshooting

- **Missing profile/model:** follow step 1; no silent ideal or dummy-data fallback.
- **Waiting for ROS / no live state:** inspect `docker logs "ap-pnc-city-$JOB_ID"` and
  verify the selected job's `stream/live.json` is updating.
- **Blank/unsupported graphics:** enable browser WebGL2/hardware acceleration and
  allow enough GPU memory for the splats and two camera views.
- **Name/port already in use:** reuse or explicitly stop/remove your own demo
  container; do not create a second mission on the same selected stream.
- **Changed solver:** regenerate the matching Linux bundle and rebuild the
  executable; mounting a new static library cannot update a running binary.
