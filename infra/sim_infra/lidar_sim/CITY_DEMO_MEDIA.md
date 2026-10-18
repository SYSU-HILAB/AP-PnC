# 3DGS demonstration media: provenance and license

The screenshots and MP4 in the `demo-city-ring-20261018` media release show
**actual online NMPC simulation**, rendered live in the browser. They are not
real-aircraft footage, recorded-trajectory playback, or an original surveyed city.

## Attribution

- **MatrixCity** — Yixuan Li, Lihan Jiang, Linning Xu, Yuanbo Xiangli, Zhenzhi Wang,
  Dahua Lin and Bo Dai, *MatrixCity: A Large-scale City Dataset for City-scale
  Neural Rendering and Beyond*, ICCV 2023.
  [Source project](https://github.com/city-super/MatrixCity).
  MatrixCity builds on Unreal Engine's City Sample; upstream rights remain with
  their respective owners.
- **CityGaussian authors / TeslaYang123** — *CityGaussian: Real-time High-quality
  Large-Scale Scene Rendering with Gaussians*, ECCV 2024.
  [Official checkpoint/model card](https://huggingface.co/TeslaYang123/CityGaussian/tree/4f73714cf50bdefe47ff8012e5c7ec71246adb9a).
  Source archive: `MatrixCityAeiral_2.zip`; member
  `mc_aerial_c36_light_75_vq/point_cloud.ply`.
- **AP-PnC contributors** — scene preparation, synthetic quadrant mirroring,
  visual scaling/cropping, occupancy extraction, controller/mission, aircraft
  overlay, live dual-camera rendering, screen capture and editing.

The CityGaussian model card declares
[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
The derived demonstration media are shared under that same noncommercial,
attribution/share-alike license, subject to upstream rights. Changes are listed
above; no endorsement by the original authors is implied. This media notice does
**not** change the AP-PnC source-code license or relicense upstream assets.

Only rendered screenshots and video are published. Gaussian checkpoints, private
actuator profiles, calibration/provenance files and raw flight CSVs stay local.

## What was recorded and edited

- Live capture at 1600×900, H.264 MP4, no audio.
- The first **82 seconds** preserve continuous browser capture, including the
  full 77.36 s simulated mission, without time scaling or cuts during flight.
- The final **4 seconds** are a still of the same run after independent CSV/map/
  return verification. Waiting after completion was removed; the still is not
  claimed to be additional flight.
- Overview switches to chase view during cruise. Both onboard cameras always
  use actual body attitude and assumed rigid extrinsics, with no horizon lock.
- Encoded output rate is 20 FPS, not a measured renderer performance guarantee;
  capture can repeat frames. Simulation time and measured speed remain visible.
- The screenshots show cruise overview, chase/dual-camera cruise, and completed
  acceptance. The orange trail is actual motion; blue is the committed reference.

The captured run passed independent verification: actual cruise speed
9.583–10.567 m/s (mean 10.007), position RMSE 0.133 m, maximum error 0.565 m,
15 online planning legs, and zero nonzero solver statuses. See
[CITY_DEMO.md](CITY_DEMO.md) for acceptance scope. Static Gaussian-derived
occupancy is not contact physics, a solid collision mesh or dynamic avoidance.

Generated media remain locally under `.artifacts/dev-sim/city-ring/media/` and
are distributed as release attachments, not committed binaries. The media release
is explicitly not a software version and does not replace the latest software
release. [Startup instructions](CITY_DEMO_START.md) explain the required local
assets and practical profile.
