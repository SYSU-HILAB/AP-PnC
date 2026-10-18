# AP-PnC

Aerodynamic prior-free trajectory generation and tracking control for a tail-sitter UAV.

## Live 3DGS flight demo

Online planning and NMPC at 10 m/s in a synthetic mirrored MatrixCity scene,
with two body-fixed cameras—actual closed-loop simulation, not trajectory playback.

[![Live NMPC flight with nose and belly cameras](https://github.com/SYSU-HILAB/AP-PnC/releases/download/demo-city-ring-20261018/city-ring-cameras.png)](https://github.com/SYSU-HILAB/AP-PnC/releases/download/demo-city-ring-20261018/city-ring-demo.mp4)

[Watch MP4 · 86 s](https://github.com/SYSU-HILAB/AP-PnC/releases/download/demo-city-ring-20261018/city-ring-demo.mp4)
· [Run the demo](infra/sim_infra/lidar_sim/CITY_DEMO_START.md)
· [Media credits](infra/sim_infra/lidar_sim/CITY_DEMO_MEDIA.md)

## Documentation

- [Quick start](infra/sim_infra/QUICK_START.md)
- [Simulation guide](infra/sim_infra/simple_sim/README.md)
- [ROS2 development scene](infra/sim_infra/lidar_sim/README.md)
- [Planner and NMPC packages](core/ros_packages/README.md)
- [Linux development containers](core/docker/README.md)

## Paper

**Aerodynamic Prior-Free Coordinated Trajectory Generation and Tracking Control
for a Tail-Sitter UAV**

[arXiv:2609.11698](https://arxiv.org/abs/2609.11698) ·
[IEEE/ASME Transactions on Mechatronics, Early Access](https://ieeexplore.ieee.org/document/11719954)

<details>
<summary>Cite this work (BibTeX)</summary>

```bibtex
@article{rong2026apPnc,
  title  = {Aerodynamic Prior-Free Coordinated Trajectory Generation and
            Tracking Control for a Tail-Sitter UAV},
  author = {Rong, Erchao and Liu, Zihao and Liang, Junning and Wang, Jianguo and
            Jie, Xiao and Fu, Haoran and Chen, Ziliang and Lyu, Ximin},
  journal = {arXiv preprint arXiv:2609.11698},
  note = {IEEE/ASME Transactions on Mechatronics, Early Access},
  year   = {2026},
  url    = {https://arxiv.org/abs/2609.11698}
}
```

</details>
