from setuptools import setup

setup(
    name="lidar_sim", version="0.1.0", packages=["lidar_sim"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/lidar_sim"]),
                ("share/lidar_sim", ["package.xml"])],
    install_requires=["setuptools", "numpy", "PyYAML"], zip_safe=True,
    maintainer="AP-PnC maintainers", maintainer_email="rongerch@outlook.com",
    description="ROS2 point-cloud scene and first-return LiDAR development simulator",
    license="BSD-3-Clause",
    entry_points={"console_scripts": ["lidar_sim_node = lidar_sim.node:main"]},
)
