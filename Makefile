GAZEBO_COMPOSE := simulations/gazebo/docker/docker-compose.yml
SIMPLE_COMPOSE := simulations/simple_sim/docker/docker-compose.yml

.PHONY: help sim-gazebo-up sim-gazebo-down sim-gazebo-build \
        sim-simple-up sim-simple-down sim-simple-build sim-simple-pcd

help:
	@echo "AP-PnC simulation entry points"
	@echo "  make sim-gazebo-up        Start Gazebo layer (PX4-VTOL SITL + QGC + ap-pnc)"
	@echo "  make sim-gazebo-down      Stop Gazebo layer"
	@echo "  make sim-gazebo-build     Rebuild Gazebo layer images"
	@echo "  make sim-simple-up        Start simple_sim layer (lightweight LiDAR sim)"
	@echo "  make sim-simple-down      Stop simple_sim layer"
	@echo "  make sim-simple-build     Rebuild simple_sim image"
	@echo "  make sim-simple-pcd       Dump one LiDAR frame as PCD and view it (open3d via uv)"

sim-gazebo-up:
	docker compose -f $(GAZEBO_COMPOSE) up

sim-gazebo-down:
	docker compose -f $(GAZEBO_COMPOSE) down

sim-gazebo-build:
	docker compose -f $(GAZEBO_COMPOSE) build

sim-simple-up:
	docker compose -f $(SIMPLE_COMPOSE) up

sim-simple-down:
	docker compose -f $(SIMPLE_COMPOSE) down

sim-simple-build:
	docker compose -f $(SIMPLE_COMPOSE) build

# Dump one LiDAR frame from the running vehicle into .artifacts/simple-sim/
# and open it on the host with open3d (uv).
sim-simple-pcd:
	docker compose -f $(SIMPLE_COMPOSE) up -d simple-sim
	docker exec simple-sim bash -lc '\
		source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
		ros2 run simple_sim simple_sim_node --ros-args \
		-p dump_pcd_path:=/workspace/.artifacts/frame.pcd & \
		NODE_PID=$$!; sleep 5; kill $$NODE_PID 2>/dev/null; wait 2>/dev/null; true'
	uv run --script ap_pnc/tools/viz_pcd.py .artifacts/simple-sim/frame.pcd
