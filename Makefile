GAZEBO_COMPOSE := simulations/gazebo/docker/docker-compose.yml
SIMPLE_COMPOSE := simulations/simple_sim/docker/docker-compose.yml

.PHONY: help sim-gazebo-up sim-gazebo-down sim-gazebo-build \
        sim-simple-up sim-simple-down sim-simple-build

help:
	@echo "AP-PnC simulation entry points"
	@echo "  make sim-gazebo-up        Start Gazebo layer (PX4-VTOL SITL + QGC + ap-pnc)"
	@echo "  make sim-gazebo-down      Stop Gazebo layer"
	@echo "  make sim-gazebo-build     Rebuild Gazebo layer images"
	@echo "  make sim-simple-up        Start simple_sim layer (lightweight LiDAR sim)"
	@echo "  make sim-simple-down      Stop simple_sim layer"
	@echo "  make sim-simple-build     Rebuild simple_sim image"

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
