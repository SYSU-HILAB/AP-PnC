#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

uv run ap-pnc cocp
docker compose -f simulations/gazebo/docker/docker-compose.yml build ap-pnc
