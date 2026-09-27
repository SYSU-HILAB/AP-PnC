---
name: ap-pnc-bootstrap
description: Bootstrap AP-PnC project from scratch on a Linux host — sync Python environment, build acados C libraries (shared + static), install acados_template, generate OCP solver code, and optionally produce a self-contained libnmpc_bundle.a for ROS2 linking. Use when setting up this project, after a fresh clone, or when regenerating OCP code.
---

# AP-PnC Bootstrap

## Prerequisites

```bash
uv --version    # https://docs.astral.sh/uv/
git --version
cmake --version
curl --version
gcc --version
make --version
```

If any missing → ask user to install.

## Install

### 1. uv sync Python environment

```bash
uv sync --group dev --group extra
```

The `extra` group installs ML/compute dependencies (JAX, Torch, CasADi, matplotlib, optax, flax, seaborn) needed for aerodynamics commands. Omit `--group extra` if you only need the C++/ROS2 pipeline.

### 2. Initialize acados submodule

```bash
git submodule update --init --recursive (inside .artifacts/acados_src)
```

### 3. Build acados C libraries (shared + static)

```bash
cmake -S .artifacts/acados_src -B .artifacts/acados/build
make -j$(nproc) -C .artifacts/acados/build
make install -C .artifacts/acados/build

cmake -S .artifacts/acados_src -B .artifacts/acados/build_static -DBUILD_SHARED_LIBS=OFF
make -j$(nproc) -C .artifacts/acados_static
```

Output:

| File                    | Type   | Purpose                |
| ----------------------- | ------ | ---------------------- |
| `.artifacts/acados/lib/*.so`  | Shared | Python codegen + local dev |
| `.artifacts/acados_static/**/*.a` | Static | ROS2 bundle (step 7)   |

### 4. Download t_renderer

```bash
mkdir -p .artifacts/acados/bin
curl -L https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64 \
  -o .artifacts/acados/bin/t_renderer
chmod +x .artifacts/acados/bin/t_renderer
```

### 5. Install acados_template

```bash
uv pip install -e .artifacts/acados_src/interfaces/acados_template
```

### 6. Generate OCP solver code + bundle

```bash
uv run ap-pnc gen-nmpc-lib
```

Output:

| Artifact                            | Location                         |
| ----------------------------------- | -------------------------------- |
| Generated `.c` / `.h` / `Makefile`   | `c_generated_code/`              |
| `libacados_ocp_solver_tailsitter_flu.so` | `c_generated_code/` (shared)     |
| `libnmpc_bundle.a`                  | `.artifacts/nmpc_solver/` (static)   |

The bundle is a single static archive containing the wrapper + generated solver + acados/hpipm/blasfeo.

## Verify

```bash
uv run ap-pnc --help

# Python interface works
uv run python -c "from acados_template import AcadosOcpSolver; print('OK')"

# Bundle exists
ls -lh .artifacts/nmpc_solver/libnmpc_bundle.a
```

## ML / Aerodynamics Usage

Install the extra dependencies separately if skipped earlier:

```bash
uv sync --group extra
```

### Aerodynamic data generation

```bash
uv run ap-pnc aerodyn gen               # coordinated aero data (JAX, default)
uv run ap-pnc aerodyn gen --engine torch
uv run ap-pnc aerodyn gen --type mesh --size 181
```

### Visualize aerodynamic coefficients

```bash
uv run ap-pnc aerodyn viz               # plot all coefficients
uv run ap-pnc aerodyn viz --type mesh   # 3D mesh plot
uv run ap-pnc aerodyn viz --type single # single run plot
```

### ML pipeline (train neural network models)

```bash
uv run ap-pnc aerodyn ml train lyu          # train single model
uv run ap-pnc aerodyn ml pipeline            # assemble dataset + train all types
```

### Compare aero derivatives across ML models

```bash
uv run ap-pnc aerodyn show                  # all aero types
uv run ap-pnc aerodyn show --aero-type lyu  # specific type
```

## Docker Build

```bash
uv run ap-pnc gen-nmpc-lib
docker compose -f infra/sim_infra/gazebo/docker/docker-compose.yml build ap-pnc
```

The resulting `nmpc_node` binary is fully self-contained — zero acados runtime deps.

See the ap-pnc-docker skill for run, hot-update, and troubleshooting.
