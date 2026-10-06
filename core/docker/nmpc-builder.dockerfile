# syntax=docker/dockerfile:1
# NMPC bundle builder — produces libnmpc_bundle_<arch>.a (aarch64 | x86_64).
#
# All toolchain versions are pinned inside this image; the acados commit MUST
# match tooling/acados_setup.py:ACADOS_COMMIT (single source of truth).
# Build per target arch (the compiled layers are arch-specific):
#   docker build --platform linux/arm64  -t ap-pnc-nmpc-builder:<rev>-aarch64 .
#   docker build --platform linux/amd64  -t ap-pnc-nmpc-builder:<rev>-x86_64 .
# Normally driven by: uv run ap-pnc gen-nmpc-lib --arch aarch64|x86_64
FROM ros:humble-ros-base

ARG ACADOS_COMMIT=2b28dc320a7d17d9f9b6ffefb59ee022d63bab83
ARG T_RENDERER_VERSION=v0.2.0

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git curl python3-pip \
    && rm -rf /var/lib/apt/lists/*

# acados at the pinned commit (codegen interface + C sources + submodules).
RUN git clone https://github.com/acados/acados.git /opt/acados_src \
 && git -C /opt/acados_src checkout ${ACADOS_COMMIT} \
 && git -C /opt/acados_src submodule update --init --recursive

# Shared build staged at /opt/acados: headers + libacados.so + codegen metadata.
# Mirrors the host layout (.artifacts/acados) so the wrapper CMake is unchanged.
RUN cmake -S /opt/acados_src -B /tmp/acados_shared -DCMAKE_INSTALL_PREFIX=/opt/acados \
 && cmake --build /tmp/acados_shared -j"$(nproc)" \
 && cmake --install /tmp/acados_shared \
 && cp /opt/acados_src/lib/link_libs.json /opt/acados_src/lib/git_commit_hash /opt/acados/lib/ \
 && rm -rf /tmp/acados_shared

# Static build at /opt/acados_static (bundle payload: acados/hpipm/blasfeo .a).
RUN cmake -S /opt/acados_src -B /opt/acados_static -DBUILD_SHARED_LIBS=OFF \
 && cmake --build /opt/acados_static -j"$(nproc)"

# t_renderer prebuilt for THIS container arch (v0.2.0 ships amd64 + arm64).
RUN case "$(uname -m)" in \
      x86_64) asset=amd64 ;; \
      aarch64) asset=arm64 ;; \
      *) echo "unsupported arch: $(uname -m)" >&2; exit 2 ;; \
    esac \
 && mkdir -p /opt/acados/bin \
 && curl -fL "https://github.com/acados/tera_renderer/releases/download/${T_RENDERER_VERSION}/t_renderer-${T_RENDERER_VERSION}-linux-${asset}" \
      -o /opt/acados/bin/t_renderer \
 && chmod +x /opt/acados/bin/t_renderer

# Python codegen deps; acados_template version == the acados commit above.
# typer: the entrypoint reuses tooling.commands.gen_nmpc_lib bundle logic.
RUN python3 -m pip install --no-cache-dir casadi numpy pyyaml typer \
 && python3 -m pip install --no-cache-dir /opt/acados_src/interfaces/acados_template

ENV ACADOS_SOURCE_DIR=/opt/acados \
    ACADOS_PYTHON_INTERFACE_PATH=/opt/acados_src/interfaces/acados_template

COPY core/docker/nmpc-builder-entrypoint.sh /usr/local/bin/nmpc-builder-entrypoint
RUN chmod +x /usr/local/bin/nmpc-builder-entrypoint

# Expects the repo mounted at ${AP_PNC_DIR:-/ws}; writes
# .artifacts/nmpc_solver/libnmpc_bundle_<arch>.a (and the unsuffixed alias
# when NMPC_BUNDLE_ALIAS=1, i.e. when the target arch matches the host).
ENTRYPOINT ["/usr/local/bin/nmpc-builder-entrypoint"]
