# syntax=docker/dockerfile:1
ARG TARGETARCH
FROM rust:1.96.0-slim-bookworm@sha256:4732ca96fd086cb9be682050c3f0176288eebaac2b80aa2bcefccfaf198e1950 AS rust
FROM simple-sim:${TARGETARCH} AS toolchain
RUN rm -rf "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack"
ARG TARGETARCH
COPY --from=rust /usr/local/rustup /usr/local/rustup
COPY --from=rust /usr/local/cargo /usr/local/cargo
ENV RUSTUP_HOME=/usr/local/rustup PATH=/usr/local/cargo/bin:$PATH
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates pkg-config libssl-dev protobuf-compiler && rm -rf /var/lib/apt/lists/*
RUN set -eux; \
    case "$TARGETARCH" in \
      arm64) triple=aarch64-unknown-linux-gnu; sha=699e1ff34dbcbe3b70176a009376b37f69867d111879ed216569a8d9f33ae235 ;; \
      amd64) triple=x86_64-unknown-linux-gnu; sha=060acc4fb061927ec4cc8c9f30d063941682a75cda5dcff29d0b4532700829cf ;; \
      *) exit 2 ;; \
    esac; \
    curl -fL --retry 3 "https://github.com/rerun-io/rerun/releases/download/0.38.1/rerun-cli-0.38.1-${triple}" -o /tmp/rerun; \
    echo "$sha  /tmp/rerun" | sha256sum -c -; \
    install -m755 /tmp/rerun /usr/local/bin/rerun; rm /tmp/rerun; \
    rerun --version; rustc --version
RUN rustup component add rustfmt clippy
ENTRYPOINT ["/entrypoint.sh"]

FROM simple-sim:${TARGETARCH} AS runtime
RUN rm -rf "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack"
COPY --from=toolchain /usr/local/bin/rerun /usr/local/bin/
# Binary is mounted read-only from the arch/OS-isolated host artifact tree.
ENV RERUN_ANALYTICS_ENABLED=false
ENTRYPOINT ["/entrypoint.sh"]
CMD ["rerun", "--version"]
