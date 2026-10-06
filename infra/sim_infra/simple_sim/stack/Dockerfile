# syntax=docker/dockerfile:1
ARG TARGETARCH
FROM rust:1.96.0-slim-bookworm@sha256:4732ca96fd086cb9be682050c3f0176288eebaac2b80aa2bcefccfaf198e1950 AS rust
FROM simple-sim:${TARGETARCH} AS toolchain
ARG TARGETARCH
COPY --from=rust /usr/local/rustup /usr/local/rustup
COPY --from=rust /usr/local/cargo /usr/local/cargo
ENV RUSTUP_HOME=/usr/local/rustup PATH=/usr/local/cargo/bin:$PATH
RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates pkg-config libssl-dev protobuf-compiler && rm -rf /var/lib/apt/lists/*
RUN set -eux; \
    case "$TARGETARCH" in \
      arm64) triple=aarch64-unknown-linux-gnu; dora_sha=46813c49fc65a6b1e1ad2bcfa5ccbd51a52f949b290162b8e2f9fd595eb29010; rerun_sha=699e1ff34dbcbe3b70176a009376b37f69867d111879ed216569a8d9f33ae235 ;; \
      amd64) triple=x86_64-unknown-linux-gnu; dora_sha=1f9c972d379391356a4ec717d7c22b4a0c1081fbb330401463a6989cee34c0f6; rerun_sha=060acc4fb061927ec4cc8c9f30d063941682a75cda5dcff29d0b4532700829cf ;; \
      *) exit 2 ;; \
    esac; \
    dir="$AP_PNC_DIR/.artifacts/simple-stack/download"; mkdir -p "$dir"; \
    curl -fL --retry 3 "https://github.com/dora-rs/dora/releases/download/v1.0.1/dora-cli-${triple}.tar.gz" -o "$dir/dora.tar.gz"; \
    echo "$dora_sha  $dir/dora.tar.gz" | sha256sum -c -; \
    tar -xzf "$dir/dora.tar.gz" -C "$dir"; \
    find "$dir" -type f -name dora -exec install -m755 '{}' /usr/local/bin/dora \;; \
    curl -fL --retry 3 "https://github.com/rerun-io/rerun/releases/download/0.38.1/rerun-cli-0.38.1-${triple}" -o "$dir/rerun"; \
    echo "$rerun_sha  $dir/rerun" | sha256sum -c -; \
    install -m755 "$dir/rerun" /usr/local/bin/rerun; rm -rf "$dir"; \
    dora --version; rerun --version; rustc --version
RUN rustup component add rustfmt clippy
ENTRYPOINT ["/entrypoint.sh"]

FROM simple-sim:${TARGETARCH} AS runtime
COPY --from=toolchain /usr/local/bin/dora /usr/local/bin/rerun /usr/local/bin/
# Binary is mounted read-only from the arch/OS-isolated host artifact tree.
ENV RERUN_ANALYTICS_ENABLED=false
ENTRYPOINT ["/entrypoint.sh"]
CMD ["dora", "--version"]
