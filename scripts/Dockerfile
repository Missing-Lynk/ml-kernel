# syntax=docker/dockerfile:1
# Hermetic build environment for the modern-kernel build.
# Pinned to a digest (not just the tag) so a fresh checkout resolves the exact same
# base layer: the tag is mutable (Debian republishes it on point releases). To bump,
# re-pull and take the new digest from `docker inspect debian:trixie-slim`.
FROM debian:trixie-slim@sha256:28de0877c2189802884ccd20f15ee41c203573bd87bb6b883f5f46362d24c5c2

# One RUN = one image layer, written as a heredoc so each step is a plain line (no `&&`
# chaining). update + install + lists-cleanup MUST stay in this one layer: separate layers
# would serve a stale apt index to the install and leave the lists baked into the image.
# `set -e` aborts the build on any failed step.
RUN <<EOF
set -e
apt-get update
apt-get install -y --no-install-recommends \
    make gcc g++ bc bison flex libssl-dev libelf-dev \
    xz-utils cpio kmod rsync lz4 ca-certificates patch
rm -rf /var/lib/apt/lists/*
EOF
