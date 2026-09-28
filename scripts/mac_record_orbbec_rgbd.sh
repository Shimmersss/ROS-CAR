#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${ORBBEC_SDK_ROOT:-$HOME/.local/share/roscar/orbbec-sdk-v1.10.16}"
OUT="${1:-$ROOT/data/rgbd-recordings/astra-s-$(date +%Y%m%d-%H%M%S)}"
SECONDS_TO_RECORD="${2:-60}"
LIB="$SDK_ROOT/SDK/lib"
if [[ ! -f "$LIB/libOrbbecSDK.1.10.16.dylib" ]]; then
  echo "找不到 Orbbec SDK v1.10.x: $SDK_ROOT" >&2
  echo "请设置 ORBBEC_SDK_ROOT 指向解压后的 macOS SDK 目录。" >&2
  exit 2
fi
BUILD_DIR="${TMPDIR:-/tmp}/roscar-orbbec-rgbd-build"
mkdir -p "$BUILD_DIR"
clang++ -std=c++17 -O2 \
  -I"$SDK_ROOT/SDK/include" \
  "$ROOT/scripts/mac_record_orbbec_rgbd.cpp" \
  -L"$LIB" -lOrbbecSDK -Wl,-rpath,"$LIB" \
  -o "$BUILD_DIR/mac_record_orbbec_rgbd"
export DYLD_LIBRARY_PATH="$LIB${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
exec "$BUILD_DIR/mac_record_orbbec_rgbd" "$OUT" "$SECONDS_TO_RECORD"
