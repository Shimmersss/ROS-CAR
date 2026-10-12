#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK="${ORBBEC_SDK_ROOT:-$HOME/.local/share/roscar/orbbec-sdk-v1.10.16}/SDK"
BUILD="$(mktemp -d "${TMPDIR:-/tmp}/roscar-gemini.XXXXXX")"
trap 'rm -rf "$BUILD"' EXIT
clang++ -std=c++17 -O2 -I"$SDK/include" "$ROOT/scripts/probe_gemini.cpp" \
  -L"$SDK/lib" -lOrbbecSDK -o "$BUILD/probe"
export DYLD_LIBRARY_PATH="$SDK/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
exec_capture="$BUILD/probe"
OUT="${1:-$ROOT/artifacts/gemini-probe}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
# SDK writes its own Log directory relative to cwd; retain it with the evidence.
cd "$OUT"
"$exec_capture" "$OUT"
