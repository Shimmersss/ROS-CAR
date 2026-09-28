#!/bin/zsh
set -euo pipefail

if [[ $# -lt 1 || $# -gt 4 ]]; then
  echo "用法: $0 FILE.oni [max_frames] [ppm_dir] [stride]" >&2
  exit 2
fi
SDK="${OPENNI2_SDK:-$HOME/Library/Orbbec/OpenNI-MacOSX-x64-2.3}"
build_dir="${TMPDIR:-/tmp}/roscar-oni-analyzer"
mkdir -p "$build_dir"
clang++ -arch x86_64 -std=c++17 -O2 -I"$SDK/Include" \
  "$PWD/scripts/analyze_oni.cpp" -L"$SDK/Redist" -lOpenNI2 \
  -o "$build_dir/analyze_oni"
install_name_tool -change libOpenNI2.dylib "$SDK/Redist/libOpenNI2.dylib" \
  "$build_dir/analyze_oni"
DYLD_INSERT_LIBRARIES="$HOME/Library/Orbbec/openni-compat/bus_compat.dylib" \
  arch -x86_64 "$build_dir/analyze_oni" "$@"
