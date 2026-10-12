#!/usr/bin/env bash
# Ubuntu Jammy libuvc-dev omits its .pc file. Describe installed libraries for upstream CMake.
set -euo pipefail
OUT="${1:?需提供隔离 pkgconfig 目录}"
mkdir -p "$OUT"
for item in 'libuvc libuvc-dev uvc' 'libglog libgoogle-glog-dev glog'; do
  read -r module package library <<< "$item"
  if pkg-config --exists "$module"; then continue; fi
  version="$(dpkg-query -W -f='${Version}' "$package")"
  cat > "$OUT/$module.pc" <<PC
prefix=/usr
Name: $module
Description: System $package; pkg-config metadata for the installed Ubuntu library
Version: $version
Libs: -l$library
Cflags:
PC
done
