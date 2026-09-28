#include <OpenNI.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "usage: analyze_oni FILE.oni\n"; return 2; }
  if (openni::OpenNI::initialize() != openni::STATUS_OK) return 3;
  openni::Device device;
  if (device.open(argv[1]) != openni::STATUS_OK) {
    std::cerr << openni::OpenNI::getExtendedError() << "\n";
    openni::OpenNI::shutdown(); return 4;
  }
  openni::VideoStream depth, color;
  if (depth.create(device, openni::SENSOR_DEPTH) != openni::STATUS_OK ||
      color.create(device, openni::SENSOR_COLOR) != openni::STATUS_OK) {
    device.close(); openni::OpenNI::shutdown(); return 5;
  }
  depth.start(); color.start();
  openni::VideoStream* streams[2] = {&depth, &color};
  std::uint64_t frames = 0, paired = 0, depth_valid = 0, depth_total = 0;
  std::uint64_t first_ts = 0, last_ts = 0;
  for (;;) {
    int changed = -1;
    if (openni::OpenNI::waitForAnyStream(streams, 2, &changed, 1000) != openni::STATUS_OK) break;
    openni::VideoFrameRef df, cf;
    if (depth.readFrame(&df) != openni::STATUS_OK || color.readFrame(&cf) != openni::STATUS_OK) break;
    if (!df.isValid() || !cf.isValid()) continue;
    ++frames;
    if (!first_ts) first_ts = df.getTimestamp();
    last_ts = df.getTimestamp();
    auto* p = static_cast<const std::uint16_t*>(df.getData());
    std::size_t n = static_cast<std::size_t>(df.getWidth()) * df.getHeight();
    depth_total += n;
    for (std::size_t i = 0; i < n; ++i) if (p[i] >= 200 && p[i] <= 8000) ++depth_valid;
    if (df.getTimestamp() == cf.getTimestamp()) ++paired;
  }
  std::printf("frames=%llu paired_timestamps=%llu depth_valid_fraction=%.6f duration_s=%.3f depth=%dx%d color=%dx%d\n",
      (unsigned long long)frames, (unsigned long long)paired,
      depth_total ? double(depth_valid)/double(depth_total) : 0.0,
      first_ts && last_ts ? double(last_ts-first_ts)/1e6 : 0.0,
      depth.getVideoMode().getResolutionX(), depth.getVideoMode().getResolutionY(),
      color.getVideoMode().getResolutionX(), color.getVideoMode().getResolutionY());
  depth.stop(); color.stop(); depth.destroy(); color.destroy(); device.close(); openni::OpenNI::shutdown();
}
