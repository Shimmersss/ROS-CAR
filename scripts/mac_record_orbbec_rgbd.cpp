#include "libobsensor/ObSensor.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static void write_bytes(const fs::path &path, const void *data, std::size_t size) {
    std::ofstream out(path, std::ios::binary);
    out.write(static_cast<const char *>(data), static_cast<std::streamsize>(size));
    if(!out) throw std::runtime_error("写入失败: " + path.string());
}

int main(int argc, char **argv) try {
    if(argc != 3) {
        std::cerr << "用法: mac_record_orbbec_rgbd <输出目录> <秒数>\n";
        return 2;
    }
    const fs::path output = argv[1];
    const double seconds = std::stod(argv[2]);
    if(seconds <= 0.0) throw std::runtime_error("秒数必须大于 0");
    fs::create_directories(output / "color_rgb8");
    fs::create_directories(output / "depth_u16le_mm");

    ob::Pipeline pipeline;
    auto config = std::make_shared<ob::Config>();
    auto colors = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
    auto depths = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    // Start with the USB2-safe common 320x240/30 mode. It avoids the
    // 1280x960 default color profile and is sufficient for motion/depth QA.
    auto color = std::const_pointer_cast<ob::StreamProfile>(colors->getProfile(1))->as<ob::VideoStreamProfile>();
    auto depth = std::const_pointer_cast<ob::StreamProfile>(depths->getProfile(1))->as<ob::VideoStreamProfile>();
    config->enableStream(color);
    config->enableStream(depth);
    // The legacy Astra S reports D2C metadata but rejects the v1 SDK's
    // alignment switch. Capture the native paired streams; registration can
    // be applied offline with the camera calibration.
    config->setAlignMode(ALIGN_DISABLE);
    // Astra S exposes synchronized frames through waitForFrames but does not
    // implement the newer explicit frame-sync control; do not reject it.
    pipeline.start(config);

    std::ofstream csv(output / "timestamps.csv");
    csv << "frame,host_monotonic_ns,color_timestamp_ms,depth_timestamp_ms,width,height\n";
    ob::FormatConvertFilter convert;
    const auto start = std::chrono::steady_clock::now();
    std::size_t count = 0;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
        auto frames = pipeline.waitForFrames(200);
        if(!frames) continue;
        auto color_frame = frames->colorFrame();
        auto depth_frame = frames->depthFrame();
        if(!color_frame || !depth_frame) continue;
        if(color_frame->format() != OB_FORMAT_RGB) {
            if(color_frame->format() == OB_FORMAT_MJPG) convert.setFormatConvertType(FORMAT_MJPG_TO_RGB);
            else if(color_frame->format() == OB_FORMAT_UYVY) convert.setFormatConvertType(FORMAT_UYVY_TO_RGB);
            else if(color_frame->format() == OB_FORMAT_YUYV) convert.setFormatConvertType(FORMAT_YUYV_TO_RGB);
            else continue;
            color_frame = convert.process(color_frame)->as<ob::ColorFrame>();
        }
        ++count;
        const auto stem = (count < 1000000 ? std::string(6 - std::to_string(count).size(), '0') : "") + std::to_string(count);
        write_bytes(output / "color_rgb8" / (stem + ".raw"), color_frame->data(), color_frame->dataSize());
        write_bytes(output / "depth_u16le_mm" / (stem + ".raw"), depth_frame->data(), depth_frame->dataSize());
        csv << count << ','
            << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << ','
            << color_frame->timeStamp() << ',' << depth_frame->timeStamp() << ','
            << color_frame->width() << ',' << color_frame->height() << '\n';
        if(count % 30 == 0) { csv.flush(); std::cout << "已录制 " << count << " 帧\n" << std::flush; }
    }
    pipeline.stop();
    std::ofstream meta(output / "metadata.json");
    meta << "{\n  \"format\": \"roscar-orbbec-v1-rgbd\",\n"
         << "  \"camera\": \"Astra S\",\n  \"sdk\": \"OrbbecSDK v1.10.x\",\n"
         << "  \"registered_depth_to_color\": false,\n  \"frame_sync\": \"waitForFrames pairing; Astra S explicit sync unsupported\",\n  \"depth_encoding\": \"uint16 little-endian millimeters\",\n"
         << "  \"color_encoding\": \"RGB8 raw\"\n}\n";
    std::cout << "录制完成: " << count << " 帧，输出到 " << output << "\n";
    return 0;
}
catch(const ob::Error &e) {
    std::cerr << "Orbbec SDK 错误: " << e.getName() << ": " << e.getMessage() << "\n";
    return 1;
}
catch(const std::exception &e) {
    std::cerr << "录制失败: " << e.what() << "\n";
    return 1;
}
