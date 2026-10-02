#include "libobsensor/ObSensor.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cmath>
namespace fs = std::filesystem;
static float safe(float value) { return std::isfinite(value) ? value : 0.f; }
static void intrinsic(std::ostream &o, OBCameraIntrinsic p) {
    o << "{\"width\":" << p.width << ",\"height\":" << p.height
      << ",\"fx\":" << safe(p.fx) << ",\"fy\":" << safe(p.fy) << ",\"cx\":" << safe(p.cx) << ",\"cy\":" << safe(p.cy) << "}";
}
static void distortion(std::ostream &o, OBCameraDistortion p) {
    o << '[' << p.k1 << ',' << p.k2 << ',' << p.p1 << ',' << p.p2 << ',' << p.k3 << ',' << p.k4 << ',' << p.k5 << ',' << p.k6 << ']';
}
int main(int argc, char **argv) try {
    if(argc != 2) throw std::runtime_error("Usage: probe_gemini output_directory (captures 10 seconds)");
    fs::path out(argv[1]); fs::create_directories(out);
    ob::Context context;
    auto devices = context.queryDeviceList();
    if(devices->deviceCount() != 1) throw std::runtime_error("Require exactly one SDK depth device");
    auto device = devices->getDevice(0);
    auto info = device->getDeviceInfo();
    std::ofstream report(out / "device.json");
    report << std::setprecision(9) << "{\"name\":" << std::quoted(info->name())
           << ",\"serial\":" << std::quoted(info->serialNumber()) << ",\"firmware\":" << std::quoted(info->firmwareVersion())
           << ",\"vid\":" << info->vid() << ",\"pid\":" << info->pid() << ",\"sdk\":\"1.10.16\"";
    ob::Pipeline pipeline(device);
    std::shared_ptr<ob::StreamProfileList> colors;
    try { colors = pipeline.getStreamProfileList(OB_SENSOR_COLOR); }
    catch(const ob::Error &e) { std::cerr << "SDK color unavailable; capture UVC separately: " << e.getMessage() << '\n'; }
    auto depths = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    auto list = [&](const char *name, std::shared_ptr<ob::StreamProfileList> profiles) {
        report << ",\"" << name << "\":[";
        for(uint32_t i=0; i<profiles->count(); ++i) {
            auto p=profiles->getProfile(i)->as<ob::VideoStreamProfile>();
            if(i) report << ',';
            report << "{\"width\":" << p->width() << ",\"height\":" << p->height() << ",\"fps\":" << p->fps() << ",\"format\":" << p->format() << '}';
        }
        report << ']';
    };
    if(colors) list("color_profiles", colors); list("depth_profiles", depths);
    std::shared_ptr<ob::VideoStreamProfile> color;
    if(colors) color=colors->getVideoStreamProfile(640,480,OB_FORMAT_MJPG,30);
    auto depth = depths->getVideoStreamProfile(640,400,OB_FORMAT_Y12,30);
    auto config = std::make_shared<ob::Config>();
    if(color) config->enableStream(color); config->enableStream(depth); config->setAlignMode(ALIGN_DISABLE);
    pipeline.start(config);
    auto param = pipeline.getCameraParamWithProfile(640,480,640,400);
    auto calibrations=device->getCalibrationCameraParamList();
    std::cerr << "Stored calibration count=" << calibrations->count() << '\n';
    for(uint32_t i=0;i<calibrations->count();++i) {
        auto candidate=calibrations->getCameraParam(i);
        std::cerr << "Stored calibration " << i << " color " << candidate.rgbIntrinsic.width << "x" << candidate.rgbIntrinsic.height << " depth " << candidate.depthIntrinsic.width << "x" << candidate.depthIntrinsic.height << '\n';
        if(candidate.rgbIntrinsic.width==640 && candidate.rgbIntrinsic.height==480 && candidate.depthIntrinsic.width==640 && candidate.depthIntrinsic.height==400) param=candidate;
    }
    report << ",\"capture_mode\":" << (color ? "\"sdk_rgbd\"" : "\"sdk_depth_only_separate_uvc_required\"");
    report << ",\"calibration_valid\":" << (param.rgbIntrinsic.fx>0 && param.depthIntrinsic.fx>0 && param.rgbIntrinsic.width>0 ? "true" : "false");
    report << ",\"color_intrinsic\":"; intrinsic(report,param.rgbIntrinsic);
    report << ",\"depth_intrinsic\":"; intrinsic(report,param.depthIntrinsic);
    report << ",\"color_distortion\":"; distortion(report,param.rgbDistortion);
    report << ",\"depth_distortion\":"; distortion(report,param.depthDistortion);
    report << ",\"depth_to_color_rotation\":[";
    for(int i=0;i<9;++i) { if(i) report << ','; report << param.transform.rot[i]; }
    report << "],\"depth_to_color_translation_mm\":[";
    for(int i=0;i<3;++i) { if(i) report << ','; report << param.transform.trans[i]; }
    report << "],\"registration_verified\":false}"; report.close();
    std::ofstream csv(out/"timestamps.csv"); csv << "frame,color_ms,depth_ms,scale_mm\n";
    ob::FormatConvertFilter convert; convert.setFormatConvertType(FORMAT_MJPG_TO_RGB);
    auto start=std::chrono::steady_clock::now(); int n=0;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<10) {
        auto frames=pipeline.waitForFrames(200); if(!frames) continue;
        auto c=frames->colorFrame(); auto d=frames->depthFrame(); if(!d || (color && !c)) continue;
        if(c && c->format()!=OB_FORMAT_RGB) c=convert.process(c)->as<ob::ColorFrame>();
        ++n; csv << n << ',' << (c ? c->timeStamp() : 0) << ',' << d->timeStamp() << ',' << d->getValueScale() << '\n';
        // Keep at most ten real pairs, not a large video cache.
        if(n%30==1) {
            auto stem=std::to_string(n);
            if(c) { std::ofstream rgb(out/(stem+".rgb"),std::ios::binary); rgb.write(static_cast<const char *>(c->data()),c->dataSize()); }
            std::ofstream z(out/(stem+".depth"),std::ios::binary); z.write(static_cast<const char *>(d->data()),d->dataSize());
        }
    }
    pipeline.stop(); std::cout << info->name() << ": " << n << (color ? " RGB-D pairs; report " : " depth frames (separate UVC needed); report ") << out << '\n';
    return n ? 0 : 1;
} catch(const ob::Error &e) { std::cerr << e.getMessage() << '\n'; return 1; }
catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
