// Gemini native depth recorder (Orbbec SDK v1.10.x, macOS). Colour is recorded separately
// over UVC by record_gemini_mac.py. Writes device.json, timestamps.csv and N.depth
// (raw uint16 640x400) or N.depth.z (zlib level 1, lossless). Stops on SIGINT/SIGTERM,
// on a `stop` file in the output directory, or after the duration.
#include <libobsensor/ObSensor.hpp>
#include <zlib.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <time.h>

namespace fs = std::filesystem;
static std::atomic<bool> stop_requested{false};

static void intrinsic(std::ostream &o, OBCameraIntrinsic p) {
    o << "{\"width\":" << p.width << ",\"height\":" << p.height << ",\"fx\":" << p.fx
      << ",\"fy\":" << p.fy << ",\"cx\":" << p.cx << ",\"cy\":" << p.cy << '}';
}
static void distortion(std::ostream &o, OBCameraDistortion p) {
    // OpenCV order k1,k2,p1,p2,k3,k4,k5,k6 (matches gemini_<serial>.json).
    o << '[' << p.k1 << ',' << p.k2 << ',' << p.p1 << ',' << p.p2 << ',' << p.k3 << ',' << p.k4 << ',' << p.k5 << ',' << p.k6 << ']';
}
static long long wall_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// Same clock as Python time.monotonic_ns() in the colour recorder, so RGB/depth pairing is immune
// to wall-clock steps. On macOS that is CLOCK_UPTIME_RAW (mach_absolute_time); std::steady_clock
// is not (it also counts sleep), so it cannot be compared across the two processes.
static long long monotonic_ns() {
#ifdef __APPLE__
    return static_cast<long long>(clock_gettime_nsec_np(CLOCK_UPTIME_RAW));
#else
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<long long>(ts.tv_sec)*1000000000LL+ts.tv_nsec;
#endif
}

int main(int argc, char **argv) try {
    if(argc < 3) throw std::runtime_error("Usage: record_gemini_depth OUT_DIR SECONDS [--raw] [--serial SERIAL]");
    fs::path out(argv[1]);
    const double seconds = std::stod(argv[2]);
    bool compress = true;
    std::string want_serial;
    for(int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if(arg == "--raw") compress = false;
        else if(arg == "--serial" && i+1 < argc) want_serial = argv[++i];
        else throw std::runtime_error("unknown argument " + arg);
    }
    if(seconds <= 0 || seconds > 200) throw std::runtime_error("SECONDS must be in (0, 200]");
    fs::create_directories(out);
    std::signal(SIGINT, [](int) { stop_requested = true; });
    std::signal(SIGTERM, [](int) { stop_requested = true; });

    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    ob::Context context;
    auto devices = context.queryDeviceList();
    if(devices->deviceCount() != 1) throw std::runtime_error("Require exactly one SDK depth device");
    auto device = devices->getDevice(0);
    auto info = device->getDeviceInfo();
    if(!want_serial.empty() && want_serial != info->serialNumber())
        throw std::runtime_error(std::string("Serial mismatch: device ") + info->serialNumber());

    ob::Pipeline pipeline(device);
    auto depths = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
    auto depth = depths->getVideoStreamProfile(640, 400, OB_FORMAT_Y12, 30);
    auto config = std::make_shared<ob::Config>();
    config->enableStream(depth);
    config->setAlignMode(ALIGN_DISABLE);

    // Depth-only pipelines report empty intrinsics: use the stored calibration that matches
    // colour 640x480 / depth 640x400 (same source as gemini_AY2755200PW.json).
    auto calibrations = device->getCalibrationCameraParamList();
    bool found = false;
    OBCameraParam param{};
    for(uint32_t i = 0; i < calibrations->count(); ++i) {
        auto c = calibrations->getCameraParam(i);
        if(c.rgbIntrinsic.width == 640 && c.rgbIntrinsic.height == 480 &&
           c.depthIntrinsic.width == 640 && c.depthIntrinsic.height == 400) { param = c; found = true; break; }
    }
    if(!found) throw std::runtime_error("No stored 640x480/640x400 calibration on the device");
    {
        std::ofstream report(out / "device.json");
        report << std::setprecision(9) << "{\"name\":" << std::quoted(info->name())
               << ",\"serial\":" << std::quoted(info->serialNumber())
               << ",\"firmware\":" << std::quoted(info->firmwareVersion())
               << ",\"vid\":" << info->vid() << ",\"pid\":" << info->pid() << ",\"sdk\":\"1.10.16\"";
        report << ",\"color_intrinsic\":"; intrinsic(report, param.rgbIntrinsic);
        report << ",\"depth_intrinsic\":"; intrinsic(report, param.depthIntrinsic);
        report << ",\"color_distortion\":"; distortion(report, param.rgbDistortion);
        report << ",\"depth_distortion\":"; distortion(report, param.depthDistortion);
        report << ",\"depth_to_color_rotation\":[";
        for(int i = 0; i < 9; ++i) { if(i) report << ','; report << param.transform.rot[i]; }
        report << "],\"depth_to_color_translation_mm\":[";
        for(int i = 0; i < 3; ++i) { if(i) report << ','; report << param.transform.trans[i]; }
        report << "],\"depth_storage\":" << (compress ? "\"zlib\"" : "\"raw\"")
               << ",\"registration_verified\":false}";
    }

    std::ofstream csv(out / "timestamps.csv");
    csv << "frame,color_ms,depth_ms,scale_mm,host_wall_ns,host_monotonic_ns\n";
    pipeline.start(config);
    std::cout << "DEPTH_STARTED " << info->serialNumber() << std::endl;
    auto start = std::chrono::steady_clock::now();
    auto last_report = start;
    int n = 0;
    std::vector<unsigned char> packed;
    while(!stop_requested && !fs::exists(out / "stop") &&
          std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < seconds) {
        auto frames = pipeline.waitForFrames(200);
        if(!frames) continue;
        auto d = frames->depthFrame();
        if(!d || d->width() != 640 || d->height() != 400 || d->dataSize() != 640*400*2) continue;
        long long received = wall_ns(), received_mono = monotonic_ns();
        ++n;
        auto stem = out / std::to_string(n);
        const auto *data = static_cast<const unsigned char *>(d->data());
        if(compress) {
            uLongf size = compressBound(d->dataSize());
            packed.resize(size);
            if(compress2(packed.data(), &size, data, d->dataSize(), 1) != Z_OK)
                throw std::runtime_error("zlib compression failed");
            std::ofstream z(stem.string()+".depth.z", std::ios::binary);
            z.write(reinterpret_cast<const char *>(packed.data()), size);
        } else {
            std::ofstream z(stem.string()+".depth", std::ios::binary);
            z.write(reinterpret_cast<const char *>(data), d->dataSize());
        }
        csv << n << ",0," << d->timeStamp() << ',' << d->getValueScale() << ',' << received << ',' << received_mono << '\n';
        auto now = std::chrono::steady_clock::now();
        if(now-last_report >= std::chrono::seconds(1)) {
            csv.flush();
            std::cout << "DEPTH_FRAMES " << n << std::endl;
            last_report = now;
        }
    }
    csv.flush();
    std::cout << "DEPTH_STOPPING " << n << std::endl;
    pipeline.stop();
    std::cout << "DEPTH_DONE " << n << std::endl;
    return n ? 0 : 1;
} catch(const ob::Error &e) { std::cerr << "SDK error: " << e.getMessage() << '\n'; return 1; }
catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
