// Gemini colour diagnostics (Orbbec SDK v1.10.x): print colour sensor properties (auto white
// balance, white balance, exposure, gain, ...) with ranges, and optionally grab SDK colour frames
// and report the mean B/G/R. Read-only unless --set NAME=VALUE is given.
//   gemini_color_probe [--frames N] [--set awb=1] [--set wb=4600] [--set ae=1]
#include <libobsensor/ObSensor.hpp>
#include <iostream>
#include <map>
#include <string>
#include <vector>

struct Prop { const char *name; OBPropertyID id; bool is_bool; };
static const std::vector<Prop> PROPS = {
    {"awb", OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, true}, {"wb", OB_PROP_COLOR_WHITE_BALANCE_INT, false},
    {"ae", OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, true}, {"exposure", OB_PROP_COLOR_EXPOSURE_INT, false},
    {"gain", OB_PROP_COLOR_GAIN_INT, false}, {"brightness", OB_PROP_COLOR_BRIGHTNESS_INT, false},
    {"saturation", OB_PROP_COLOR_SATURATION_INT, false}, {"contrast", OB_PROP_COLOR_CONTRAST_INT, false},
    {"gamma", OB_PROP_COLOR_GAMMA_INT, false}, {"hue", OB_PROP_COLOR_HUE_INT, false},
    {"sharpness", OB_PROP_COLOR_SHARPNESS_INT, false},
};

static void print_props(const std::shared_ptr<ob::Device> &device) {
    for(auto &p : PROPS) {
        std::cout << "  " << p.name << ": ";
        try {
            if(!device->isPropertySupported(p.id, OB_PERMISSION_READ)) { std::cout << "not supported\n"; continue; }
            if(p.is_bool) std::cout << device->getBoolProperty(p.id) << '\n';
            else {
                auto r = device->getIntPropertyRange(p.id);
                std::cout << device->getIntProperty(p.id) << "  (range " << r.min << ".." << r.max
                          << ", default " << r.def << ")\n";
            }
        } catch(ob::Error &e) { std::cout << "error " << e.getMessage() << '\n'; }
    }
}

int main(int argc, char **argv) try {
    int frames = 0;
    std::vector<std::pair<std::string, int>> sets;
    for(int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if(arg == "--frames" && i+1 < argc) frames = std::stoi(argv[++i]);
        else if(arg == "--set" && i+1 < argc) {
            std::string kv = argv[++i]; auto eq = kv.find('=');
            if(eq == std::string::npos) throw std::runtime_error("--set needs NAME=VALUE");
            sets.emplace_back(kv.substr(0, eq), std::stoi(kv.substr(eq+1)));
        } else throw std::runtime_error("unknown argument " + arg);
    }
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    ob::Context context;
    auto devices = context.queryDeviceList();
    if(devices->deviceCount() != 1) throw std::runtime_error("Require exactly one SDK device");
    auto device = devices->getDevice(0);
    auto info = device->getDeviceInfo();
    std::cout << info->name() << " serial " << info->serialNumber() << " fw " << info->firmwareVersion() << '\n';
    auto sensors = device->getSensorList();
    std::cout << "sensors:";
    for(uint32_t i = 0; i < sensors->count(); ++i) std::cout << ' ' << sensors->type(i);
    std::cout << "  (OB_SENSOR_COLOR=" << OB_SENSOR_COLOR << ")\nproperties before:\n";
    print_props(device);
    for(auto &[name, value] : sets) {
        auto it = std::find_if(PROPS.begin(), PROPS.end(), [&](const Prop &p) { return name == p.name; });
        if(it == PROPS.end()) throw std::runtime_error("unknown property " + name);
        if(it->is_bool) device->setBoolProperty(it->id, value != 0); else device->setIntProperty(it->id, value);
        std::cout << "set " << name << " = " << value << '\n';
    }
    if(!sets.empty()) { std::cout << "properties after:\n"; print_props(device); }
    if(frames <= 0) return 0;

    ob::Pipeline pipeline(device);
    auto profiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
    std::shared_ptr<ob::StreamProfile> profile;
    try { profile = profiles->getVideoStreamProfile(640, 480, OB_FORMAT_RGB, 30); }
    catch(ob::Error &) { profile = profiles->getVideoStreamProfile(640, 0, OB_FORMAT_ANY, 30); }
    auto vp = profile->as<ob::VideoStreamProfile>();
    std::cout << "SDK colour stream " << vp->width() << 'x' << vp->height() << " format " << vp->format() << '\n';
    auto config = std::make_shared<ob::Config>();
    config->enableStream(profile);
    pipeline.start(config);
    for(int n = 0, got = 0; n < frames*10 && got < frames; ++n) {
        auto set = pipeline.waitForFrames(200);
        if(!set || !set->colorFrame()) continue;
        auto frame = set->colorFrame();
        ++got;
        if(frame->format() != OB_FORMAT_RGB) {
            std::cout << "frame " << got << " format " << frame->format() << " (mean only for RGB)\n";
            continue;
        }
        auto *px = static_cast<const uint8_t *>(frame->data());
        double sum[3] = {0, 0, 0};
        size_t count = frame->width()*frame->height();
        for(size_t i = 0; i < count; ++i) for(int c = 0; c < 3; ++c) sum[c] += px[3*i+c];
        if(got == 1 || got == frames || got % 10 == 0)
            std::cout << "frame " << got << " mean B/G/R " << int(sum[2]/count) << '/' << int(sum[1]/count) << '/'
                      << int(sum[0]/count) << '\n';
    }
    pipeline.stop();
    return 0;
} catch(ob::Error &e) {
    std::cerr << "SDK error: " << e.getName() << " " << e.getArgs() << " " << e.getMessage() << '\n';
    return 1;
} catch(std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}
