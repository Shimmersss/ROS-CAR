// Read (and optionally set) standard UVC image controls of a USB camera over libusb control
// transfers, without streaming. Works on macOS while the system UVC driver owns the interface,
// so it needs no camera permission. Used to check the Gemini colour white balance.
//   uvc_controls [--vid 0x2bc5] [--set awb=1] [--set wb=4600] [--set ae=8]
// Build: clang -O2 scripts/uvc_controls.c -I/opt/homebrew/include/libusb-1.0 -L/opt/homebrew/lib -lusb-1.0
#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { GET_CUR = 0x81, GET_MIN = 0x82, GET_MAX = 0x83, GET_DEF = 0x87, SET_CUR = 0x01 };
typedef struct { const char *name; int pu; int selector; int size; } Control;
// pu=1: processing unit selector, pu=0: camera terminal selector (UVC 1.1 tables A-12, A-13).
static const Control CONTROLS[] = {
    {"awb", 1, 0x0B, 1}, {"wb", 1, 0x0A, 2}, {"gain", 1, 0x04, 2}, {"brightness", 1, 0x02, 2},
    {"contrast", 1, 0x03, 2}, {"saturation", 1, 0x07, 2}, {"hue", 1, 0x06, 2}, {"gamma", 1, 0x09, 2},
    {"sharpness", 1, 0x08, 2}, {"powerline", 1, 0x05, 1},
    {"ae", 0, 0x02, 1}, {"exposure", 0, 0x04, 4},
};
#define N_CONTROLS (sizeof(CONTROLS)/sizeof(CONTROLS[0]))

static long get(libusb_device_handle *h, int req, int iface, int unit, const Control *c, int *ok) {
    unsigned char buf[4] = {0};
    int r = libusb_control_transfer(h, 0xA1, req, c->selector << 8, unit << 8 | iface, buf, c->size, 500);
    *ok = r == c->size;
    long v = 0;
    for(int i = c->size-1; i >= 0; --i) v = v << 8 | buf[i];
    if(c->size == 2) v = (long)(short)v;   // signed for brightness/hue; unsigned ranges still fit
    return v;
}

static int set(libusb_device_handle *h, int iface, int unit, const Control *c, long value) {
    unsigned char buf[4];
    for(int i = 0; i < c->size; ++i) buf[i] = (value >> (8*i)) & 0xff;
    return libusb_control_transfer(h, 0x21, SET_CUR, c->selector << 8, unit << 8 | iface, buf, c->size, 500);
}

int main(int argc, char **argv) {
    int vid = 0x2bc5;
    const char *sets[8]; int n_sets = 0;
    for(int i = 1; i < argc; ++i) {
        if(!strcmp(argv[i], "--vid") && i+1 < argc) vid = (int)strtol(argv[++i], NULL, 0);
        else if(!strcmp(argv[i], "--set") && i+1 < argc && n_sets < 8) sets[n_sets++] = argv[++i];
        else { fprintf(stderr, "usage: %s [--vid 0x2bc5] [--set NAME=VALUE]...\n", argv[0]); return 2; }
    }
    libusb_context *ctx;
    if(libusb_init(&ctx)) return 1;
    libusb_device **list;
    ssize_t n = libusb_get_device_list(ctx, &list);
    int found = 0;
    for(ssize_t d = 0; d < n; ++d) {
        struct libusb_device_descriptor dd;
        libusb_get_device_descriptor(list[d], &dd);
        if(dd.idVendor != vid) continue;
        struct libusb_config_descriptor *cfg;
        if(libusb_get_active_config_descriptor(list[d], &cfg)) continue;
        libusb_device_handle *h;
        int err = libusb_open(list[d], &h);
        printf("device %04x:%04x bus %d addr %d%s\n", dd.idVendor, dd.idProduct, libusb_get_bus_number(list[d]),
               libusb_get_device_address(list[d]), err ? " (cannot open)" : "");
        for(int i = 0; i < cfg->bNumInterfaces && !err; ++i) {
            const struct libusb_interface_descriptor *alt = &cfg->interface[i].altsetting[0];
            if(alt->bInterfaceClass != 0x0E || alt->bInterfaceSubClass != 1) continue;   // video control
            int pu = -1, ct = -1;
            for(int k = 0; k + 2 < alt->extra_length; k += alt->extra[k]) {
                const unsigned char *x = alt->extra + k;
                if(x[0] < 3 || x[1] != 0x24) { if(x[0] == 0) break; continue; }
                if(x[2] == 0x05) pu = x[3];
                if(x[2] == 0x02 && x[0] >= 6 && (x[4] | x[5] << 8) == 0x0201) ct = x[3];
            }
            printf(" video control interface %d: processing unit %d, camera terminal %d\n",
                   alt->bInterfaceNumber, pu, ct);
            for(int s = 0; s < n_sets; ++s) {
                char name[32]; long value;
                if(sscanf(sets[s], "%31[^=]=%ld", name, &value) != 2) continue;
                for(size_t c = 0; c < N_CONTROLS; ++c) {
                    int unit = CONTROLS[c].pu ? pu : ct;
                    if(strcmp(name, CONTROLS[c].name) || unit < 0) continue;
                    int r = set(h, alt->bInterfaceNumber, unit, &CONTROLS[c], value);
                    printf("  set %s=%ld -> %s\n", name, value, r == CONTROLS[c].size ? "ok" : libusb_error_name(r));
                }
            }
            for(size_t c = 0; c < N_CONTROLS; ++c) {
                int unit = CONTROLS[c].pu ? pu : ct, ok, ok2, ok3, ok4;
                if(unit < 0) continue;
                long cur = get(h, GET_CUR, alt->bInterfaceNumber, unit, &CONTROLS[c], &ok);
                if(!ok) { printf("  %-10s unsupported\n", CONTROLS[c].name); continue; }
                long lo = get(h, GET_MIN, alt->bInterfaceNumber, unit, &CONTROLS[c], &ok2);
                long hi = get(h, GET_MAX, alt->bInterfaceNumber, unit, &CONTROLS[c], &ok3);
                long def = get(h, GET_DEF, alt->bInterfaceNumber, unit, &CONTROLS[c], &ok4);
                printf("  %-10s %ld", CONTROLS[c].name, cur);
                if(ok2 && ok3) printf("  (range %ld..%ld", lo, hi);
                if(ok4) printf(", default %ld", def);
                printf("%s\n", ok2 && ok3 ? ")" : "");
            }
            found = 1;
        }
        if(!err) libusb_close(h);
        libusb_free_config_descriptor(cfg);
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    if(!found) { fprintf(stderr, "no UVC video control interface for vendor %04x\n", vid); return 1; }
    return 0;
}
