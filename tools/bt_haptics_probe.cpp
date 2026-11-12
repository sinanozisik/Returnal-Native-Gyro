// Standalone Bluetooth haptics probe. Does not link ReturnalGyro.
// Packet layouts:
//   saxense — egormanga/SAxense SAxense.c (DualSense and Edge, hidraw). https://github.com/egormanga/SAxense
//   anyps5  — AnyPS5 issue 2056, felt on a DualSense over Windows Bluetooth. https://github.com/boykopovar/AnyPS5/issues/2056
// CRC seed 0xA2 matches Linux hid-playstation PS_OUTPUT_CRC32_SEED and SAxense's ~0xEADA2D49 state.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <hidsdi.h>
#include <setupapi.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

namespace {
uint32_t crc32(const uint8_t* data, size_t size) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
void sign(uint8_t* report, unsigned size) {
    std::vector<uint8_t> seeded(size - 3);
    seeded[0] = 0xA2;
    std::memcpy(seeded.data() + 1, report, size - 4);
    const uint32_t crc = crc32(seeded.data(), seeded.size());
    std::memcpy(report + size - 4, &crc, 4);
}
std::string hexOf(const uint8_t* data, unsigned size) {
    static const char* digits = "0123456789abcdef";
    std::string out(size * 2, '0');
    for (unsigned i = 0; i < size; ++i) { out[i * 2] = digits[data[i] >> 4]; out[i * 2 + 1] = digits[data[i] & 0x0f]; }
    return out;
}
long long qpc() { LARGE_INTEGER n{}; QueryPerformanceCounter(&n); return n.QuadPart; }
long long qpcHz() { static long long hz = [] { LARGE_INTEGER n{}; QueryPerformanceFrequency(&n); return n.QuadPart ? n.QuadPart : 1; }(); return hz; }

struct Options {
    unsigned vid = 0x054C, pid = 0x0DF2;
    int index = 0;
    int seconds = 10;
    int gain = 100;
    int hz = 150;
    int rate = 3000;
    int length = 142;
    bool saxense = true;
    bool seqInc = false;
    bool mode = false;
    bool rumble = false;
    bool setOutput = false;
    bool selftest = false;
    bool list = false;
    uint8_t flag0 = 0;
    uint8_t flag1 = 0x62;
    uint8_t hapticFlags = 1;
    std::string variant = "saxense";
};

void usage() {
    std::puts(
        "BtHapticsProbe — DualSense / Edge Bluetooth haptics variants\n"
        "  --list                         show matching HID interfaces\n"
        "  --selftest                     check the CRC seed and exit\n"
        "  --pid 0DF2|0CE6                default 0DF2 (Edge)\n"
        "  --index N                      which listed interface to open\n"
        "  --variant NAME                 rumble | saxense | saxense-seq | saxense-pad |\n"
        "                                 saxense-output | saxense-mode | saxense-lpf | anyps5\n"
        "  --layout saxense|anyps5\n"
        "  --length 142|547\n"
        "  --seq fixed|inc\n"
        "  --mode on|off                  send a 0x31 before 0x32\n"
        "  --flag0 HH --flag1 HH          0x31 flag bytes when mode is on\n"
        "  --gain 0-127 --hz 150 --rate 3000 --seconds 10\n"
        "  --write writefile|setoutput\n"
        "  --rumble                       one second of classic 0x31 rumble, then stop\n");
}

bool argEq(const char* arg, const char* name) { return std::strcmp(arg, name) == 0; }
unsigned hexByte(const char* text) { return static_cast<unsigned>(std::strtoul(text, nullptr, 0)); }

void applyVariant(Options& opt) {
    if (opt.variant == "rumble") opt.rumble = true;
    else if (opt.variant == "saxense") { opt.saxense = true; opt.seqInc = false; opt.mode = false; opt.length = 142; opt.setOutput = false; }
    else if (opt.variant == "saxense-seq") { opt.saxense = true; opt.seqInc = true; opt.mode = false; opt.length = 142; }
    else if (opt.variant == "saxense-pad") { opt.saxense = true; opt.seqInc = false; opt.mode = false; opt.length = 547; }
    else if (opt.variant == "saxense-output") { opt.saxense = true; opt.seqInc = false; opt.mode = false; opt.length = 142; opt.setOutput = true; }
    else if (opt.variant == "saxense-mode") { opt.saxense = true; opt.seqInc = false; opt.mode = true; opt.flag0 = 0; opt.flag1 = 0x62; opt.hapticFlags = 1; }
    else if (opt.variant == "saxense-lpf") { opt.saxense = true; opt.seqInc = false; opt.mode = true; opt.flag0 = 0; opt.flag1 = 0x20; opt.hapticFlags = 1; }
    else if (opt.variant == "anyps5") { opt.saxense = false; opt.seqInc = true; opt.mode = true; opt.flag0 = 0; opt.flag1 = 0x62; opt.hapticFlags = 1; opt.length = 142; }
    else if (opt.variant != "custom") { std::fprintf(stderr, "unknown variant %s\n", opt.variant.c_str()); opt.variant.clear(); }
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        auto need = [&](const char* name) { if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", name); return false; } return true; };
        if (argEq(arg, "--help") || argEq(arg, "-h")) return false;
        if (argEq(arg, "--list")) { opt.list = true; continue; }
        if (argEq(arg, "--selftest")) { opt.selftest = true; continue; }
        if (argEq(arg, "--rumble")) { opt.rumble = true; continue; }
        if (argEq(arg, "--pid")) { if (!need(arg)) return false; opt.pid = hexByte(argv[++i]); continue; }
        if (argEq(arg, "--index")) { if (!need(arg)) return false; opt.index = std::atoi(argv[++i]); continue; }
        if (argEq(arg, "--seconds")) { if (!need(arg)) return false; opt.seconds = std::atoi(argv[++i]); continue; }
        if (argEq(arg, "--gain")) { if (!need(arg)) return false; opt.gain = std::clamp(std::atoi(argv[++i]), 0, 127); continue; }
        if (argEq(arg, "--hz")) { if (!need(arg)) return false; opt.hz = std::atoi(argv[++i]); continue; }
        if (argEq(arg, "--rate")) { if (!need(arg)) return false; opt.rate = std::max(1, std::atoi(argv[++i])); continue; }
        if (argEq(arg, "--length")) { if (!need(arg)) return false; opt.length = std::atoi(argv[++i]); opt.variant = "custom"; continue; }
        if (argEq(arg, "--flag0")) { if (!need(arg)) return false; opt.flag0 = static_cast<uint8_t>(hexByte(argv[++i])); opt.mode = true; opt.variant = "custom"; continue; }
        if (argEq(arg, "--flag1")) { if (!need(arg)) return false; opt.flag1 = static_cast<uint8_t>(hexByte(argv[++i])); opt.mode = true; opt.variant = "custom"; continue; }
        if (argEq(arg, "--layout")) { if (!need(arg)) return false; opt.saxense = std::strcmp(argv[++i], "anyps5") != 0; opt.variant = "custom"; continue; }
        if (argEq(arg, "--seq")) { if (!need(arg)) return false; opt.seqInc = std::strcmp(argv[++i], "inc") == 0; opt.variant = "custom"; continue; }
        if (argEq(arg, "--mode")) { if (!need(arg)) return false; opt.mode = std::strcmp(argv[++i], "on") == 0; opt.variant = "custom"; continue; }
        if (argEq(arg, "--write")) { if (!need(arg)) return false; opt.setOutput = std::strcmp(argv[++i], "setoutput") == 0; opt.variant = "custom"; continue; }
        if (argEq(arg, "--variant")) {
            if (!need(arg)) return false;
            opt.variant = argv[++i];
            applyVariant(opt);
            if (opt.variant.empty()) return false;
            continue;
        }
        std::fprintf(stderr, "unknown argument %s\n", arg);
        return false;
    }
    return true;
}

std::vector<std::wstring> hidPaths(unsigned vid, unsigned pid) {
    GUID iface{};
    HidD_GetHidGuid(&iface);
    HDEVINFO info = SetupDiGetClassDevsW(&iface, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    std::vector<std::wstring> bluetooth, other;
    if (info == INVALID_HANDLE_VALUE) return {};
    SP_DEVICE_INTERFACE_DATA data{};
    data.cbSize = sizeof data;
    for (DWORD index = 0; SetupDiEnumDeviceInterfaces(info, nullptr, &iface, index, &data); ++index) {
        DWORD bytes = 0;
        SetupDiGetDeviceInterfaceDetailW(info, &data, nullptr, 0, &bytes, nullptr);
        std::vector<uint8_t> buffer(bytes);
        auto* detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W*>(buffer.data());
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        if (!SetupDiGetDeviceInterfaceDetailW(info, &data, detail, bytes, nullptr, nullptr)) continue;
        HANDLE handle = CreateFileW(detail->DevicePath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
        if (handle == INVALID_HANDLE_VALUE) continue;
        HIDD_ATTRIBUTES attr{};
        attr.Size = sizeof attr;
        const bool match = HidD_GetAttributes(handle, &attr) && attr.VendorID == vid && attr.ProductID == pid;
        CloseHandle(handle);
        if (!match) continue;
        std::wstring path = detail->DevicePath;
        std::wstring lower = path;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::towlower);
        if (lower.find(L"00001124") != std::wstring::npos || lower.find(L"bthenum") != std::wstring::npos) bluetooth.push_back(path);
        else other.push_back(path);
    }
    SetupDiDestroyDeviceInfoList(info);
    bluetooth.insert(bluetooth.end(), other.begin(), other.end());
    return bluetooth;
}

bool writeReport(HANDLE handle, const uint8_t* report, unsigned size, bool setOutput, const char* label, bool fullHex) {
    std::vector<uint8_t> buffer(report, report + size);
    BOOL ok = FALSE;
    DWORD sent = 0;
    if (setOutput) ok = HidD_SetOutputReport(handle, buffer.data(), static_cast<ULONG>(buffer.size()));
    else ok = WriteFile(handle, buffer.data(), static_cast<DWORD>(buffer.size()), &sent, nullptr);
    const DWORD error = ok ? 0 : GetLastError();
    std::printf("%s size=%u ok=%d err=%lu sent=%lu\n", label, size, ok ? 1 : 0, error, sent);
    if (fullHex) std::printf("  %s\n", hexOf(buffer.data(), size).c_str());
    return ok == TRUE;
}

void build31(uint8_t* dst, uint8_t seq, uint8_t flag0, uint8_t flag1, uint8_t right, uint8_t left, uint8_t hapticFlags) {
    std::memset(dst, 0, 78);
    dst[0] = 0x31;
    dst[1] = static_cast<uint8_t>((seq & 0x0f) << 4);
    dst[2] = 0x10;
    dst[3] = flag0;
    dst[4] = flag1;
    dst[5] = right;
    dst[6] = left;
    dst[42] = hapticFlags;
    sign(dst, 78);
}

void build32(uint8_t* dst, unsigned size, bool saxense, uint8_t seq, uint8_t counter, const int8_t* samples) {
    std::memset(dst, 0, size);
    dst[0] = 0x32;
    dst[1] = static_cast<uint8_t>((seq & 0x0f) << 4);
    dst[2] = 0x91;
    dst[3] = 0x07;
    dst[4] = 0xFE;
    if (saxense) {
        dst[9] = 0xFF;
        dst[10] = counter;
    } else {
        dst[5] = 16;
        dst[10] = counter;
    }
    dst[11] = 0x92;
    dst[12] = 0x40;
    std::memcpy(dst + 13, samples, 64);
    sign(dst, 142);
}

int selftest() {
    uint8_t report[142]{};
    int8_t samples[64]{};
    for (int i = 0; i < 64; ++i) samples[i] = static_cast<int8_t>(i == 0 ? 100 : 0);
    build32(report, 142, true, 0, 0, samples);
    const bool header = report[0] == 0x32 && report[1] == 0 && report[2] == 0x91 && report[3] == 0x07 && report[4] == 0xFE
        && report[5] == 0 && report[9] == 0xFF && report[10] == 0 && report[11] == 0x92 && report[12] == 0x40 && report[13] == 100;
    uint8_t any[142]{};
    build32(any, 142, false, 3, 9, samples);
    const bool anyHeader = any[1] == 0x30 && any[5] == 16 && any[9] == 0 && any[10] == 9;
    uint32_t crc = 0;
    std::memcpy(&crc, report + 138, 4);
    std::vector<uint8_t> seeded(139);
    seeded[0] = 0xA2;
    std::memcpy(seeded.data() + 1, report, 138);
    const bool crcOk = crc32(seeded.data(), 139) == crc;
    std::printf("selftest saxense-header=%d anyps5-header=%d crc=%d\n", header ? 1 : 0, anyHeader ? 1 : 0, crcOk ? 1 : 0);
    std::printf("  saxense %s\n", hexOf(report, 16).c_str());
    std::printf("  anyps5  %s\n", hexOf(any, 16).c_str());
    return header && anyHeader && crcOk ? 0 : 1;
}

int run(const Options& opt) {
    if (opt.selftest) return selftest();
    const auto paths = hidPaths(opt.vid, opt.pid);
    if (opt.list || paths.empty()) {
        std::printf("HID VID=%04X PID=%04X count=%zu\n", opt.vid, opt.pid, paths.size());
        for (size_t i = 0; i < paths.size(); ++i) std::wprintf(L"  [%zu] %s\n", i, paths[i].c_str());
        if (!opt.list && paths.empty()) std::puts("no Bluetooth or USB interface matched");
        return paths.empty() && !opt.list ? 1 : 0;
    }
    if (opt.index < 0 || opt.index >= static_cast<int>(paths.size())) { std::fprintf(stderr, "index out of range\n"); return 1; }
    HANDLE handle = CreateFileW(paths[opt.index].c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
    if (handle == INVALID_HANDLE_VALUE) { std::fprintf(stderr, "CreateFile failed %lu\n", GetLastError()); return 1; }
    std::wprintf(L"open [%d] %s\n", opt.index, paths[opt.index].c_str());
    std::printf("variant=%s layout=%s length=%d seq=%s mode=%d write=%s gain=%d hz=%d rate=%d\n",
        opt.variant.c_str(), opt.saxense ? "saxense" : "anyps5", opt.length, opt.seqInc ? "inc" : "fixed", opt.mode ? 1 : 0,
        opt.setOutput ? "setoutput" : "writefile", opt.gain, opt.hz, opt.rate);
    uint8_t seq = 0;
    if (opt.rumble) {
        const long long start = qpc();
        const long long duration = qpcHz() * static_cast<long long>(opt.seconds);
        int sent = 0;
        while (qpc() - start < duration) {
            uint8_t report[78];
            build31(report, seq++, 0x03, 0, 0xFF, 0xFF, 0);
            writeReport(handle, report, 78, opt.setOutput, "rumble 0x31", sent < 2);
            ++sent;
            Sleep(50);
        }
        uint8_t stop[78];
        build31(stop, seq++, 0x03, 0, 0, 0, 0);
        writeReport(handle, stop, 78, opt.setOutput, "rumble stop", true);
        const double elapsed = static_cast<double>(qpc() - start) / static_cast<double>(qpcHz());
        std::printf("done sent=%d elapsed=%.2fs\n", sent, elapsed);
        CloseHandle(handle);
        return 0;
    }
    if (opt.mode) {
        uint8_t report[78];
        build31(report, seq++, opt.flag0, opt.flag1, 0, 0, opt.hapticFlags);
        std::printf("mode flag0=%02X flag1=%02X hapticFlags=%02X\n", opt.flag0, opt.flag1, opt.hapticFlags);
        writeReport(handle, report, 78, opt.setOutput, "mode 0x31", true);
        Sleep(20);
    }
    const double step = 2.0 * 3.14159265358979323846 * opt.hz / opt.rate;
    double phase = 0;
    const long long start = qpc();
    const long long duration = qpcHz() * static_cast<long long>(opt.seconds);
    const long long period = (std::max)(1LL, qpcHz() * 32 / opt.rate);
    int sent = 0, failed = 0, index = 0;
    uint8_t counter = 0;
    while (qpc() - start < duration) {
        const long long slot = start + period * index;
        const long long now = qpc();
        if (slot > now) {
            const DWORD wait = static_cast<DWORD>((slot - now) * 1000 / qpcHz());
            if (wait) Sleep(wait);
        }
        if (qpc() - start >= duration) break;
        int8_t samples[64]{};
        for (int i = 0; i < 32; ++i) {
            int sample = static_cast<int>(std::lround(opt.gain * std::sin(phase)));
            sample = std::clamp(sample, -127, 127);
            samples[i * 2] = samples[i * 2 + 1] = static_cast<int8_t>(sample);
            phase += step;
        }
        std::vector<uint8_t> report(static_cast<size_t>((std::max)(opt.length, 142)), 0);
        build32(report.data(), static_cast<unsigned>(report.size()), opt.saxense, opt.seqInc ? seq++ : 0, counter++, samples);
        const bool ok = writeReport(handle, report.data(), static_cast<unsigned>(report.size()), opt.setOutput, "haptic 0x32", sent < 2 || sent % 100 == 0);
        if (ok) ++sent; else ++failed;
        ++index;
    }
    const double elapsed = static_cast<double>(qpc() - start) / static_cast<double>(qpcHz());
    std::printf("done sent=%d failed=%d elapsed=%.2fs\n", sent, failed, elapsed);
    CloseHandle(handle);
    return failed && !sent ? 1 : 0;
}
}

int main(int argc, char** argv) {
    Options opt;
    if (argc < 2 || !parseArgs(argc, argv, opt)) { usage(); return argc < 2 ? 0 : 1; }
    return run(opt);
}
