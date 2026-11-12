#include "rg/bt_haptic_hook.hpp"
#include <opus.h>
#include "rg/dualsense_bt.hpp"
#include "rg/bt_haptic.hpp"
#include "rg/runtime.hpp"
#include "rg/scepad_bridge.hpp"
#include "rg/stall_watch.hpp"
#include <MinHook.h>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <audioclient.h>
#include <cfgmgr32.h>
#include <mmdeviceapi.h>
#include <propsys.h>
#include <timeapi.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cwctype>
#include <cstring>
#include <intrin.h>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>
namespace rg {
namespace {
using GetMmDeviceFn = bool(__fastcall*)(void*, int, IMMDevice**);
GetMmDeviceFn originalGetMmDevice{};
std::atomic<int> hookState{0}; // 0 pending, 1 installed, 2 missing
std::atomic<int> sentPackets{}, droppedPackets{}, crcErrors{};
std::atomic<int> speakerPackets{}, speakerDrops{}, speakerEncodeErr{}, speakerDepth{}, speakerCombined{};
std::atomic<int> speakerPeak{}, speakerFrames{};
std::atomic<long long> speakerSq{};
std::atomic<long long> latencySumUs{};
std::atomic<int> latencyCount{};
std::atomic<int> silentBuffers{}, liveBuffers{}, peak12{}, peak34{}, nonzeroSamples{};
std::atomic<long long> sumSq12{}, sumSq34{}, pcmFrames{};
std::atomic<int> noopLive{}, noopPeak12{}, noopPeak34{};
std::atomic<int> underruns{}, overruns{}, pendingWrites{}, ringDepth{}, selectedPair{};
std::mutex latencyMutex;
std::vector<long long> latencyUs;
std::mutex formatMutex;
std::string initFormat = "unset";

struct Sample { std::int8_t left, right; long long qpc; };
struct Engine {
    static constexpr int kRing = 512;
    std::mutex mutex;
    Sample ring[kRing]{};
    int ringHead = 0;
    int ringTail = 0;
    int ringCount = 0;
    bool modeSent = false;
    long long lastMode = 0;
    long long lastNonZero = 0;
    std::uint8_t block = 0;
    long long lastTone = 0;
    double tonePhase = 0;
    bool toneWasOn = false;
    std::atomic<bool> run{true};
    std::atomic<bool> hapticRunning{false};
    std::atomic<int> hapticOwner{0};
    static constexpr int kSpeakerRing = 4096;
    struct SpeakerSample { std::int16_t left, right; };
    SpeakerSample speaker[kSpeakerRing]{};
    int spkHead = 0;
    int spkTail = 0;
    int spkCount = 0;
    std::mutex eventMutex;
    HANDLE pacedEvents[8]{};
    int pacedCount = 0;
    std::atomic<bool> padArmed{false};
};
Engine& engine() { static Engine instance; return instance; }
long long qpc() {
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    return now.QuadPart;
}
long long qpcHz() {
    static long long hz = [] { LARGE_INTEGER v{}; QueryPerformanceFrequency(&v); return v.QuadPart; }();
    return hz;
}
void logLine(const std::string& text) { if (runtime) runtime->log(text); }
std::string iidText(REFIID id) {
    wchar_t wide[64]{};
    if (!StringFromGUID2(id, wide, 64)) return "?";
    char narrow[96]{};
    WideCharToMultiByte(CP_UTF8, 0, wide, -1, narrow, 96, nullptr, nullptr);
    return narrow;
}
void trace(const std::string& text) { logLine("BtHaptics " + text); }
void traceHr(const std::string& text, HRESULT hr) {
    char code[12]{};
    sprintf_s(code, "%08X", static_cast<unsigned>(hr));
    trace(text + " hr=" + code);
}
void noteUnknown(const char* where, REFIID id) {
    static std::mutex guard;
    static std::unordered_set<std::string> seen;
    const auto key = std::string(where) + " " + iidText(id);
    std::lock_guard lock(guard);
    if (!seen.insert(key).second || seen.size() > 48) return;
    traceHr(std::string(where) + " QueryInterface " + iidText(id), E_NOINTERFACE);
}
bool noteHot(const char* name) {
    static std::mutex guard;
    static int getBuffer = 0, releaseBuffer = 0, padding = 0;
    int* slot = nullptr;
    if (std::strcmp(name, "GetBuffer") == 0) slot = &getBuffer;
    else if (std::strcmp(name, "ReleaseBuffer") == 0) slot = &releaseBuffer;
    else if (std::strcmp(name, "GetCurrentPadding") == 0) slot = &padding;
    else return true;
    std::lock_guard lock(guard);
    if (*slot >= 8) return false;
    ++*slot;
    return true;
}
const GUID kPcmSubformat{0x00000001, 0x0000, 0x0010, {0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71}};
const PROPERTYKEY kEngineFormat{{0xf19f064d, 0x082c, 0x4e27, {0xbc, 0x73, 0x68, 0x82, 0xa1, 0xbb, 0x8e, 0x4c}}, 0};
const DEVPROPKEY kContainerProp{{0x8c7ed206, 0x3f8a, 0x4827, {0xb3, 0xab, 0xae, 0x9e, 0x1f, 0xae, 0xfc, 0x6c}}, 2};
const PROPERTYKEY kContainer{kContainerProp.fmtid, kContainerProp.pid};
const PROPERTYKEY kFriendlyName{{0xa45c254e, 0xdf1c, 0x4efd, {0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0}}, 14};
const PROPERTYKEY kDeviceDesc{{0xa45c254e, 0xdf1c, 0x4efd, {0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0}}, 2};
const PROPERTYKEY kInterfaceName{{0xb3f8fa53, 0x0004, 0x438e, {0x90, 0x03, 0x51, 0xa4, 0x6e, 0x13, 0x9b, 0xfc}}, 6};
const PROPERTYKEY kFormFactor{{0x1da5d803, 0xd492, 0x4edd, {0x8c, 0x23, 0xe0, 0xc0, 0xff, 0xee, 0x7f, 0x0e}}, 0};
bool edgePad() {
    return runtime && runtime->controllerProduct.load(std::memory_order_acquire) == 0x0df2;
}
const wchar_t* padInterfaceName() {
    return edgePad() ? L"DualSense Edge Wireless Controller" : L"DualSense Wireless Controller";
}
const wchar_t* padFriendlyName() {
    return edgePad() ? L"Speakers (DualSense Edge Wireless Controller)" : L"Speakers (DualSense Wireless Controller)";
}
std::string wideText(const wchar_t* text) {
    if (!text) return {};
    char narrow[200]{};
    WideCharToMultiByte(CP_UTF8, 0, text, -1, narrow, 180, nullptr, nullptr);
    return narrow;
}
bool controllerContainer(GUID& out) {
    static GUID found{};
    static std::atomic<bool> ready{false};
    if (ready.load(std::memory_order_acquire)) { out = found; return true; }
    const GUID hid{0x4d1e55b2, 0xf16f, 0x11cf, {0xb8, 0x8b, 0x00, 0xaa, 0x00, 0x3e, 0x80, 0x30}};
    ULONG bytes = 0;
    if (CM_Get_Device_Interface_List_SizeW(&bytes, const_cast<GUID*>(&hid), nullptr, CM_GET_DEVICE_INTERFACE_LIST_PRESENT) != CR_SUCCESS || bytes < 2) return false;
    std::vector<wchar_t> list(bytes);
    if (CM_Get_Device_Interface_ListW(const_cast<GUID*>(&hid), nullptr, list.data(), bytes, CM_GET_DEVICE_INTERFACE_LIST_PRESENT) != CR_SUCCESS) return false;
    const unsigned want = runtime ? runtime->controllerProduct.load(std::memory_order_acquire) : 0;
    for (const wchar_t* path = list.data(); path && *path; path += std::wcslen(path) + 1) {
        std::wstring lower(path);
        for (auto& ch : lower) ch = static_cast<wchar_t>(towlower(ch));
        if (lower.find(L"vid_054c") == std::wstring::npos) continue;
        const bool edge = lower.find(L"pid_0df2") != std::wstring::npos;
        const bool pad = lower.find(L"pid_0ce6") != std::wstring::npos;
        if (!edge && !pad) continue;
        if (want == 0x0df2 && !edge) continue;
        if (want == 0x0ce6 && !pad) continue;
        GUID id{};
        DEVPROPTYPE type = 0;
        ULONG size = sizeof id;
        if (CM_Get_Device_Interface_PropertyW(path, &kContainerProp, &type, reinterpret_cast<PBYTE>(&id), &size, 0) != CR_SUCCESS || type != DEVPROP_TYPE_GUID) continue;
        found = id;
        ready.store(true, std::memory_order_release);
        wchar_t wide[80]{};
        char text[80]{};
        StringFromGUID2(id, wide, 80);
        WideCharToMultiByte(CP_UTF8, 0, wide, -1, text, 80, nullptr, nullptr);
        logLine(std::string("BtHaptics container ") + text);
        out = id;
        return true;
    }
    return false;
}
bool setWide(PROPVARIANT* value, const wchar_t* text) {
    const std::size_t chars = std::wcslen(text) + 1;
    auto* copy = static_cast<wchar_t*>(CoTaskMemAlloc(chars * sizeof(wchar_t)));
    if (!copy) return false;
    std::memcpy(copy, text, chars * sizeof(wchar_t));
    value->vt = VT_LPWSTR;
    value->pwszVal = copy;
    return true;
}
bool sameKey(REFPROPERTYKEY key, const PROPERTYKEY& known) { return key.pid == known.pid && IsEqualGUID(key.fmtid, known.fmtid); }
WAVEFORMATEXTENSIBLE mixFormat();
bool setBlob(PROPVARIANT* value) {
    const auto mix = mixFormat();
    auto* blob = static_cast<std::uint8_t*>(CoTaskMemAlloc(sizeof mix));
    if (!blob) return false;
    std::memcpy(blob, &mix, sizeof mix);
    value->vt = VT_BLOB;
    value->blob.cbSize = sizeof mix;
    value->blob.pBlobData = blob;
    return true;
}
std::string callerText(void* address) {
    if (!address) return "unknown";
    HMODULE module = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, static_cast<LPCWSTR>(address), &module);
    wchar_t path[260]{};
    const wchar_t* file = L"?";
    if (module && GetModuleFileNameW(module, path, 260)) {
        const wchar_t* slash = wcsrchr(path, L'\\');
        file = slash ? slash + 1 : path;
    }
    char name[160]{};
    WideCharToMultiByte(CP_UTF8, 0, file, -1, name, 120, nullptr, nullptr);
    char line[200]{};
    sprintf_s(line, "%s+0x%llX", name, static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(address) - reinterpret_cast<std::uintptr_t>(module)));
    return line;
}
unsigned long long akAudioRva(void* address) {
    if (!address) return 0;
    HMODULE module = nullptr;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, static_cast<LPCWSTR>(address), &module) || !module) return 0;
    wchar_t path[260]{};
    if (!GetModuleFileNameW(module, path, 260) || !wcsstr(path, L"AkAudio")) return 0;
    return static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(address) - reinterpret_cast<std::uintptr_t>(module));
}
const unsigned long long kPadSinkRva = 0x1D57D3;
const unsigned long long kContainerSinkRva = 0x31630A;
const GUID kFloatSubformat{0x00000003, 0x0000, 0x0010, {0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71}};
WAVEFORMATEXTENSIBLE mixFormat() {
    // USB Edge PKEY_AudioEngine_DeviceFormat: 4ch 48 kHz 16-bit PCM, mask 0x33 (front L/R + back L/R).
    WAVEFORMATEXTENSIBLE mix{};
    mix.Format.wFormatTag = WAVE_FORMAT_EXTENSIBLE;
    mix.Format.nChannels = 4;
    mix.Format.nSamplesPerSec = 48000;
    mix.Format.wBitsPerSample = 16;
    mix.Format.nBlockAlign = 8;
    mix.Format.nAvgBytesPerSec = 48000 * 8;
    mix.Format.cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
    mix.Samples.wValidBitsPerSample = 16;
    mix.dwChannelMask = 0x33;
    mix.SubFormat = kPcmSubformat;
    return mix;
}
WAVEFORMATEXTENSIBLE floatMixFormat() {
    WAVEFORMATEXTENSIBLE mix{};
    mix.Format.wFormatTag = WAVE_FORMAT_EXTENSIBLE;
    mix.Format.nChannels = 4;
    mix.Format.nSamplesPerSec = 48000;
    mix.Format.wBitsPerSample = 32;
    mix.Format.nBlockAlign = kHapticFrameBytes;
    mix.Format.nAvgBytesPerSec = 48000 * kHapticFrameBytes;
    mix.Format.cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
    mix.Samples.wValidBitsPerSample = 32;
    mix.dwChannelMask = 0x33;
    mix.SubFormat = kFloatSubformat;
    return mix;
}
bool extensibleOf(const WAVEFORMATEX* format, const GUID& sub, unsigned valid, unsigned mask) {
    if (!format || format->wFormatTag != WAVE_FORMAT_EXTENSIBLE || format->cbSize < 22) return false;
    const auto* ext = reinterpret_cast<const WAVEFORMATEXTENSIBLE*>(format);
    return IsEqualGUID(ext->SubFormat, sub) && ext->Samples.wValidBitsPerSample == valid && ext->dwChannelMask == mask;
}
bool usbPcmFormat(const WAVEFORMATEX* format) {
    if (!format || format->nChannels != 4 || format->nSamplesPerSec != 48000 || format->wBitsPerSample != 16 || format->nBlockAlign != 8) return false;
    return format->wFormatTag == WAVE_FORMAT_PCM || extensibleOf(format, kPcmSubformat, 16, 0x33);
}
bool float4Format(const WAVEFORMATEX* format) {
    if (!format || format->nChannels != 4 || format->nSamplesPerSec != 48000 || format->wBitsPerSample != 32 || format->nBlockAlign < 16) return false;
    return format->wFormatTag == WAVE_FORMAT_IEEE_FLOAT || extensibleOf(format, kFloatSubformat, 32, 0x33);
}
std::string describeFormat(const WAVEFORMATEX* format) {
    if (!format) return "missing";
    std::string text = "tag=" + std::to_string(format->wFormatTag) + " ch=" + std::to_string(format->nChannels) + " rate=" + std::to_string(format->nSamplesPerSec) + " bits=" + std::to_string(format->wBitsPerSample) + " align=" + std::to_string(format->nBlockAlign) + " cb=" + std::to_string(format->cbSize);
    if (format->wFormatTag == WAVE_FORMAT_EXTENSIBLE && format->cbSize >= 22) {
        const auto* ext = reinterpret_cast<const WAVEFORMATEXTENSIBLE*>(format);
        text += " validBits=" + std::to_string(ext->Samples.wValidBitsPerSample) + " mask=" + std::to_string(ext->dwChannelMask) + " sub=" + iidText(ext->SubFormat);
    }
    return text;
}
bool mixFormatMatches(const WAVEFORMATEX* format) { return usbPcmFormat(format) || float4Format(format); }
bool btDualSenseWanted() {
    if (!runtime || !runtime->controllerBluetooth.load(std::memory_order_acquire)) return false;
    if (!runtime->snapshot().BtHaptics) return false;
    const unsigned product = runtime->controllerProduct.load(std::memory_order_acquire);
    return product == 0x0ce6 || product == 0x0df2;
}
void raisePeak(std::atomic<int>& slot, int value) {
    int current = slot.load(std::memory_order_relaxed);
    while (value > current && !slot.compare_exchange_weak(current, value, std::memory_order_relaxed)) {}
}
long long rmsOf(long long squares, long long frames) {
    if (frames <= 0) return 0;
    return static_cast<long long>(std::sqrt(static_cast<double>(squares) / static_cast<double>(frames)));
}
void publishStats() {
    static long long last = 0;
    const long long now = qpc();
    if (last && now - last < qpcHz()) return;
    last = now;
    const int sent = sentPackets.exchange(0);
    const int drops = droppedPackets.exchange(0);
    const int crc = crcErrors.exchange(0);
    const int underrun = underruns.exchange(0);
    const int overrun = overruns.exchange(0);
    const int pending = pendingWrites.load(std::memory_order_relaxed);
    const int depth = ringDepth.load(std::memory_order_relaxed);
    const int pair = selectedPair.load(std::memory_order_relaxed);
    const int silent = silentBuffers.exchange(0);
    const int live = liveBuffers.exchange(0);
    const int p12 = peak12.exchange(0);
    const int p34 = peak34.exchange(0);
    const long long frames = pcmFrames.exchange(0);
    const long long sq12 = sumSq12.exchange(0);
    const long long sq34 = sumSq34.exchange(0);
    const double rms12 = frames ? std::sqrt(static_cast<double>(sq12) / static_cast<double>(frames)) / 1000000.0 : 0;
    const double rms34 = frames ? std::sqrt(static_cast<double>(sq34) / static_cast<double>(frames)) / 1000000.0 : 0;
    const int nonzero = nonzeroSamples.exchange(0);
    const int nopLive = noopLive.exchange(0);
    const int nop12 = noopPeak12.exchange(0);
    const int nop34 = noopPeak34.exchange(0);
    std::vector<long long> samples;
    {
        std::lock_guard lock(latencyMutex);
        samples.swap(latencyUs);
    }
    std::sort(samples.begin(), samples.end());
    const long long p50 = samples.empty() ? 0 : samples[(samples.size() - 1) * 50 / 100];
    const long long p95 = samples.empty() ? 0 : samples[(samples.size() - 1) * 95 / 100];
    const long long pmax = samples.empty() ? 0 : samples.back();
    std::string format;
    {
        std::lock_guard lock(formatMutex);
        format = initFormat;
    }
    auto unit = [](double value) { char text[32]{}; sprintf_s(text, "%.6g", value); return std::string(text); };
    if (sent || drops || crc || underrun || overrun || silent || live || nopLive)
        logLine("BtHaptics packets=" + std::to_string(sent) + "/s drops=" + std::to_string(drops) + " underrun=" + std::to_string(underrun) + " overrun=" + std::to_string(overrun)
            + " pending=" + std::to_string(pending) + " queue=" + std::to_string(depth) + "/" + std::to_string(Engine::kRing)
            + " latencyUs p50=" + std::to_string(p50) + " p95=" + std::to_string(p95) + " max=" + std::to_string(pmax) + " crcErr=" + std::to_string(crc)
            + " silent=" + std::to_string(silent) + " live=" + std::to_string(live)
            + " peak12=" + unit(p12 / 1000000.0) + " rms12=" + unit(rms12)
            + " peak34=" + unit(p34 / 1000000.0) + " rms34=" + unit(rms34)
            + " pair=" + std::to_string(pair) + " nonzero=" + std::to_string(nonzero)
            + " fmt=" + format
            + " noopLive=" + std::to_string(nopLive) + " noopPeak12=" + unit(nop12 / 1000000.0) + " noopPeak34=" + unit(nop34 / 1000000.0));
    const int spk = speakerPackets.exchange(0);
    const int spkDrop = speakerDrops.exchange(0);
    const int spkErr = speakerEncodeErr.exchange(0);
    const int spkFrames = speakerFrames.exchange(0);
    const long long spkSq = speakerSq.exchange(0);
    const int spkPeak = speakerPeak.exchange(0);
    const int spkDepth = speakerDepth.load(std::memory_order_relaxed);
    if (spk || spkDrop || spkErr || spkFrames || speakerStreamActive()) {
        const double spkRms = spkFrames ? std::sqrt(static_cast<double>(spkSq) / static_cast<double>(spkFrames)) / 1000000.0 : 0;
        const int spkCombined = speakerCombined.exchange(0);
        logLine("BtSpeaker packets=" + std::to_string(spk) + "/s encodeErr=" + std::to_string(spkErr) + " drops=" + std::to_string(spkDrop)
            + " combined=" + std::to_string(spkCombined)
            + " queue=" + std::to_string(spkDepth) + "/" + std::to_string(Engine::kSpeakerRing)
            + " peak=" + unit(spkPeak / 1000000.0) + " rms=" + unit(spkRms));
    }
}
std::string hexBytes(const std::uint8_t* data, unsigned length) {
    static const char* digits = "0123456789abcdef";
    std::string out(static_cast<std::size_t>(length) * 2, '0');
    for (unsigned i = 0; i < length; ++i) {
        out[i * 2] = digits[data[i] >> 4];
        out[i * 2 + 1] = digits[data[i] & 0x0f];
    }
    return out;
}
void clearHapticRing() {
    auto& hub = engine();
    std::lock_guard lock(hub.mutex);
    hub.ringHead = hub.ringTail = hub.ringCount = 0;
    ringDepth.store(0, std::memory_order_relaxed);
}
void enqueueHapticSample(std::int8_t left, std::int8_t right) {
    auto& hub = engine();
    std::lock_guard lock(hub.mutex);
    if (hub.ringCount == Engine::kRing) {
        hub.ringHead = (hub.ringHead + 1) % Engine::kRing;
        --hub.ringCount;
        ++overruns;
    }
    hub.ring[hub.ringTail] = {left, right, qpc()};
    hub.ringTail = (hub.ringTail + 1) % Engine::kRing;
    ++hub.ringCount;
    ringDepth.store(hub.ringCount, std::memory_order_relaxed);
}
bool takeHapticBlock(std::array<std::int8_t, 64>& block, long long& stamp) {
    auto& hub = engine();
    std::lock_guard lock(hub.mutex);
    if (hub.ringCount < 32) return false;
    const int extra = hapticBacklogDrop(hub.ringCount);
    if (extra) ++overruns;
    for (int i = 0; i < extra; ++i) {
        hub.ringHead = (hub.ringHead + 1) % Engine::kRing;
        --hub.ringCount;
    }
    stamp = hub.ring[hub.ringHead].qpc;
    int nonzero = 0;
    for (int i = 0; i < 32; ++i) {
        const Sample sample = hub.ring[hub.ringHead];
        hub.ringHead = (hub.ringHead + 1) % Engine::kRing;
        --hub.ringCount;
        block[static_cast<std::size_t>(i) * 2] = sample.left;
        block[static_cast<std::size_t>(i) * 2 + 1] = sample.right;
        if (sample.left || sample.right) ++nonzero;
    }
    ringDepth.store(hub.ringCount, std::memory_order_relaxed);
    nonzeroSamples += nonzero;
    return true;
}
void enqueueSpeakerSample(std::int16_t left, std::int16_t right) {
    auto& hub = engine();
    std::lock_guard lock(hub.mutex);
    if (hub.spkCount == Engine::kSpeakerRing) {
        hub.spkHead = (hub.spkHead + 1) % Engine::kSpeakerRing;
        --hub.spkCount;
        ++speakerDrops;
    }
    hub.speaker[hub.spkTail] = {left, right};
    hub.spkTail = (hub.spkTail + 1) % Engine::kSpeakerRing;
    ++hub.spkCount;
    speakerDepth.store(hub.spkCount, std::memory_order_relaxed);
}
bool takeSpeakerBlock(std::int16_t* interleaved512) {
    auto& hub = engine();
    std::lock_guard lock(hub.mutex);
    if (hub.spkCount < 512) return false;
    for (int i = 0; i < 512; ++i) {
        const auto sample = hub.speaker[hub.spkHead];
        hub.spkHead = (hub.spkHead + 1) % Engine::kSpeakerRing;
        --hub.spkCount;
        interleaved512[i * 2] = sample.left;
        interleaved512[i * 2 + 1] = sample.right;
    }
    speakerDepth.store(hub.spkCount, std::memory_order_relaxed);
    return true;
}
void compress512to480(const std::int16_t* in, std::int16_t* out) {
    for (int i = 0; i < 480; ++i) {
        const double pos = static_cast<double>(i) * 511.0 / 479.0;
        const int index = static_cast<int>(pos);
        const int next = index + 1 < 512 ? index + 1 : index;
        const double frac = pos - index;
        for (int channel = 0; channel < 2; ++channel) {
            const double a = in[index * 2 + channel];
            const double b = in[next * 2 + channel];
            out[i * 2 + channel] = static_cast<std::int16_t>(std::lround(a + (b - a) * frac));
        }
    }
}
struct SpeakerDump {
    std::atomic<int> stage{0};
    std::uint8_t bytes[334]{};
    int opusHot = 0;
    int index = 0;
    bool combined = false;
};
SpeakerDump& speakerDump() { static SpeakerDump dump; return dump; }
bool queueSpeakerDump(const std::uint8_t* report, int opusHot, bool combined, int index) {
    auto& dump = speakerDump();
    int idle = 0;
    if (!dump.stage.compare_exchange_strong(idle, 1, std::memory_order_acq_rel)) return false;
    std::memcpy(dump.bytes, report, 334);
    dump.opusHot = opusHot;
    dump.combined = combined;
    dump.index = index;
    dump.stage.store(2, std::memory_order_release);
    return true;
}
void logQueuedSpeakerDump() {
    auto& dump = speakerDump();
    if (dump.stage.load(std::memory_order_acquire) != 2) return;
    trace("speaker 0x35 nonzero #" + std::to_string(dump.index) + " combined=" + (dump.combined ? "1" : "0")
        + " opusNonzero=" + std::to_string(dump.opusHot)
        + " hdr=0-12 opus=13-212 haptic=213-278"
        + " raw=" + hexBytes(dump.bytes, 334));
    dump.stage.store(0, std::memory_order_release);
}
void noteWriteDone(long long stamp, bool countPacket) {
    if (countPacket) ++sentPackets;
    if (!stamp) return;
    const long long us = (qpc() - stamp) * 1000000 / qpcHz();
    std::lock_guard lock(latencyMutex);
    if (latencyUs.size() >= 256) latencyUs.erase(latencyUs.begin());
    latencyUs.push_back(us);
}
struct IoSlot {
    OVERLAPPED ov{};
    std::uint8_t report[547]{};
    long long stamp = 0;
    bool busy = false;
    bool countHaptic = false;
};
void reapSlot(IoSlot& slot, HANDLE hid, int kind) {
    if (!slot.busy || !hid) return;
    if (WaitForSingleObject(slot.ov.hEvent, 0) != WAIT_OBJECT_0) return;
    DWORD sent = 0;
    const BOOL ok = GetOverlappedResult(hid, &slot.ov, &sent, FALSE);
    slot.busy = false;
    if (ok) noteWriteDone(slot.stamp, kind == 1 || slot.countHaptic);
    else if (kind == 2) ++speakerDrops;
    else ++droppedPackets;
    slot.countHaptic = false;
}
bool testToneWanted() {
    if (!runtime || !runtime->snapshot().BtHaptics || !runtime->snapshot().BtHapticsTestTone) return false;
    if (!runtime->controllerBluetooth.load(std::memory_order_acquire)) return false;
    const unsigned product = runtime->controllerProduct.load(std::memory_order_acquire);
    return product == 0x0ce6 || product == 0x0df2;
}
void waitUntil(HANDLE timer, long long when) {
    const long long now = qpc();
    if (now >= when) return;
    long long hundredNs = (when - now) * 10000000 / qpcHz();
    if (hundredNs < 1) return;
    LARGE_INTEGER due{};
    due.QuadPart = -hundredNs;
    if (!timer || !SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE)) return;
    WaitForSingleObject(timer, 30);
}
void senderLoop() {
    timeBeginPeriod(1);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, 0x00000002, TIMER_ALL_ACCESS);
    if (!timer) timer = CreateWaitableTimerW(nullptr, FALSE, nullptr);
    IoSlot data[8]{};
    IoSlot speakerSlots[4]{};
    IoSlot mode{};
    for (auto& slot : data) slot.ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    for (auto& slot : speakerSlots) slot.ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    mode.ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    HANDLE hid = nullptr;
    std::wstring opened;
    bool sync = false;
    bool loggedOpen = false;
    std::uint8_t modeSeq = 0;
    const long long period = (std::max)(1LL, qpcHz() * 32 / 3000);
    long long next = qpc();
    trace("sender priority=highest slots=8 cadenceUs=10667. Speaker report 0x35 follows haptics when BtSpeaker=1");
    auto submit = [&](IoSlot& slot, const std::uint8_t* bytes, unsigned size, long long stamp, bool countPacket, bool countDrop) {
        if (countDrop && runtime && !runtime->snapshot().BtHapticsSend) return false;
        stamp = qpc();
        if (sync || !hid) {
            std::uint8_t copy[547]{};
            std::memcpy(copy, bytes, size);
            const bool ok = sendTrackedBluetoothReport(copy, size);
            if (ok) noteWriteDone(stamp, countPacket);
            else if (countDrop) ++droppedPackets;
            return ok;
        }
        if (slot.busy) return false;
        std::memset(slot.report, 0, sizeof slot.report);
        std::memcpy(slot.report, bytes, size);
        slot.stamp = stamp;
        ResetEvent(slot.ov.hEvent);
        DWORD wrote = 0;
        const BOOL ok = WriteFile(hid, slot.report, size, &wrote, &slot.ov);
        if (ok) { noteWriteDone(stamp, countPacket); return true; }
        const DWORD error = GetLastError();
        if (error == ERROR_IO_PENDING) { slot.busy = true; return true; }
        if (error == ERROR_INVALID_USER_BUFFER) {
            const BOOL padded = WriteFile(hid, slot.report, 547, &wrote, &slot.ov);
            if (padded) { noteWriteDone(stamp, countPacket); return true; }
            if (GetLastError() == ERROR_IO_PENDING) { slot.busy = true; return true; }
        }
        if (error == ERROR_INVALID_PARAMETER) {
            sync = true;
            trace("overlapped WriteFile rejected; sender fell back to synchronous writes");
            std::uint8_t copy[547]{};
            std::memcpy(copy, bytes, size);
            const bool fallback = sendTrackedBluetoothReport(copy, size);
            if (fallback) noteWriteDone(stamp, countPacket);
            else if (countDrop) ++droppedPackets;
            return fallback;
        }
        if (countDrop) ++droppedPackets;
        return false;
    };
    int speakerErr = 0;
    OpusEncoder* speakerEncoder = opus_encoder_create(48000, 2, OPUS_APPLICATION_AUDIO, &speakerErr);
    if (speakerEncoder) {
        opus_encoder_ctl(speakerEncoder, OPUS_SET_BITRATE(160000));
        opus_encoder_ctl(speakerEncoder, OPUS_SET_VBR(0));
    }
    std::uint8_t speakerSeq = 0;
    std::uint8_t speakerCounter = 0;
    auto sendSpeaker = [&](long long now, const std::int8_t* haptic64) {
        const auto settings = runtime ? runtime->snapshot() : Settings{};
        if (!runtime || !settings.BtSpeaker || !settings.BtSpeakerReport || !speakerStreamActive()) return;
        std::array<std::int16_t, 512 * 2> raw{};
        const bool got = takeSpeakerBlock(raw.data());
        if (!got && !haptic64) return;
        if (!got) raw.fill(0);
        std::array<std::int16_t, 480 * 2> pcm{};
        compress512to480(raw.data(), pcm.data());
        unsigned char encoded[200]{};
        const int bytes = speakerEncoder ? opus_encode(speakerEncoder, pcm.data(), 480, encoded, 200) : -1;
        if (bytes != 200) { ++speakerEncodeErr; return; }
        std::uint8_t report[334]{};
        const bool likeHaptic = settings.BtSpeakerCounterStyle == 0;
        if (!buildBluetoothSpeakerReport(report, speakerSeq, speakerCounter, encoded, haptic64, likeHaptic)) { ++speakerEncodeErr; return; }
        speakerSeq = static_cast<std::uint8_t>((speakerSeq + 1) & 0x0f);
        speakerCounter = static_cast<std::uint8_t>(speakerCounter + (likeHaptic ? 1 : 2));
        int opusHot = 0;
        for (int i = 13; i < 213; ++i) if (report[i]) ++opusHot;
        bool hapticHot = false;
        if (haptic64) for (int i = 215; i < 279; ++i) if (report[i]) { hapticHot = true; break; }
        static int dumped = 0;
        if (dumped < 3 && hapticHot && opusHot >= 16 && queueSpeakerDump(report, opusHot, haptic64 != nullptr, dumped + 1)) ++dumped;
        IoSlot* free = nullptr;
        for (auto& slot : speakerSlots) if (!slot.busy) { free = &slot; break; }
        if (!free) { ++speakerDrops; return; }
        free->countHaptic = haptic64 != nullptr;
        if (!submit(*free, report, 334, now, haptic64 != nullptr, false)) { free->countHaptic = false; ++speakerDrops; return; }
        ++speakerPackets;
        if (haptic64) { ++speakerCombined; setHapticStreamActive(true); }
    };
    auto& hub = engine();
    while (hub.run.load(std::memory_order_acquire)) {
        wchar_t path[2048]{};
        const bool havePath = copyNewestBluetoothPath(path, 2048);
        if (!sync && havePath && opened != path) {
            for (auto& slot : data) reapSlot(slot, hid, 1);
            for (auto& slot : speakerSlots) reapSlot(slot, hid, 2);
            reapSlot(mode, hid, 0);
            bool busy = mode.busy;
            for (const auto& slot : data) busy = busy || slot.busy;
            for (const auto& slot : speakerSlots) busy = busy || slot.busy;
            if (hid && busy) {
                CancelIo(hid);
                for (auto& slot : data) if (slot.busy) { WaitForSingleObject(slot.ov.hEvent, 20); reapSlot(slot, hid, 1); }
                for (auto& slot : speakerSlots) if (slot.busy) { WaitForSingleObject(slot.ov.hEvent, 20); reapSlot(slot, hid, 2); }
                if (mode.busy) { WaitForSingleObject(mode.ov.hEvent, 20); reapSlot(mode, hid, 0); }
            }
            if (hid) CloseHandle(hid);
            hid = CreateFileW(path, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
            if (hid == INVALID_HANDLE_VALUE) {
                hid = nullptr;
                opened.clear();
                next = qpc() + period;
                static int openFailures = 0;
                if (openFailures < 4) { ++openFailures; trace("overlapped HID open failed " + std::to_string(GetLastError()) + "; retrying"); }
                waitUntil(timer, qpc() + qpcHz() / 5);
                continue;
            }
            opened = path;
            if (!loggedOpen) { loggedOpen = true; trace(std::string("sender hid ") + (sync ? "sync" : "overlapped")); }
        }
        for (auto& slot : data) reapSlot(slot, hid, 1);
        for (auto& slot : speakerSlots) reapSlot(slot, hid, 2);
        reapSlot(mode, hid, 0);
        int pending = mode.busy ? 1 : 0;
        for (const auto& slot : data) if (slot.busy) ++pending;
        pendingWrites.store(pending, std::memory_order_relaxed);
        const bool tone = testToneWanted();
        const bool running = hub.hapticRunning.load(std::memory_order_acquire);
        const bool speaking = speakerStreamActive();
        if (!tone && !running && !speaking && ringDepth.load(std::memory_order_relaxed) == 0 && speakerDepth.load(std::memory_order_relaxed) == 0) {
            if (hub.toneWasOn || hub.modeSent) { hub.toneWasOn = false; hub.modeSent = false; hub.lastMode = 0; setHapticStreamActive(false); }
            waitUntil(timer, qpc() + qpcHz() / 20);
            next = qpc() + period;
            continue;
        }
        waitUntil(timer, next);
        const long long now = qpc();
        if (now > next + period) next = now;
        next += period;
        if (tone && !hub.toneWasOn) { hub.toneWasOn = true; clearHapticRing(); }
        if (!tone) hub.toneWasOn = false;
        std::array<std::int8_t, 64> block{};
        long long stamp = now;
        bool underrun = false;
        if (tone || running) {
            if (tone) {
                for (int i = 0; i < 32; ++i) {
                    int sample = static_cast<int>(std::lround(100.0 * std::sin(hub.tonePhase)));
                    if (sample > 127) sample = 127;
                    if (sample < -128) sample = -128;
                    const auto narrow = static_cast<std::int8_t>(sample);
                    block[static_cast<std::size_t>(i) * 2] = narrow;
                    block[static_cast<std::size_t>(i) * 2 + 1] = narrow;
                    hub.tonePhase += 2.0 * 3.14159265358979323846 * 150.0 / 3000.0;
                    if (hub.tonePhase > 2.0 * 3.14159265358979323846) hub.tonePhase -= 2.0 * 3.14159265358979323846;
                }
                nonzeroSamples += 32;
            } else if (!takeHapticBlock(block, stamp)) {
                underrun = true;
                ++underruns;
                stamp = now;
            }
            const bool full = runtime && runtime->snapshot().BtHapticsModeFlags != 0;
            if (!hub.modeSent || !hub.lastMode || now - hub.lastMode >= qpcHz()) {
                std::uint8_t modeReport[78]{};
                if (buildBluetoothHapticModeReport(modeReport, modeSeq, full)) {
                    const auto snap = runtime ? runtime->snapshot() : Settings{};
                    const bool speakerBits = speaking && runtime && snap.BtSpeaker && snap.BtSpeakerFlags31;
                    if (speakerBits) {
                        applyBluetoothSpeaker(modeReport, static_cast<std::uint8_t>(std::clamp(snap.BtSpeakerVolume, 0, 127)));
                        static bool compared = false;
                        if (!compared) {
                            compared = true;
                            trace("speaker 0x31 vs dualsense-neo: flag0|=" + hexBytes(modeReport + 3, 1)
                                + " (want f0) flag1=" + hexBytes(modeReport + 4, 1)
                                + " vol=" + hexBytes(modeReport + 8, 1)
                                + " audio=" + hexBytes(modeReport + 10, 1) + " (want 30)"
                                + " preamp=" + hexBytes(modeReport + 40, 1) + " (want 05)");
                        }
                    }
                    refreshBluetoothOutputCrc(modeReport, 78);
                    if (!mode.busy && submit(mode, modeReport, 78, now, false, true)) {
                        hub.modeSent = true;
                        hub.lastMode = now;
                        setHapticStreamActive(true);
                        ++modeSeq;
                        trace(std::string("mode 0x31 ") + (full ? "full " : "lpf ")
                            + " report=" + (snap.BtSpeakerReport ? "1" : "0")
                            + " flags31=" + (snap.BtSpeakerFlags31 ? "1" : "0")
                            + " speakerBits=" + (speakerBits ? "1" : "0")
                            + " flag0=" + hexBytes(modeReport + 3, 1) + " flag1=" + hexBytes(modeReport + 4, 1)
                            + " vol=" + hexBytes(modeReport + 8, 1) + " audio=" + hexBytes(modeReport + 10, 1)
                            + " audio2=" + hexBytes(modeReport + 40, 1) + " haptic=" + hexBytes(modeReport + 42, 1)
                            + " raw=" + hexBytes(modeReport, 48));
                    }
                }
            }
            static bool routeLogged = false;
            if (speaking && runtime && !routeLogged) {
                routeLogged = true;
                const auto snap = runtime->snapshot();
                const bool rep = snap.BtSpeaker && snap.BtSpeakerReport;
                trace(std::string("speaker route report=") + (snap.BtSpeakerReport ? "1" : "0")
                    + " flags31=" + (snap.BtSpeakerFlags31 ? "1" : "0")
                    + " combine=" + (rep ? "1" : "0")
                    + " separate0x32=" + (rep ? "0" : "1")
                    + " mode0x31=every 1s on the sender hid; game 0x31 is still merged");
            }
            const bool combine = speaking && runtime && runtime->snapshot().BtSpeaker && runtime->snapshot().BtSpeakerReport;
            if (!combine) {
                std::uint8_t report[142]{};
                if (buildBluetoothHapticReport(report, block.data(), 0, hub.block++) && hapticReportCrcValid(report)) {
                    IoSlot* free = nullptr;
                    for (auto& slot : data) if (!slot.busy) { free = &slot; break; }
                    if (!free) ++droppedPackets;
                    else if (submit(*free, report, 142, stamp, true, true)) setHapticStreamActive(true);
                } else { ++crcErrors; ++droppedPackets; }
            }
            sendSpeaker(now, combine ? block.data() : nullptr);
        } else sendSpeaker(now, nullptr);
        (void)underrun;
    }
    if (speakerEncoder) opus_encoder_destroy(speakerEncoder);
    if (hid) {
        CancelIo(hid);
        CloseHandle(hid);
    }
    if (timer) CloseHandle(timer);
    for (auto& slot : data) if (slot.ov.hEvent) CloseHandle(slot.ov.hEvent);
    for (auto& slot : speakerSlots) if (slot.ov.hEvent) CloseHandle(slot.ov.hEvent);
    if (mode.ov.hEvent) CloseHandle(mode.ov.hEvent);
    timeEndPeriod(1);
}
void watchPacedEvent(HANDLE event) {
    if (!event) return;
    auto& hub = engine();
    std::lock_guard lock(hub.eventMutex);
    for (int i = 0; i < hub.pacedCount; ++i) if (hub.pacedEvents[i] == event) return;
    if (hub.pacedCount < 8) hub.pacedEvents[hub.pacedCount++] = event;
}
void forgetPacedEvent(HANDLE event) {
    if (!event) return;
    auto& hub = engine();
    std::lock_guard lock(hub.eventMutex);
    for (int i = 0; i < hub.pacedCount; ++i) if (hub.pacedEvents[i] == event) {
        hub.pacedEvents[i] = hub.pacedEvents[--hub.pacedCount];
        return;
    }
}
void workerLoop() {
    auto& hub = engine();
    const long long pulse = (std::max)(1LL, qpcHz() * 32 / 3000);
    long long nextPulse = 0;
    while (hub.run.load(std::memory_order_acquire)) {
        Sleep(2);
        Stall stall("haptic event");
        logQueuedSpeakerDump();
        publishStats();
        const long long now = qpc();
        if (nextPulse && now < nextPulse) continue;
        nextPulse = now + pulse;
        HANDLE copy[8]{};
        int count = 0;
        {
            std::lock_guard lock(hub.eventMutex);
            count = hub.pacedCount;
            for (int i = 0; i < count; ++i) copy[i] = hub.pacedEvents[i];
        }
        for (int i = 0; i < count; ++i) if (copy[i]) SetEvent(copy[i]);
    }
}
void reportWorkerCrash(unsigned code) {
    if (runtime) runtime->flushCrash("BtHaptics worker exception " + std::to_string(code));
}
void worker() {
    __try { workerLoop(); }
    __except (EXCEPTION_EXECUTE_HANDLER) { reportWorkerCrash(GetExceptionCode()); }
}
void sender() {
    __try { senderLoop(); }
    __except (EXCEPTION_EXECUTE_HANDLER) { reportWorkerCrash(GetExceptionCode()); }
}
void ensureWorker() {
    startStallWatch();
    static std::atomic<bool> started{false};
    if (started.exchange(true)) return;
    std::thread(worker).detach();
    std::thread(sender).detach();
}
class Client;
struct Stream {
    std::mutex mutex;
    HapticDownsampler downsampler;
    std::vector<std::uint8_t> scratch;
    WAVEFORMATEX format{};
    UINT32 bufferFrames = 480;
    UINT32 pending = 0;
    UINT32 outstanding = 0;
    UINT64 played = 0;
    UINT64 submitted = 0;
    long long startQpc = 0;
    long long born = 0;
    int earlyLogs = 0;
    int getBufFail = 0;
    bool started = false;
    bool haptic = false;
    bool floatSamples = false;
    bool dumped = false;
    int id = 0;
    DWORD flags = 0;
    HANDLE event = nullptr;
    long long lastDrain = 0;
    void* caller = nullptr;
    std::string route = "other";
    int getBuffers = 0;
    UINT64 gotFrames = 0;
    float chPeak[4]{};
    double chSq[4]{};
    int chNz[4]{};
    long long statFrames = 0;
    long long lastClientLog = 0;
    void drain() {
        if (!started) return;
        if (!startQpc) startQpc = qpc();
        const unsigned rate = format.nSamplesPerSec ? format.nSamplesPerSec : 48000;
        const long long elapsed = (qpc() - startQpc) * static_cast<long long>(rate) / qpcHz();
        const UINT64 clock = elapsed > 0 ? static_cast<UINT64>(elapsed) : 0;
        const UINT64 consumed = clock < submitted ? clock : submitted;
        played = consumed;
        const UINT64 queued = submitted - consumed;
        pending = queued > bufferFrames ? bufferFrames : static_cast<UINT32>(queued);
    }
};
void traceStream(Stream* stream, const std::string& line) {
    if (!stream) return;
    if (!stream->born) stream->born = qpc();
    const bool early = stream->earlyLogs < 160 && qpc() - stream->born < qpcHz() * 3;
    if (!early) return;
    ++stream->earlyLogs;
    trace("tid=" + std::to_string(GetCurrentThreadId()) + " client=" + std::to_string(stream->id) + " route=" + stream->route + " " + line);
}
class RenderClient final : public IAudioRenderClient {
    Client* owner = nullptr;
    Stream* stream = nullptr;
    long refs = 1;
public:
    RenderClient(Client* client, Stream* data) : owner(client), stream(data) {}
    ULONG STDMETHODCALLTYPE Release() override;
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioRenderClient)) { *out = this; AddRef(); return S_OK; }
        noteUnknown("IAudioRenderClient", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    HRESULT STDMETHODCALLTYPE GetBuffer(UINT32 frames, BYTE** data) override {
        Stall stall("WASAPI GetBuffer");
        if (!data || !stream) return E_POINTER;
        std::lock_guard lock(stream->mutex);
        if (!stream->started) { traceStream(stream, "GetBuffer rejected not-started frames=" + std::to_string(frames)); return AUDCLNT_E_NOT_INITIALIZED; }
        if (stream->outstanding) { traceStream(stream, "GetBuffer rejected out-of-order frames=" + std::to_string(frames)); return AUDCLNT_E_OUT_OF_ORDER; }
        stream->drain();
        if (frames > stream->bufferFrames - stream->pending) {
            ++stream->getBufFail;
            traceStream(stream, "GetBuffer rejected full frames=" + std::to_string(frames) + " padding=" + std::to_string(stream->pending) + " size=" + std::to_string(stream->bufferFrames) + " hr=88890005");
            return AUDCLNT_E_BUFFER_TOO_LARGE;
        }
        const unsigned payload = frames * kHapticFrameBytes;
        const unsigned total = renderBufferBytes(frames);
        stream->scratch.assign(total, 0);
        paintBufferGuard(stream->scratch.data(), payload * 2, total);
        stream->outstanding = frames;
        stream->getBuffers += 1;
        stream->gotFrames += frames;
        *data = stream->scratch.data();
        traceStream(stream, "GetBuffer frames=" + std::to_string(frames) + " padding=" + std::to_string(stream->pending) + " hr=0");
        if (noteHot("GetBuffer")) trace("IAudioRenderClient::GetBuffer client=" + std::to_string(stream->id) + " route=" + stream->route + " frames=" + std::to_string(frames));
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE ReleaseBuffer(UINT32 frames, DWORD flags) override;
};
std::int16_t readSample(const std::uint8_t* frame, unsigned offset) {
    std::int16_t value = 0;
    std::memcpy(&value, frame + offset, sizeof value);
    return value;
}
int peakOf(int value) { return std::abs(value); }
int channelSample(const std::uint8_t* frame, int index, bool floating) {
    if (floating) {
        float value = 0;
        std::memcpy(&value, frame + static_cast<std::size_t>(index) * 4, sizeof value);
        if (!std::isfinite(value)) return 0;
        value = std::clamp(value, -1.f, 1.f);
        return static_cast<int>(std::lround(value * 32767.f));
    }
    return readSample(frame, static_cast<unsigned>(index) * 2);
}
void publishClient(Stream* stream) {
    if (!stream) return;
    const long long now = qpc();
    if (stream->lastClientLog && now - stream->lastClientLog < qpcHz()) return;
    stream->lastClientLog = now;
    auto num = [](double value) { char text[32]{}; sprintf_s(text, "%.6g", value); return std::string(text); };
    const long long frames = stream->statFrames;
    std::string line = "client=" + std::to_string(stream->id) + " route=" + stream->route + " haptic=" + (stream->haptic ? "1" : "0")
        + " padding=" + std::to_string(stream->pending) + " played=" + std::to_string(stream->played) + " getBuf=" + std::to_string(stream->getBuffers) + " getBufFail=" + std::to_string(stream->getBufFail) + " frames=" + std::to_string(stream->gotFrames);
    for (int channel = 0; channel < 4; ++channel) {
        const double rms = frames ? std::sqrt(stream->chSq[channel] / static_cast<double>(frames)) : 0;
        line += " ch" + std::to_string(channel + 1) + " peak=" + num(stream->chPeak[channel]) + " rms=" + num(rms) + " nz=" + std::to_string(stream->chNz[channel]);
        stream->chPeak[channel] = 0;
        stream->chSq[channel] = 0;
        stream->chNz[channel] = 0;
    }
    stream->statFrames = 0;
    stream->getBuffers = 0;
    stream->gotFrames = 0;
    stream->getBufFail = 0;
    trace(line);
}
HRESULT STDMETHODCALLTYPE RenderClient::ReleaseBuffer(UINT32 frames, DWORD flags) {
    Stall stall("WASAPI ReleaseBuffer");
    if (!stream) return AUDCLNT_E_NOT_INITIALIZED;
    std::lock_guard lock(stream->mutex);
    if (!stream->outstanding || frames > stream->outstanding) return AUDCLNT_E_INVALID_SIZE;
    const bool silent = (flags & AUDCLNT_BUFFERFLAGS_SILENT) != 0;
    const unsigned payload = frames * kHapticFrameBytes;
    const unsigned total = renderBufferBytes(frames);
        bool feed = false;
        if (stream->haptic) {
            int owner = engine().hapticOwner.load(std::memory_order_acquire);
            if (owner == 0 || owner == stream->id) {
                engine().hapticOwner.store(stream->id, std::memory_order_release);
                feed = true;
            }
        }
    if (stream->scratch.size() < total || !bufferGuardIntact(stream->scratch.data(), payload * 2, total))
        trace("buffer guard corrupted client=" + std::to_string(stream->id) + " frames=" + std::to_string(frames) + " bytes=" + std::to_string(stream->scratch.size()));
    int local12 = 0, local34 = 0;
    long long sq12 = 0, sq34 = 0;
    if (!silent && stream->scratch.size() >= payload) {
        const UINT32 dumped = std::min(frames, 32u);
        const bool first = feed && !stream->dumped;
        if (first) stream->dumped = true;
        if (!stream->floatSamples && stream->scratch.size() >= static_cast<std::size_t>(frames) * 16) {
            int floatHits = 0;
            const UINT32 looked = std::min(frames, 16u);
            for (UINT32 frame = 0; frame < looked; ++frame) {
                const auto* bytes = stream->scratch.data() + static_cast<std::size_t>(frame) * 16;
                float value = 0;
                std::memcpy(&value, bytes, sizeof value);
                const std::uint8_t exponent = bytes[3];
                if (((exponent >= 0x30 && exponent <= 0x4f) || (exponent >= 0xb0 && exponent <= 0xcf)) && std::isfinite(value) && std::fabs(value) < 8.f) ++floatHits;
            }
            if (floatHits >= 2) {
                stream->floatSamples = true;
                trace("pcm16 request carried float32 frames; decoding float");
            }
        }
        const bool floating = stream->floatSamples;
        const unsigned stride = floating ? 16u : 8u;
        auto readChannel = [&](const std::uint8_t* frame, int index) {
            if (floating) {
                float value = 0;
                std::memcpy(&value, frame + static_cast<std::size_t>(index) * 4, sizeof value);
                return std::isfinite(value) ? value : 0.f;
            }
            std::int16_t value = 0;
            std::memcpy(&value, frame + static_cast<std::size_t>(index) * 2, sizeof value);
            return static_cast<float>(value) / 32768.f;
        };
        float peakF[4]{};
        double sqF[4]{};
        float peak12 = 0, peak34 = 0;
        for (UINT32 frame = 0; frame < frames; ++frame) {
            const auto* bytes = stream->scratch.data() + static_cast<std::size_t>(frame) * stride;
            float sample[4]{};
            for (int channel = 0; channel < 4; ++channel) {
                sample[channel] = readChannel(bytes, channel);
                const float absValue = std::fabs(sample[channel]);
                stream->chPeak[channel] = std::max(stream->chPeak[channel], absValue);
                stream->chSq[channel] += static_cast<double>(sample[channel]) * sample[channel];
                if (absValue > 1e-6f) ++stream->chNz[channel];
                if (frame < dumped) { peakF[channel] = std::max(peakF[channel], absValue); sqF[channel] += static_cast<double>(sample[channel]) * sample[channel]; }
            }
            peak12 = std::max(peak12, std::max(std::fabs(sample[0]), std::fabs(sample[1])));
            peak34 = std::max(peak34, std::max(std::fabs(sample[2]), std::fabs(sample[3])));
        }
        if (stream->haptic && peak34 > 1e-6f) {
            engine().hapticOwner.store(stream->id, std::memory_order_release);
            engine().hapticRunning.store(true, std::memory_order_release);
            feed = true;
        }
        const auto settings = runtime ? runtime->snapshot() : Settings{};
        const int pair = hapticChannelPair(settings.BtHapticsChannelPair, peak12, peak34);
        selectedPair.store(pair, std::memory_order_relaxed);
        const float gain = static_cast<float>(std::clamp(settings.BtHapticsGain, 1, 16));
        const int leftIndex = pair == 12 ? 0 : 2;
        const int rightIndex = leftIndex + 1;
        if (feed && pair) {
            for (UINT32 frame = 0; frame < frames; ++frame) {
                const auto* bytes = stream->scratch.data() + static_cast<std::size_t>(frame) * stride;
                const float left = readChannel(bytes, leftIndex);
                const float right = readChannel(bytes, rightIndex);
                std::int8_t outLeft = 0, outRight = 0;
                if (stream->downsampler.push(floatToPcm16(left * gain), floatToPcm16(right * gain), outLeft, outRight)) enqueueHapticSample(outLeft, outRight);
            }
        }
        auto micro = [](float value) { return static_cast<int>(std::llround(std::fabs(value) * 1000000.0)); };
        auto microSq = [](float value) { const long long scaled = std::llround(std::fabs(value) * 1000000.0); return scaled * scaled; };
        local12 = micro(peak12);
        local34 = micro(peak34);
        for (UINT32 frame = 0; frame < frames; ++frame) {
            const auto* bytes = stream->scratch.data() + static_cast<std::size_t>(frame) * stride;
            float sample[4]{};
            for (int channel = 0; channel < 4; ++channel) sample[channel] = readChannel(bytes, channel);
            sq12 += microSq(sample[0]) + microSq(sample[1]);
            sq34 += microSq(sample[2]) + microSq(sample[3]);
        }
        if (first) {
            auto num = [](double value) { char text[32]{}; sprintf_s(text, "%.6g", value); return std::string(text); };
            trace("first buffer client=" + std::to_string(stream->id) + " frames=" + std::to_string(dumped) + " stride=" + std::to_string(stride) + " float=" + (floating ? "1" : "0") + " hex=" + hexBytes(stream->scratch.data(), dumped * stride));
            float signedSample[4]{};
            for (int channel = 0; channel < 4; ++channel) signedSample[channel] = readChannel(stream->scratch.data(), channel);
            std::string levels = "first levels";
            for (int channel = 0; channel < 4; ++channel) levels += " ch" + std::to_string(channel + 1) + "=" + num(signedSample[channel]) + " peak=" + num(peakF[channel]) + " rms=" + num(dumped ? std::sqrt(sqF[channel] / dumped) : 0);
            trace(levels + " sending=ch" + std::to_string(pair ? pair : 0));
        }
        const bool hot12 = peak12 > 1e-6f, hot34 = peak34 > 1e-6f;
        if (feed && hot12 != hot34) {
            static long long lastPair = 0;
            const long long now = qpc();
            if (!lastPair || now - lastPair >= qpcHz()) {
                lastPair = now;
                trace(pair == 34 ? "signal pair=34" : pair == 12 ? "signal pair=12 speaker fallback" : "actuators silent, not sending speaker");
            }
        }
    }
    stream->statFrames += frames;
    if (feed) {
        if (silent) ++silentBuffers; else ++liveBuffers;
        pcmFrames += frames; sumSq12 += sq12; sumSq34 += sq34;
        raisePeak(peak12, local12); raisePeak(peak34, local34);
    } else if (!silent) {
        ++noopLive; raisePeak(noopPeak12, local12); raisePeak(noopPeak34, local34);
    }
    stream->submitted += frames;
    stream->drain();
    stream->outstanding = 0;
    const auto speakerSettings = runtime ? runtime->snapshot() : Settings{};
    if (stream->route.rfind("speaker", 0) == 0 && runtime && speakerSettings.BtSpeaker && speakerSettings.BtSpeakerReport) {
        const float gain = static_cast<float>(std::clamp(speakerSettings.BtSpeakerGain, 0, 8));
        const bool floating = stream->floatSamples;
        const unsigned stride = floating ? 16u : 8u;
        float peak = 0;
        double square = 0;
        std::vector<std::int16_t> pcm(static_cast<std::size_t>(frames) * 2);
        for (UINT32 frame = 0; frame < frames; ++frame) {
            float left = 0, right = 0;
            if (!silent && stream->scratch.size() >= static_cast<std::size_t>(frame + 1) * stride) {
                const auto* bytes = stream->scratch.data() + static_cast<std::size_t>(frame) * stride;
                auto read = [&](int index) {
                    if (floating) {
                        float value = 0;
                        std::memcpy(&value, bytes + static_cast<std::size_t>(index) * 4, sizeof value);
                        return std::isfinite(value) ? value : 0.f;
                    }
                    std::int16_t value = 0;
                    std::memcpy(&value, bytes + static_cast<std::size_t>(index) * 2, sizeof value);
                    return static_cast<float>(value) / 32768.f;
                };
                left = read(0) * gain;
                right = read(1) * gain;
            }
            peak = std::max(peak, std::max(std::fabs(left), std::fabs(right)));
            auto microSq = [](float value) {
                const long long scaled = std::llround(std::fabs(value) * 1000000.0);
                return scaled * scaled;
            };
            square += static_cast<double>(microSq(left) + microSq(right));
            pcm[static_cast<std::size_t>(frame) * 2] = floatToPcm16(left);
            pcm[static_cast<std::size_t>(frame) * 2 + 1] = floatToPcm16(right);
        }
        auto& hub = engine();
        std::lock_guard lock(hub.mutex);
        for (UINT32 frame = 0; frame < frames; ++frame) {
            if (hub.spkCount == Engine::kSpeakerRing) {
                hub.spkHead = (hub.spkHead + 1) % Engine::kSpeakerRing;
                --hub.spkCount;
                ++speakerDrops;
            }
            hub.speaker[hub.spkTail] = {pcm[static_cast<std::size_t>(frame) * 2], pcm[static_cast<std::size_t>(frame) * 2 + 1]};
            hub.spkTail = (hub.spkTail + 1) % Engine::kSpeakerRing;
            ++hub.spkCount;
        }
        speakerDepth.store(hub.spkCount, std::memory_order_relaxed);
        raisePeak(speakerPeak, static_cast<int>(std::llround(peak * 1000000.0)));
        speakerSq.fetch_add(static_cast<long long>(std::llround(square)), std::memory_order_relaxed);
        speakerFrames.fetch_add(static_cast<int>(frames), std::memory_order_relaxed);
    }
    traceStream(stream, "ReleaseBuffer frames=" + std::to_string(frames) + " flags=" + std::to_string(flags) + " padding=" + std::to_string(stream->pending) + " hr=0");
    publishClient(stream);
    if (noteHot("ReleaseBuffer")) trace("IAudioRenderClient::ReleaseBuffer client=" + std::to_string(stream->id) + " route=" + stream->route + " haptic=" + (stream->haptic ? "1" : "0") + " frames=" + std::to_string(frames) + " flags=" + std::to_string(flags));
    return S_OK;
}
class Clock final : public IAudioClock {
    Client* owner = nullptr;
    Stream* stream = nullptr;
    long refs = 1;
public:
    Clock(Client* client, Stream* data) : owner(client), stream(data) {}
    ULONG STDMETHODCALLTYPE Release() override;
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioClock)) { *out = this; AddRef(); return S_OK; }
        noteUnknown("IAudioClock", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    HRESULT STDMETHODCALLTYPE GetFrequency(UINT64* frequency) override { if (!frequency) return E_POINTER; *frequency = stream && stream->format.nSamplesPerSec ? stream->format.nSamplesPerSec : 48000; return S_OK; }
    HRESULT STDMETHODCALLTYPE GetPosition(UINT64* position, UINT64* qpcPosition) override {
        if (!position || !stream) return E_POINTER;
        std::lock_guard lock(stream->mutex);
        stream->drain();
        *position = stream->played;
        if (qpcPosition) *qpcPosition = static_cast<UINT64>(qpc());
        traceStream(stream, "GetPosition frames=" + std::to_string(stream->played) + " padding=" + std::to_string(stream->pending) + " hr=0");
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetCharacteristics(DWORD* characteristics) override { if (!characteristics) return E_POINTER; *characteristics = 0; return S_OK; }
};
class Clock2 final : public IAudioClock2 {
    Client* owner = nullptr;
    Stream* stream = nullptr;
    long refs = 1;
public:
    Clock2(Client* client, Stream* data) : owner(client), stream(data) {}
    ULONG STDMETHODCALLTYPE Release() override;
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioClock2)) { *out = this; AddRef(); return S_OK; }
        noteUnknown("IAudioClock2", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    HRESULT STDMETHODCALLTYPE GetDevicePosition(UINT64* position, UINT64* qpcPosition) override {
        if (!position || !stream) return E_POINTER;
        std::lock_guard lock(stream->mutex);
        stream->drain();
        *position = stream->played;
        if (qpcPosition) *qpcPosition = static_cast<UINT64>(qpc());
        traceStream(stream, "GetDevicePosition frames=" + std::to_string(stream->played) + " hr=0");
        return S_OK;
    }
};
class ClockRate final : public IAudioClockAdjustment {
    long refs = 1;
public:
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioClockAdjustment)) { *out = this; AddRef(); return S_OK; }
        noteUnknown("IAudioClockAdjustment", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { return InterlockedDecrement(&refs); }
    HRESULT STDMETHODCALLTYPE SetSampleRate(float rate) override { traceHr("IAudioClockAdjustment::SetSampleRate " + std::to_string(rate), S_OK); return S_OK; }
};
ClockRate& clockRate() { static ClockRate instance; return instance; }
class PropertyStore final : public IPropertyStore {
    long refs = 1;
public:
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IPropertyStore)) { *out = this; AddRef(); return S_OK; }
        noteUnknown("IPropertyStore", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { return InterlockedDecrement(&refs); }
    HRESULT STDMETHODCALLTYPE GetCount(DWORD* count) override { if (!count) return E_POINTER; *count = 1; return S_OK; }
    HRESULT STDMETHODCALLTYPE GetAt(DWORD index, PROPERTYKEY* key) override {
        if (!key) return E_POINTER;
        if (index != 0) return E_INVALIDARG;
        *key = kEngineFormat;
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetValue(REFPROPERTYKEY key, PROPVARIANT* value) override {
        if (!value) return E_POINTER;
        PropVariantInit(value);
        if (sameKey(key, kEngineFormat)) {
            if (!setBlob(value)) return E_OUTOFMEMORY;
            static bool logged = false;
            if (!logged) { logged = true; traceHr("IPropertyStore::GetValue DeviceFormat", S_OK); }
            return S_OK;
        }
        if (sameKey(key, kFriendlyName)) {
            const wchar_t* name = padFriendlyName();
            if (!setWide(value, name)) return E_OUTOFMEMORY;
            static bool logged = false;
            if (!logged) { logged = true; trace("IPropertyStore::GetValue FriendlyName " + wideText(name)); }
            return S_OK;
        }
        if (sameKey(key, kInterfaceName)) {
            const wchar_t* name = padInterfaceName();
            if (!setWide(value, name)) return E_OUTOFMEMORY;
            static bool logged = false;
            if (!logged) { logged = true; trace("IPropertyStore::GetValue InterfaceName " + wideText(name)); }
            return S_OK;
        }
        if (sameKey(key, kDeviceDesc)) {
            if (!setWide(value, L"Speakers")) return E_OUTOFMEMORY;
            static bool logged = false;
            if (!logged) { logged = true; trace("IPropertyStore::GetValue DeviceDesc Speakers"); }
            return S_OK;
        }
        if (sameKey(key, kFormFactor)) {
            value->vt = VT_UI4;
            value->ulVal = 1;
            static bool logged = false;
            if (!logged) { logged = true; traceHr("IPropertyStore::GetValue FormFactor Speakers", S_OK); }
            return S_OK;
        }
        if (sameKey(key, kContainer)) {
            GUID id{};
            if (controllerContainer(id)) {
                auto* copy = static_cast<GUID*>(CoTaskMemAlloc(sizeof(GUID)));
                if (!copy) return E_OUTOFMEMORY;
                *copy = id;
                value->vt = VT_CLSID;
                value->puuid = copy;
                static bool logged = false;
                if (!logged) { logged = true; traceHr("IPropertyStore::GetValue ContainerId", S_OK); }
                return S_OK;
            }
        }
        static std::mutex missingGuard;
        static std::unordered_set<std::string> missing;
        const std::string id = iidText(key.fmtid) + ":" + std::to_string(key.pid);
        std::lock_guard missingLock(missingGuard);
        if (missing.insert(id).second && missing.size() <= 64)
            trace("IPropertyStore::GetValue missing fmtid=" + iidText(key.fmtid) + " pid=" + std::to_string(key.pid) + " hr=80070490");
        return HRESULT_FROM_WIN32(ERROR_NOT_FOUND);
    }
    HRESULT STDMETHODCALLTYPE SetValue(REFPROPERTYKEY, REFPROPVARIANT) override { traceHr("IPropertyStore::SetValue", STG_E_ACCESSDENIED); return STG_E_ACCESSDENIED; }
    HRESULT STDMETHODCALLTYPE Commit() override { traceHr("IPropertyStore::Commit", S_OK); return S_OK; }
};
PropertyStore& store() { static PropertyStore instance; return instance; }
class Client final : public IAudioClient3 {
public:
    Stream stream;
private:
    long refs = 1;
    void retire() {
        if (stream.event) forgetPacedEvent(stream.event);
        if (stream.route.rfind("speaker", 0) == 0) setSpeakerStreamActive(false);
        if (!stream.haptic) return;
        int id = stream.id;
        if (engine().hapticOwner.compare_exchange_strong(id, 0, std::memory_order_acq_rel)) {
            engine().hapticRunning.store(false, std::memory_order_release);
            clearHapticRing();
        }
        stream.haptic = false;
    }
public:
    Client() {
        static std::atomic<int> next{1};
        stream.id = next.fetch_add(1);
        stream.caller = _ReturnAddress();
    }
    ~Client() { retire(); }
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioClient) || id == __uuidof(IAudioClient2) || id == __uuidof(IAudioClient3)) { *out = this; AddRef(); trace("IAudioClient::QueryInterface " + iidText(id) + " hr=00000000"); return S_OK; }
        if (id == __uuidof(IAudioClock)) { auto* clock = new Clock(this, &stream); AddRef(); *out = clock; traceStream(&stream, "QueryInterface IAudioClock hr=0"); return S_OK; }
        if (id == __uuidof(IAudioClock2)) { auto* clock = new Clock2(this, &stream); AddRef(); *out = clock; traceStream(&stream, "QueryInterface IAudioClock2 hr=0"); return S_OK; }
        if (id == __uuidof(IAudioClockAdjustment)) { *out = &clockRate(); clockRate().AddRef(); traceStream(&stream, "QueryInterface IAudioClockAdjustment hr=0"); return S_OK; }
        traceStream(&stream, "QueryInterface " + iidText(id) + " hr=80004002");
        noteUnknown("IAudioClient", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { auto left = InterlockedDecrement(&refs); if (!left) delete this; return left; }
    HRESULT STDMETHODCALLTYPE Initialize(AUDCLNT_SHAREMODE share, DWORD flags, REFERENCE_TIME duration, REFERENCE_TIME, const WAVEFORMATEX* format, LPCGUID) override {
        Stall stall("WASAPI Initialize");
        const bool extensible = format && format->wFormatTag == WAVE_FORMAT_EXTENSIBLE && format->cbSize >= 22;
        const auto* ext = extensible ? reinterpret_cast<const WAVEFORMATEXTENSIBLE*>(format) : nullptr;
        const bool floatFormat = format && format->nChannels == 4 && format->wBitsPerSample == 32 && format->nBlockAlign >= 16 && ((format->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) || (ext && IsEqualGUID(ext->SubFormat, kFloatSubformat)));
        const bool pcm16 = format && format->nChannels == 4 && format->wBitsPerSample == 16 && format->nBlockAlign >= 8 && format->nSamplesPerSec != 0 && !floatFormat;
        std::string described = "missing";
        if (format) {
            described = "tag=" + std::to_string(format->wFormatTag) + " ch=" + std::to_string(format->nChannels) + " rate=" + std::to_string(format->nSamplesPerSec) + " bits=" + std::to_string(format->wBitsPerSample) + " align=" + std::to_string(format->nBlockAlign) + " cb=" + std::to_string(format->cbSize);
            if (ext) described += " validBits=" + std::to_string(ext->Samples.wValidBitsPerSample) + " mask=" + std::to_string(ext->dwChannelMask) + " sub=" + iidText(ext->SubFormat);
            described += floatFormat ? " decode=float32" : pcm16 ? " decode=int16" : " decode=unsupported";
        }
        trace("IAudioClient::Initialize requested " + described);
        if (!format || format->nChannels != 4 || !format->nSamplesPerSec) return AUDCLNT_E_UNSUPPORTED_FORMAT;
        const bool decodeFloat = floatFormat || (!pcm16 && format->wBitsPerSample >= 32);
        trace(std::string("IAudioClient::Initialize reading ") + (decodeFloat ? "float32 align=16 validBits=32 mask=33" : "pcm16 align=8 validBits=16 mask=33"));
        {
            std::lock_guard formatLock(formatMutex);
            initFormat = described;
        }
        std::lock_guard lock(stream.mutex);
        stream.floatSamples = decodeFloat;
        stream.flags = flags;
        stream.format = *format;
        stream.format.wBitsPerSample = decodeFloat ? 32 : 16;
        stream.format.nBlockAlign = decodeFloat ? kHapticFrameBytes : 8;
        stream.format.cbSize = 0;
        const long long rounded = (static_cast<long long>(duration) * stream.format.nSamplesPerSec + 5000000) / 10000000;
        UINT32 frames = static_cast<UINT32>(rounded > 0 ? rounded : 0);
        if (frames < 480) frames = 480;
        if (frames > 48000) frames = 48000;
        stream.bufferFrames = frames;
        stream.pending = stream.outstanding = 0;
        stream.submitted = stream.played = 0;
        stream.startQpc = 0;
        stream.started = false;
        ensureWorker();
        traceStream(&stream, "Initialize flags=" + std::to_string(flags) + " duration=" + std::to_string(duration) + " frames=" + std::to_string(frames) + " " + described + " hr=0");
        trace("IAudioClient::Initialize client=" + std::to_string(stream.id) + " route=" + stream.route + " haptic=" + (stream.haptic ? "1" : "0") + " caller=" + callerText(_ReturnAddress()) + " activate=" + callerText(stream.caller) + " flags=" + std::to_string(flags) + " frames=" + std::to_string(frames));
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetBufferSize(UINT32* frames) override { if (!frames) return E_POINTER; *frames = stream.bufferFrames; return S_OK; }
    HRESULT STDMETHODCALLTYPE GetStreamLatency(REFERENCE_TIME* latency) override { if (!latency) return E_POINTER; *latency = 100000; return S_OK; }
    HRESULT STDMETHODCALLTYPE GetCurrentPadding(UINT32* padding) override {
        if (!padding) return E_POINTER;
        std::lock_guard lock(stream.mutex);
        stream.drain();
        *padding = stream.pending;
        traceStream(&stream, "GetCurrentPadding padding=" + std::to_string(*padding) + " played=" + std::to_string(stream.played) + " hr=0");
        publishClient(&stream);
        if (noteHot("GetCurrentPadding")) trace("IAudioClient::GetCurrentPadding client=" + std::to_string(stream.id) + " route=" + stream.route + " padding=" + std::to_string(*padding));
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE IsFormatSupported(AUDCLNT_SHAREMODE, const WAVEFORMATEX* format, WAVEFORMATEX** closest) override {
        if (mixFormatMatches(format)) { trace("IAudioClient::IsFormatSupported accepted " + describeFormat(format)); return S_OK; }
        if (closest) {
            auto* mix = static_cast<WAVEFORMATEXTENSIBLE*>(CoTaskMemAlloc(sizeof(WAVEFORMATEXTENSIBLE)));
            if (mix) { *mix = floatMixFormat(); *closest = &mix->Format; }
        }
        trace("IAudioClient::IsFormatSupported closest float32 for " + describeFormat(format));
        return closest && *closest ? S_FALSE : AUDCLNT_E_UNSUPPORTED_FORMAT;
    }
    HRESULT STDMETHODCALLTYPE GetMixFormat(WAVEFORMATEX** format) override {
        if (!format) return E_POINTER;
        auto* mix = static_cast<WAVEFORMATEXTENSIBLE*>(CoTaskMemAlloc(sizeof(WAVEFORMATEXTENSIBLE)));
        if (!mix) return E_OUTOFMEMORY;
        *mix = floatMixFormat();
        *format = &mix->Format;
        trace("IAudioClient::GetMixFormat returned " + describeFormat(&mix->Format));
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetDevicePeriod(REFERENCE_TIME* def, REFERENCE_TIME* minimum) override {
        if (def) *def = 100000;
        if (minimum) *minimum = 30000;
        traceHr("IAudioClient::GetDevicePeriod default=10ms min=3ms", S_OK);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE Start() override {
        Stall stall("WASAPI Start");
        std::lock_guard lock(stream.mutex);
        stream.started = true;
        stream.startQpc = qpc();
        stream.submitted = stream.played = 0;
        stream.pending = 0;
        stream.lastDrain = stream.startQpc;
        if (stream.haptic && engine().hapticOwner.load(std::memory_order_acquire) == stream.id) engine().hapticRunning.store(true, std::memory_order_release);
        if (stream.route.rfind("speaker", 0) == 0) setSpeakerStreamActive(true);
        if (stream.event) SetEvent(stream.event);
        traceStream(&stream, "Start hr=0");
        trace("IAudioClient::Start client=" + std::to_string(stream.id) + " route=" + stream.route + " haptic=" + (stream.haptic ? "1" : "0"));
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE Stop() override { stream.started = false; if (stream.route.rfind("speaker", 0) == 0) setSpeakerStreamActive(false); if (engine().hapticOwner.load(std::memory_order_acquire) == stream.id) { engine().hapticRunning.store(false, std::memory_order_release); clearHapticRing(); } trace("IAudioClient::Stop client=" + std::to_string(stream.id)); return S_OK; }
    HRESULT STDMETHODCALLTYPE Reset() override { std::lock_guard lock(stream.mutex); stream.pending = stream.outstanding = 0; stream.submitted = stream.played = 0; stream.startQpc = 0; stream.started = false; traceStream(&stream, "Reset hr=0"); if (engine().hapticOwner.load(std::memory_order_acquire) == stream.id) { engine().hapticRunning.store(false, std::memory_order_release); clearHapticRing(); } return S_OK; }
    HRESULT STDMETHODCALLTYPE SetEventHandle(HANDLE handle) override {
        if (stream.event && stream.event != handle) forgetPacedEvent(stream.event);
        stream.event = handle;
        if (handle) watchPacedEvent(handle);
        trace("IAudioClient::SetEventHandle client=" + std::to_string(stream.id) + " route=" + stream.route + (handle ? " set" : " null"));
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetService(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IAudioRenderClient)) { auto* render = new RenderClient(this, &stream); AddRef(); *out = render; traceStream(&stream, "GetService IAudioRenderClient hr=0"); return S_OK; }
        if (id == __uuidof(IAudioClock)) { auto* clock = new Clock(this, &stream); AddRef(); *out = clock; traceStream(&stream, "GetService IAudioClock hr=0"); return S_OK; }
        if (id == __uuidof(IAudioClock2)) { auto* clock = new Clock2(this, &stream); AddRef(); *out = clock; traceStream(&stream, "GetService IAudioClock2 hr=0"); return S_OK; }
        if (id == __uuidof(IAudioClockAdjustment)) { *out = &clockRate(); clockRate().AddRef(); traceStream(&stream, "GetService IAudioClockAdjustment hr=0"); return S_OK; }
        traceStream(&stream, "GetService " + iidText(id) + " hr=80004002");
        noteUnknown("IAudioClient::GetService", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    HRESULT STDMETHODCALLTYPE IsOffloadCapable(AUDIO_STREAM_CATEGORY, BOOL* capable) override { if (capable) *capable = FALSE; traceHr("IAudioClient2::IsOffloadCapable", S_OK); return S_OK; }
    HRESULT STDMETHODCALLTYPE SetClientProperties(const AudioClientProperties* properties) override { traceHr(std::string("IAudioClient2::SetClientProperties ") + (properties ? "set" : "null"), S_OK); return S_OK; }
    HRESULT STDMETHODCALLTYPE GetBufferSizeLimits(const WAVEFORMATEX*, BOOL eventDriven, REFERENCE_TIME* minimum, REFERENCE_TIME* maximum) override {
        if (minimum) *minimum = 100000;
        if (maximum) *maximum = 1000000;
        traceHr(std::string("IAudioClient2::GetBufferSizeLimits event=") + (eventDriven ? "1" : "0"), S_OK);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetSharedModeEnginePeriod(const WAVEFORMATEX*, UINT32* def, UINT32* fundamental, UINT32* minimum, UINT32* maximum) override {
        if (def) *def = 480;
        if (fundamental) *fundamental = 480;
        if (minimum) *minimum = 480;
        if (maximum) *maximum = 4800;
        traceHr("IAudioClient3::GetSharedModeEnginePeriod 480", S_OK);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetCurrentSharedModeEnginePeriod(WAVEFORMATEX** format, UINT32* frames) override {
        if (frames) *frames = 480;
        if (format) {
            auto* mix = static_cast<WAVEFORMATEXTENSIBLE*>(CoTaskMemAlloc(sizeof(WAVEFORMATEXTENSIBLE)));
            if (!mix) return E_OUTOFMEMORY;
            *mix = floatMixFormat();
            *format = &mix->Format;
        }
        traceHr("IAudioClient3::GetCurrentSharedModeEnginePeriod", S_OK);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE InitializeSharedAudioStream(DWORD flags, UINT32 period, const WAVEFORMATEX* format, LPCGUID session) override {
        trace("IAudioClient3::InitializeSharedAudioStream flags=" + std::to_string(flags) + " period=" + std::to_string(period));
        return Initialize(AUDCLNT_SHAREMODE_SHARED, flags, period ? static_cast<REFERENCE_TIME>(period) * 10000000 / 48000 : 100000, 0, format, session);
    }
};
ULONG STDMETHODCALLTYPE RenderClient::Release() {
    auto left = InterlockedDecrement(&refs);
    if (!left) { auto* kept = owner; delete this; if (kept) kept->Release(); }
    return left;
}
ULONG STDMETHODCALLTYPE Clock2::Release() {
    auto left = InterlockedDecrement(&refs);
    if (!left) { auto* kept = owner; delete this; if (kept) kept->Release(); }
    return left;
}
ULONG STDMETHODCALLTYPE Clock::Release() {
    auto left = InterlockedDecrement(&refs);
    if (!left) { auto* kept = owner; delete this; if (kept) kept->Release(); }
    return left;
}
class Endpoint final : public IMMEndpoint {
    long refs = 1;
public:
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IMMEndpoint)) { *out = this; AddRef(); traceHr("IMMEndpoint::QueryInterface", S_OK); return S_OK; }
        noteUnknown("IMMEndpoint", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { return InterlockedDecrement(&refs); }
    HRESULT STDMETHODCALLTYPE GetDataFlow(EDataFlow* flow) override { if (!flow) return E_POINTER; *flow = eRender; traceHr("IMMEndpoint::GetDataFlow eRender", S_OK); return S_OK; }
};
Endpoint& endpoint() { static Endpoint instance; return instance; }
class Device final : public IMMDevice {
    long refs = 1;
public:
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IMMDevice)) { *out = this; AddRef(); traceHr("IMMDevice::QueryInterface " + iidText(id), S_OK); return S_OK; }
        if (id == __uuidof(IMMEndpoint)) { *out = &endpoint(); endpoint().AddRef(); traceHr("IMMDevice::QueryInterface IMMEndpoint", S_OK); return S_OK; }
        noteUnknown("IMMDevice", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { return InterlockedDecrement(&refs); }
    HRESULT STDMETHODCALLTYPE Activate(REFIID id, DWORD context, PROPVARIANT*, void** out) override {
        if (!out) return E_POINTER;
        if (id != __uuidof(IAudioClient) && id != __uuidof(IAudioClient2) && id != __uuidof(IAudioClient3) && id != __uuidof(IUnknown)) {
            noteUnknown("IMMDevice::Activate", id);
            *out = nullptr;
            return E_NOINTERFACE;
        }
        auto* created = new Client();
        created->stream.caller = _ReturnAddress();
        const unsigned long long rva = akAudioRva(created->stream.caller);
        if (rva == kContainerSinkRva) {
            created->stream.haptic = true;
            created->stream.route = "container 0x31630A";
            int current = 0;
            engine().hapticOwner.compare_exchange_strong(current, created->stream.id, std::memory_order_acq_rel);
        } else if (rva == kPadSinkRva) {
            created->stream.route = "speaker 0x1D57D3";
        } else created->stream.route = "other " + callerText(created->stream.caller);
        *out = created;
        trace("IMMDevice::Activate client=" + std::to_string(created->stream.id) + " haptic=" + (created->stream.haptic ? "1" : "0") + " route=" + created->stream.route + " caller=" + callerText(created->stream.caller) + " iid=" + iidText(id));
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE OpenPropertyStore(DWORD access, IPropertyStore** properties) override {
        if (!properties) return E_POINTER;
        *properties = &store();
        store().AddRef();
        traceHr("IMMDevice::OpenPropertyStore access=" + std::to_string(access), S_OK);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetId(LPWSTR* id) override {
        if (!id) return E_POINTER;
        const wchar_t name[] = L"{0.0.0.00000000}.{054c0ce6-0df2-4000-8000-4475616c7365}";
        auto* copy = static_cast<wchar_t*>(CoTaskMemAlloc(sizeof name));
        if (!copy) return E_OUTOFMEMORY;
        std::memcpy(copy, name, sizeof name);
        *id = copy;
        traceHr("IMMDevice::GetId", S_OK);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetState(DWORD* state) override { if (!state) return E_POINTER; *state = DEVICE_STATE_ACTIVE; traceHr("IMMDevice::GetState active", S_OK); return S_OK; }
};
Device& device() { static Device instance; return instance; }
const wchar_t kFakeEndpointId[] = L"{0.0.0.00000000}.{054c0ce6-0df2-4000-8000-4475616c7365}";
bool fakeEndpointId(LPCWSTR id) { return id && _wcsicmp(id, kFakeEndpointId) == 0; }
IMMDevice* watchPadDevice(IMMDevice* device);
class CollectionWrap final : public IMMDeviceCollection {
    IMMDeviceCollection* real = nullptr;
    bool extra = false;
    long refs = 1;
public:
    CollectionWrap(IMMDeviceCollection* inner, bool includeFake) : real(inner), extra(includeFake) {}
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IMMDeviceCollection)) { *out = this; AddRef(); return S_OK; }
        noteUnknown("IMMDeviceCollection", id);
        *out = nullptr; return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { auto left = InterlockedDecrement(&refs); if (!left) { if (real) real->Release(); delete this; } return left; }
    HRESULT STDMETHODCALLTYPE GetCount(UINT* count) override {
        if (!count) return E_POINTER;
        UINT inner = 0;
        const HRESULT hr = real ? real->GetCount(&inner) : S_OK;
        if (FAILED(hr)) return hr;
        *count = inner + (extra ? 1u : 0u);
        traceHr("IMMDeviceCollection::GetCount " + std::to_string(*count), S_OK);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE Item(UINT index, IMMDevice** item) override {
        if (!item) return E_POINTER;
        UINT inner = 0;
        if (real) real->GetCount(&inner);
        if (extra && index == inner) { *item = &device(); device().AddRef(); traceHr("IMMDeviceCollection::Item fake", S_OK); return S_OK; }
        const HRESULT hr = real ? real->Item(index, item) : E_INVALIDARG;
        if (SUCCEEDED(hr) && item && *item) *item = watchPadDevice(*item);
        return hr;
    }
};
class EnumeratorWrap final : public IMMDeviceEnumerator {
    IMMDeviceEnumerator* real = nullptr;
    long refs = 1;
public:
    explicit EnumeratorWrap(IMMDeviceEnumerator* inner) : real(inner) {}
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IMMDeviceEnumerator)) { *out = this; AddRef(); return S_OK; }
        noteUnknown("IMMDeviceEnumerator", id);
        return real ? real->QueryInterface(id, out) : E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { auto left = InterlockedDecrement(&refs); if (!left) { if (real) real->Release(); delete this; } return left; }
    HRESULT STDMETHODCALLTYPE EnumAudioEndpoints(EDataFlow flow, DWORD state, IMMDeviceCollection** collection) override {
        if (!collection) return E_POINTER;
        IMMDeviceCollection* inner = nullptr;
        const HRESULT hr = real->EnumAudioEndpoints(flow, state, &inner);
        traceHr("IMMDeviceEnumerator::EnumAudioEndpoints flow=" + std::to_string(flow) + " state=" + std::to_string(state), hr);
        if (FAILED(hr)) return hr;
        const bool include = (flow == eRender || flow == eAll) && (state & DEVICE_STATE_ACTIVE) != 0;
        *collection = new CollectionWrap(inner, include);
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetDefaultAudioEndpoint(EDataFlow flow, ERole role, IMMDevice** endpoint) override {
        const HRESULT hr = real->GetDefaultAudioEndpoint(flow, role, endpoint);
        std::string text = "none";
        if (SUCCEEDED(hr) && endpoint && *endpoint) {
            wchar_t* wide = nullptr;
            if (SUCCEEDED((*endpoint)->GetId(&wide)) && wide) {
                char narrow[300]{};
                WideCharToMultiByte(CP_UTF8, 0, wide, -1, narrow, 280, nullptr, nullptr);
                text = narrow;
                CoTaskMemFree(wide);
            }
        }
        static int noted = 0;
        const bool fake = text.find("054c0ce6-0df2-4000-8000-4475616c7365") != std::string::npos;
        if (noted < 4 || (fake && noted < 8)) { ++noted; trace("default render flow=" + std::to_string(flow) + " id=" + text + (fake ? " FAKE" : " real")); }
        if (SUCCEEDED(hr) && endpoint && *endpoint && !fake) *endpoint = watchPadDevice(*endpoint);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE GetDevice(LPCWSTR id, IMMDevice** deviceOut) override {
        if (fakeEndpointId(id)) {
            if (!deviceOut) return E_POINTER;
            *deviceOut = &device();
            device().AddRef();
            traceHr("IMMDeviceEnumerator::GetDevice fake", S_OK);
            return S_OK;
        }
        const HRESULT hr = real->GetDevice(id, deviceOut);
        static int logged = 0;
        if (logged < 8) { ++logged; traceHr("IMMDeviceEnumerator::GetDevice real", hr); }
        if (SUCCEEDED(hr) && deviceOut && *deviceOut) *deviceOut = watchPadDevice(*deviceOut);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE RegisterEndpointNotificationCallback(IMMNotificationClient* client) override {
        traceHr("IMMDeviceEnumerator::RegisterEndpointNotificationCallback", S_OK);
        return real->RegisterEndpointNotificationCallback(client);
    }
    HRESULT STDMETHODCALLTYPE UnregisterEndpointNotificationCallback(IMMNotificationClient* client) override {
        return real->UnregisterEndpointNotificationCallback(client);
    }
};
using CoCreateFn = HRESULT(WINAPI*)(REFCLSID, LPUNKNOWN, DWORD, REFIID, LPVOID*);
CoCreateFn originalCoCreate{};
HRESULT WINAPI coCreate(REFCLSID clsid, LPUNKNOWN outer, DWORD context, REFIID iid, LPVOID* out) {
    const HRESULT hr = originalCoCreate(clsid, outer, context, iid, out);
    if (FAILED(hr) || outer || !IsEqualGUID(clsid, __uuidof(MMDeviceEnumerator)) || !out || !*out) return hr;
    if (!IsEqualGUID(iid, __uuidof(IUnknown)) && !IsEqualGUID(iid, __uuidof(IMMDeviceEnumerator))) {
        traceHr("CoCreateInstance MMDeviceEnumerator unwrapped iid=" + iidText(iid), hr);
        return hr;
    }
    auto* unknown = static_cast<IUnknown*>(*out);
    IMMDeviceEnumerator* real = nullptr;
    if (FAILED(unknown->QueryInterface(__uuidof(IMMDeviceEnumerator), reinterpret_cast<void**>(&real)))) return hr;
    unknown->Release();
    *out = new EnumeratorWrap(real);
    static bool logged = false;
    if (!logged) { logged = true; traceHr("CoCreateInstance MMDeviceEnumerator wrapped", S_OK); }
    return S_OK;
}
void** observeClientVtable = nullptr;
class ObserveClient;
void logObserve(ObserveClient* client, const std::string& line);
class ObserveClock;
class ObserveRender final : public IAudioRenderClient {
    IAudioRenderClient* inner = nullptr;
    ObserveClient* owner = nullptr;
    long refs = 1;
    BYTE* data = nullptr;
public:
    ObserveRender(IAudioRenderClient* real, ObserveClient* client);
    ~ObserveRender();
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioRenderClient)) { *out = this; AddRef(); return S_OK; }
        return inner ? inner->QueryInterface(id, out) : E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { auto left = InterlockedDecrement(&refs); if (!left) delete this; return left; }
    HRESULT STDMETHODCALLTYPE GetBuffer(UINT32 frames, BYTE** out) override {
        if (!inner) return AUDCLNT_E_NOT_INITIALIZED;
        const HRESULT hr = inner->GetBuffer(frames, out);
        data = SUCCEEDED(hr) && out ? *out : nullptr;
        logObserve(owner, "GetBuffer frames=" + std::to_string(frames) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE ReleaseBuffer(UINT32 frames, DWORD flags) override;
};
class ObserveClock final : public IAudioClock {
    IAudioClock* inner = nullptr;
    ObserveClient* owner = nullptr;
    long refs = 1;
public:
    ObserveClock(IAudioClock* real, ObserveClient* client);
    ~ObserveClock();
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioClock)) { *out = this; AddRef(); return S_OK; }
        const HRESULT hr = inner ? inner->QueryInterface(id, out) : E_NOINTERFACE;
        logObserve(owner, "IAudioClock QueryInterface " + iidText(id) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { auto left = InterlockedDecrement(&refs); if (!left) delete this; return left; }
    HRESULT STDMETHODCALLTYPE GetFrequency(UINT64* frequency) override {
        const HRESULT hr = inner ? inner->GetFrequency(frequency) : AUDCLNT_E_NOT_INITIALIZED;
        logObserve(owner, "GetFrequency freq=" + std::to_string(frequency && SUCCEEDED(hr) ? *frequency : 0) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE GetPosition(UINT64* position, UINT64* qpcPosition) override {
        const HRESULT hr = inner ? inner->GetPosition(position, qpcPosition) : AUDCLNT_E_NOT_INITIALIZED;
        logObserve(owner, "GetPosition frames=" + std::to_string(position && SUCCEEDED(hr) ? *position : 0) + " qpc=" + std::to_string(qpcPosition && SUCCEEDED(hr) ? *qpcPosition : 0) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE GetCharacteristics(DWORD* characteristics) override {
        const HRESULT hr = inner ? inner->GetCharacteristics(characteristics) : AUDCLNT_E_NOT_INITIALIZED;
        logObserve(owner, "GetCharacteristics hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
};
class ObserveClient final : public IAudioClient3 {
public:
    IAudioClient* audio = nullptr;
    IAudioClient2* audio2 = nullptr;
    IAudioClient3* audio3 = nullptr;
    void* activateCaller = nullptr;
    long refs = 1;
    int id = 0;
    WAVEFORMATEXTENSIBLE format{};
    bool haveFormat = false;
    DWORD streamFlags = 0;
    REFERENCE_TIME period = 0;
    float peak[4]{};
    double square[4]{};
    int nonzero[4]{};
    long long seen = 0;
    float diff12 = 0, diff34 = 0;
    long long lastLog = 0;
    bool loggedMix = false;
    std::string route = "other";
    long long born = 0;
    int earlyLogs = 0;
    void early(const std::string& line) {
        if (!born) born = qpc();
        if (earlyLogs >= 200 || qpc() - born > qpcHz() * 3) return;
        ++earlyLogs;
        trace("USB observe tid=" + std::to_string(GetCurrentThreadId()) + " client=" + std::to_string(id) + " route=" + route + " " + line);
    }
    ObserveClient(IAudioClient* inner, void* caller) : audio(inner), activateCaller(caller) {
        static std::atomic<int> next{1};
        id = next.fetch_add(1);
        const unsigned long long rva = akAudioRva(caller);
        if (rva == kPadSinkRva) route = "pad-haptic 0x1D57D3";
        else if (rva == kContainerSinkRva) route = "container 0x31630A";
        else route = "other " + callerText(caller);
        if (!observeClientVtable) observeClientVtable = *reinterpret_cast<void***>(this);
        if (audio) {
            audio->QueryInterface(__uuidof(IAudioClient2), reinterpret_cast<void**>(&audio2));
            audio->QueryInterface(__uuidof(IAudioClient3), reinterpret_cast<void**>(&audio3));
        }
    }
    ~ObserveClient() {
        if (audio3) audio3->Release();
        if (audio2) audio2->Release();
        if (audio) audio->Release();
    }
    void consume(const BYTE* buffer, UINT32 frames, DWORD flags) {
        if (!haveFormat || !buffer || !frames) return;
        const bool silent = (flags & AUDCLNT_BUFFERFLAGS_SILENT) != 0;
        const bool floating = format.Format.wFormatTag == WAVE_FORMAT_IEEE_FLOAT || (format.Format.wFormatTag == WAVE_FORMAT_EXTENSIBLE && IsEqualGUID(format.SubFormat, kFloatSubformat));
        const unsigned channels = std::min<unsigned>(format.Format.nChannels, 4);
        const unsigned stride = format.Format.nBlockAlign ? format.Format.nBlockAlign : (floating ? channels * 4u : channels * 2u);
        if (!silent) {
            for (UINT32 frame = 0; frame < frames; ++frame) {
                const auto* bytes = buffer + static_cast<std::size_t>(frame) * stride;
                float sample[4]{};
                for (unsigned channel = 0; channel < channels; ++channel) {
                    if (floating) {
                        float value = 0;
                        std::memcpy(&value, bytes + channel * 4, sizeof value);
                        sample[channel] = std::isfinite(value) ? value : 0.f;
                    } else {
                        std::int16_t value = 0;
                        std::memcpy(&value, bytes + channel * 2, sizeof value);
                        sample[channel] = static_cast<float>(value) / 32768.f;
                    }
                    peak[channel] = std::max(peak[channel], std::fabs(sample[channel]));
                    square[channel] += static_cast<double>(sample[channel]) * sample[channel];
                    if (std::fabs(sample[channel]) > 1e-6f) ++nonzero[channel];
                }
                if (channels >= 2) diff12 = std::max(diff12, std::fabs(sample[0] - sample[1]));
                if (channels >= 4) diff34 = std::max(diff34, std::fabs(sample[2] - sample[3]));
            }
        }
        seen += frames;
        const long long now = qpc();
        if (lastLog && now - lastLog < qpcHz()) return;
        lastLog = now;
        auto num = [](double value) { char text[32]{}; sprintf_s(text, "%.6g", value); return std::string(text); };
        std::string line = "USB observe client=" + std::to_string(id) + " route=" + route + " caller=" + callerText(activateCaller) + " " + describeFormat(&format.Format) + " flags=" + std::to_string(streamFlags) + " period=" + std::to_string(period) + " frames=" + std::to_string(seen);
        const long long counted = seen ? seen : 1;
        for (unsigned channel = 0; channel < 4; ++channel) {
            const double rms = seen ? std::sqrt(square[channel] / static_cast<double>(counted)) : 0;
            line += " ch" + std::to_string(channel + 1) + " peak=" + num(peak[channel]) + " rms=" + num(rms) + " nz=" + std::to_string(nonzero[channel]);
            peak[channel] = 0;
            square[channel] = 0;
            nonzero[channel] = 0;
        }
        line += " diff12=" + num(diff12) + " diff34=" + num(diff34);
        diff12 = diff34 = 0;
        seen = 0;
        trace(line);
    }
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IAudioClient)) { *out = static_cast<IAudioClient*>(this); AddRef(); return S_OK; }
        if (id == __uuidof(IAudioClient2)) { if (!audio2) { *out = nullptr; return E_NOINTERFACE; } *out = static_cast<IAudioClient2*>(this); AddRef(); return S_OK; }
        if (id == __uuidof(IAudioClient3)) { if (!audio3) { *out = nullptr; return E_NOINTERFACE; } *out = static_cast<IAudioClient3*>(this); AddRef(); return S_OK; }
        const HRESULT hr = audio ? audio->QueryInterface(id, out) : E_NOINTERFACE;
        early("QueryInterface " + iidText(id) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { auto left = InterlockedDecrement(&refs); if (!left) delete this; return left; }
    HRESULT STDMETHODCALLTYPE Initialize(AUDCLNT_SHAREMODE share, DWORD flags, REFERENCE_TIME duration, REFERENCE_TIME periodicity, const WAVEFORMATEX* requested, LPCGUID session) override {
        streamFlags = flags;
        period = duration;
        if (requested) {
            std::memset(&format, 0, sizeof format);
            const unsigned bytes = sizeof(WAVEFORMATEX) + requested->cbSize;
            std::memcpy(&format, requested, bytes > sizeof format ? sizeof format : bytes);
            haveFormat = true;
        }
        const HRESULT hr = audio ? audio->Initialize(share, flags, duration, periodicity, requested, session) : AUDCLNT_E_NOT_INITIALIZED;
        UINT32 bufferFrames = 0;
        if (SUCCEEDED(hr) && audio) audio->GetBufferSize(&bufferFrames);
        early("Initialize share=" + std::to_string(share) + " flags=" + std::to_string(flags) + " period=" + std::to_string(duration) + " frames=" + std::to_string(bufferFrames) + " hr=" + std::to_string(static_cast<unsigned>(hr)) + " " + describeFormat(requested));
        trace("USB observe Initialize share=" + std::to_string(share) + " flags=" + std::to_string(flags) + " period=" + std::to_string(duration) + " frames=" + std::to_string(bufferFrames) + " route=" + route + " caller=" + callerText(_ReturnAddress()) + " activate=" + callerText(activateCaller) + " " + describeFormat(requested));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE GetBufferSize(UINT32* frames) override {
        const HRESULT hr = audio ? audio->GetBufferSize(frames) : AUDCLNT_E_NOT_INITIALIZED;
        early("GetBufferSize frames=" + std::to_string(frames && SUCCEEDED(hr) ? *frames : 0) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE GetStreamLatency(REFERENCE_TIME* latency) override {
        const HRESULT hr = audio ? audio->GetStreamLatency(latency) : AUDCLNT_E_NOT_INITIALIZED;
        early("GetStreamLatency latency=" + std::to_string(latency && SUCCEEDED(hr) ? *latency : 0) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE GetCurrentPadding(UINT32* padding) override {
        const HRESULT hr = audio ? audio->GetCurrentPadding(padding) : AUDCLNT_E_NOT_INITIALIZED;
        early("GetCurrentPadding padding=" + std::to_string(padding && SUCCEEDED(hr) ? *padding : 0) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE IsFormatSupported(AUDCLNT_SHAREMODE share, const WAVEFORMATEX* requested, WAVEFORMATEX** closest) override { return audio ? audio->IsFormatSupported(share, requested, closest) : AUDCLNT_E_NOT_INITIALIZED; }
    HRESULT STDMETHODCALLTYPE GetMixFormat(WAVEFORMATEX** requested) override {
        const HRESULT hr = audio ? audio->GetMixFormat(requested) : AUDCLNT_E_NOT_INITIALIZED;
        if (!loggedMix && SUCCEEDED(hr) && requested && *requested) { loggedMix = true; trace("USB observe GetMixFormat caller=" + callerText(activateCaller) + " " + describeFormat(*requested)); }
        return hr;
    }
    HRESULT STDMETHODCALLTYPE GetDevicePeriod(REFERENCE_TIME* def, REFERENCE_TIME* minimum) override {
        const HRESULT hr = audio ? audio->GetDevicePeriod(def, minimum) : AUDCLNT_E_NOT_INITIALIZED;
        static bool logged = false;
        if (!logged && SUCCEEDED(hr)) { logged = true; trace("USB observe GetDevicePeriod default=" + std::to_string(def ? *def : 0) + " min=" + std::to_string(minimum ? *minimum : 0)); }
        return hr;
    }
    HRESULT STDMETHODCALLTYPE Start() override { const HRESULT hr = audio ? audio->Start() : AUDCLNT_E_NOT_INITIALIZED; early("Start hr=" + std::to_string(static_cast<unsigned>(hr))); return hr; }
    HRESULT STDMETHODCALLTYPE Stop() override { const HRESULT hr = audio ? audio->Stop() : AUDCLNT_E_NOT_INITIALIZED; early("Stop hr=" + std::to_string(static_cast<unsigned>(hr))); return hr; }
    HRESULT STDMETHODCALLTYPE Reset() override { const HRESULT hr = audio ? audio->Reset() : AUDCLNT_E_NOT_INITIALIZED; early("Reset hr=" + std::to_string(static_cast<unsigned>(hr))); return hr; }
    HRESULT STDMETHODCALLTYPE SetEventHandle(HANDLE handle) override { const HRESULT hr = audio ? audio->SetEventHandle(handle) : AUDCLNT_E_NOT_INITIALIZED; early(std::string("SetEventHandle ") + (handle ? "set" : "null") + " hr=" + std::to_string(static_cast<unsigned>(hr))); return hr; }
    HRESULT STDMETHODCALLTYPE GetService(REFIID iid, void** out) override {
        if (!audio) return AUDCLNT_E_NOT_INITIALIZED;
        const HRESULT hr = audio->GetService(iid, out);
        early("GetService " + iidText(iid) + " hr=" + std::to_string(static_cast<unsigned>(hr)) + " caller=" + callerText(_ReturnAddress()));
        if (SUCCEEDED(hr) && out && *out && iid == __uuidof(IAudioRenderClient)) *out = new ObserveRender(static_cast<IAudioRenderClient*>(*out), this);
        if (SUCCEEDED(hr) && out && *out && iid == __uuidof(IAudioClock)) *out = new ObserveClock(static_cast<IAudioClock*>(*out), this);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE IsOffloadCapable(AUDIO_STREAM_CATEGORY category, BOOL* capable) override { return audio2 ? audio2->IsOffloadCapable(category, capable) : E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE SetClientProperties(const AudioClientProperties* properties) override { return audio2 ? audio2->SetClientProperties(properties) : E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE GetBufferSizeLimits(const WAVEFORMATEX* requested, BOOL eventDriven, REFERENCE_TIME* minimum, REFERENCE_TIME* maximum) override { return audio2 ? audio2->GetBufferSizeLimits(requested, eventDriven, minimum, maximum) : E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE GetSharedModeEnginePeriod(const WAVEFORMATEX* requested, UINT32* def, UINT32* fundamental, UINT32* minimum, UINT32* maximum) override { return audio3 ? audio3->GetSharedModeEnginePeriod(requested, def, fundamental, minimum, maximum) : E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE GetCurrentSharedModeEnginePeriod(WAVEFORMATEX** requested, UINT32* frames) override { return audio3 ? audio3->GetCurrentSharedModeEnginePeriod(requested, frames) : E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE InitializeSharedAudioStream(DWORD flags, UINT32 frames, const WAVEFORMATEX* requested, LPCGUID session) override { return audio3 ? audio3->InitializeSharedAudioStream(flags, frames, requested, session) : E_NOTIMPL; }
};
void logObserve(ObserveClient* client, const std::string& line) { if (client) client->early(line); }
ObserveClock::ObserveClock(IAudioClock* real, ObserveClient* client) : inner(real), owner(client) { if (owner) owner->AddRef(); }
ObserveClock::~ObserveClock() { if (inner) inner->Release(); if (owner) owner->Release(); }
ObserveRender::ObserveRender(IAudioRenderClient* real, ObserveClient* client) : inner(real), owner(client) { if (owner) owner->AddRef(); }
ObserveRender::~ObserveRender() { if (inner) inner->Release(); if (owner) owner->Release(); }
HRESULT STDMETHODCALLTYPE ObserveRender::ReleaseBuffer(UINT32 frames, DWORD flags) {
    if (owner && data) owner->consume(data, frames, flags);
    data = nullptr;
    const HRESULT hr = inner ? inner->ReleaseBuffer(frames, flags) : AUDCLNT_E_NOT_INITIALIZED;
    logObserve(owner, "ReleaseBuffer frames=" + std::to_string(frames) + " flags=" + std::to_string(flags) + " hr=" + std::to_string(static_cast<unsigned>(hr)));
    return hr;
}
void logUsbProperties(IMMDevice* endpoint) {
    static bool done = false;
    if (done || !endpoint) return;
    done = true;
    IPropertyStore* properties = nullptr;
    if (FAILED(endpoint->OpenPropertyStore(STGM_READ, &properties)) || !properties) return;
    auto read = [&](const PROPERTYKEY& key, const char* name) {
        PROPVARIANT value;
        PropVariantInit(&value);
        const HRESULT hr = properties->GetValue(key, &value);
        std::string text = "hr=" + std::to_string(static_cast<unsigned>(hr));
        if (SUCCEEDED(hr) && value.vt == VT_LPWSTR && value.pwszVal) {
            char narrow[180]{};
            WideCharToMultiByte(CP_UTF8, 0, value.pwszVal, -1, narrow, 160, nullptr, nullptr);
            text = narrow;
        } else if (SUCCEEDED(hr) && value.vt == VT_UI4) text = std::to_string(value.ulVal);
        else if (SUCCEEDED(hr) && value.vt == VT_BLOB && value.blob.pBlobData && value.blob.cbSize >= sizeof(WAVEFORMATEX)) text = describeFormat(reinterpret_cast<WAVEFORMATEX*>(value.blob.pBlobData));
        else if (SUCCEEDED(hr) && value.vt == VT_CLSID && value.puuid) text = iidText(*value.puuid);
        PropVariantClear(&value);
        trace(std::string("USB observe prop ") + name + " " + text);
    };
    read(kEngineFormat, "DeviceFormat");
    read(kFriendlyName, "FriendlyName");
    read(kDeviceDesc, "DeviceDesc");
    read(kInterfaceName, "InterfaceName");
    read(kFormFactor, "FormFactor");
    read(kContainer, "ContainerId");
    properties->Release();
}
void** observeDeviceVtable = nullptr;
thread_local void* outerActivateCaller = nullptr;
class ObserveDevice final : public IMMDevice {
    IMMDevice* inner = nullptr;
    long refs = 1;
public:
    explicit ObserveDevice(IMMDevice* real) : inner(real) {
        if (!observeDeviceVtable) observeDeviceVtable = *reinterpret_cast<void***>(this);
        logUsbProperties(inner);
    }
    ~ObserveDevice() { if (inner) inner->Release(); }
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id, void** out) override {
        if (!out) return E_POINTER;
        if (id == __uuidof(IUnknown) || id == __uuidof(IMMDevice)) { *out = this; AddRef(); return S_OK; }
        return inner ? inner->QueryInterface(id, out) : E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override { auto left = InterlockedDecrement(&refs); if (!left) delete this; return left; }
    HRESULT STDMETHODCALLTYPE Activate(REFIID id, DWORD context, PROPVARIANT* params, void** out) override {
        if (!inner) return E_FAIL;
        void* raw = nullptr;
        outerActivateCaller = _ReturnAddress();
        const HRESULT hr = inner->Activate(id, context, params, &raw);
        outerActivateCaller = nullptr;
        if (FAILED(hr) || !raw) { if (out) *out = raw; return hr; }
        const bool audio = id == __uuidof(IAudioClient) || id == __uuidof(IAudioClient2) || id == __uuidof(IAudioClient3);
        if (audio && raw && !(observeClientVtable && *reinterpret_cast<void***>(raw) == observeClientVtable)) raw = new ObserveClient(static_cast<IAudioClient*>(raw), _ReturnAddress());
        if (out) *out = raw;
        trace("USB observe Activate caller=" + callerText(_ReturnAddress()) + " route=" + (raw && observeClientVtable && *reinterpret_cast<void***>(raw) == observeClientVtable ? static_cast<ObserveClient*>(raw)->route : std::string("passthrough")) + " iid=" + iidText(id));
        return hr;
    }
    HRESULT STDMETHODCALLTYPE OpenPropertyStore(DWORD access, IPropertyStore** properties) override { return inner ? inner->OpenPropertyStore(access, properties) : E_FAIL; }
    HRESULT STDMETHODCALLTYPE GetId(LPWSTR* id) override { return inner ? inner->GetId(id) : E_FAIL; }
    HRESULT STDMETHODCALLTYPE GetState(DWORD* state) override { return inner ? inner->GetState(state) : E_FAIL; }
};
using DeviceActivateFn = HRESULT(STDMETHODCALLTYPE*)(IMMDevice*, REFIID, DWORD, PROPVARIANT*, void**);
struct ActivateHook { void** vtable; DeviceActivateFn original; };
std::mutex activateHookMutex;
std::vector<ActivateHook> activateHooks;
std::mutex padDeviceMutex;
std::unordered_map<void*, bool> padDeviceCache;
std::wstring recordedPadId;
GUID recordedContainer{};
bool haveRecordedContainer = false;
bool isObserveDevice(IMMDevice* device) {
    return device && observeDeviceVtable && *reinterpret_cast<void***>(device) == observeDeviceVtable;
}
bool isObserveClientPtr(void* pointer) {
    return pointer && observeClientVtable && *reinterpret_cast<void***>(pointer) == observeClientVtable;
}
bool readDeviceContainer(IMMDevice* device, GUID& out) {
    if (!device) return false;
    IPropertyStore* store = nullptr;
    if (FAILED(device->OpenPropertyStore(STGM_READ, &store)) || !store) return false;
    PROPVARIANT value;
    PropVariantInit(&value);
    const HRESULT hr = store->GetValue(kContainer, &value);
    const bool ok = SUCCEEDED(hr) && value.vt == VT_CLSID && value.puuid;
    if (ok) out = *value.puuid;
    PropVariantClear(&value);
    store->Release();
    return ok;
}
bool deviceNameHasDualSense(IMMDevice* device) {
    IPropertyStore* store = nullptr;
    if (!device || FAILED(device->OpenPropertyStore(STGM_READ, &store)) || !store) return false;
    bool match = false;
    for (const PROPERTYKEY* key : {&kFriendlyName, &kInterfaceName, &kDeviceDesc}) {
        PROPVARIANT value;
        PropVariantInit(&value);
        if (SUCCEEDED(store->GetValue(*key, &value)) && value.vt == VT_LPWSTR && value.pwszVal && wcsstr(value.pwszVal, L"DualSense")) match = true;
        PropVariantClear(&value);
    }
    store->Release();
    return match;
}
bool isPadEndpoint(IMMDevice* device) {
    if (!device || isObserveDevice(device)) return false;
    {
        std::lock_guard lock(padDeviceMutex);
        const auto found = padDeviceCache.find(device);
        if (found != padDeviceCache.end()) return found->second;
    }
    bool match = false;
    LPWSTR id = nullptr;
    if (SUCCEEDED(device->GetId(&id)) && id) {
        std::lock_guard lock(padDeviceMutex);
        if (!recordedPadId.empty() && _wcsicmp(id, recordedPadId.c_str()) == 0) match = true;
        CoTaskMemFree(id);
    }
    GUID container{};
    if (!match && readDeviceContainer(device, container)) {
        GUID hid{};
        const bool hidOk = controllerContainer(hid);
        std::lock_guard lock(padDeviceMutex);
        if (haveRecordedContainer && IsEqualGUID(container, recordedContainer)) match = true;
        if (hidOk && IsEqualGUID(container, hid)) match = true;
    }
    if (!match && deviceNameHasDualSense(device)) match = true;
    if (match) {
        std::lock_guard lock(padDeviceMutex);
        padDeviceCache[device] = true;
    }
    return match;
}
HRESULT STDMETHODCALLTYPE hookedDeviceActivate(IMMDevice* self, REFIID id, DWORD context, PROPVARIANT* params, void** out) {
    DeviceActivateFn original = nullptr;
    {
        void** vtable = *reinterpret_cast<void***>(self);
        std::lock_guard lock(activateHookMutex);
        for (const auto& hook : activateHooks) if (hook.vtable == vtable) original = hook.original;
    }
    if (!original) return E_FAIL;
    const HRESULT hr = original(self, id, context, params, out);
    if (FAILED(hr) || !out || !*out) return hr;
    const bool audio = id == __uuidof(IAudioClient) || id == __uuidof(IAudioClient2) || id == __uuidof(IAudioClient3);
    if (!audio || !isPadEndpoint(self) || isObserveClientPtr(*out)) return hr;
    void* caller = _ReturnAddress();
    if (!akAudioRva(caller) && outerActivateCaller) caller = outerActivateCaller;
    auto* client = new ObserveClient(static_cast<IAudioClient*>(*out), caller);
    *out = client;
    trace("USB observe Activate caller=" + callerText(_ReturnAddress()) + " route=" + client->route + " iid=" + iidText(id));
    return hr;
}
void hookDeviceActivate(IMMDevice* device) {
    if (!device || isObserveDevice(device)) return;
    void** vtable = *reinterpret_cast<void***>(device);
    void** slot = vtable + 3;
    std::lock_guard lock(activateHookMutex);
    for (const auto& hook : activateHooks) if (hook.vtable == vtable) return;
    if (*slot == reinterpret_cast<void*>(&hookedDeviceActivate)) return;
    DWORD old = 0;
    if (!VirtualProtect(slot, sizeof(void*), PAGE_EXECUTE_READWRITE, &old)) {
        trace("USB observe Activate hook failed");
        return;
    }
    ActivateHook hook{vtable, reinterpret_cast<DeviceActivateFn>(*slot)};
    *slot = reinterpret_cast<void*>(&hookedDeviceActivate);
    DWORD ignored = 0;
    VirtualProtect(slot, sizeof(void*), old, &ignored);
    activateHooks.push_back(hook);
    trace("USB observe hooked IMMDevice::Activate");
}
void rememberRealPad(IMMDevice* device) {
    if (!device || isObserveDevice(device)) return;
    hookDeviceActivate(device);
    LPWSTR id = nullptr;
    if (SUCCEEDED(device->GetId(&id)) && id) {
        std::lock_guard lock(padDeviceMutex);
        recordedPadId = id;
        padDeviceCache[device] = true;
        CoTaskMemFree(id);
    }
    GUID container{};
    if (readDeviceContainer(device, container)) {
        std::lock_guard lock(padDeviceMutex);
        recordedContainer = container;
        haveRecordedContainer = true;
        padDeviceCache[device] = true;
    }
}
IMMDevice* watchPadDevice(IMMDevice* device) {
    if (!device || isObserveDevice(device) || !isPadEndpoint(device)) return device;
    hookDeviceActivate(device);
    static int noted = 0;
    if (noted < 6) { ++noted; trace("USB observe wrapped enumerated pad device"); }
    return new ObserveDevice(device);
}
bool __fastcall getMmDevice(void* self, int pad, IMMDevice** out) {
    if (!originalGetMmDevice) return false;
    const bool wanted = btDualSenseWanted();
    const bool found = originalGetMmDevice(self, pad, out);
    static int lookups = 0;
    if (lookups < 12) { ++lookups; trace("GetMMDeviceFromPadHandle pad=" + std::to_string(pad) + " wanted=" + (wanted ? "1" : "0") + " found=" + (found ? "1" : "0")); }
    if (found && out && *out) {
        rememberRealPad(*out);
        *out = new ObserveDevice(*out);
        trace("USB observe wrapped real pad endpoint");
        return true;
    }
    if (!wanted || !out) return found;
    *out = &device();
    device().AddRef();
    engine().padArmed.store(true, std::memory_order_release);
    static bool logged = false;
    if (!logged) { logged = true; logLine("BtHaptics fake 4ch pcm16 mask=33 endpoint for pad " + std::to_string(pad) + " name follows the connected DualSense or Edge"); }
    trace("pad haptic armed for the next AkAudio+0x1D57D3 client");
    return true;
}
}
using DebugAFn = void(WINAPI*)(LPCSTR);
using DebugWFn = void(WINAPI*)(LPCWSTR);
DebugAFn originalDebugA{};
DebugWFn originalDebugW{};
bool wwiseText(const std::string& text) {
    std::string lower(text.size(), '\0');
    for (std::size_t i = 0; i < text.size(); ++i) lower[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[i])));
    return lower.find("haptic") != std::string::npos || lower.find("scepad") != std::string::npos || lower.find("mmdevice") != std::string::npos || lower.find("channel count") != std::string::npos || lower.find("audio device") != std::string::npos;
}
void WINAPI debugA(LPCSTR text) {
    if (text && wwiseText(text)) logLine(std::string("Wwise ") + std::string(text).substr(0, 300));
    if (originalDebugA) originalDebugA(text);
}
void WINAPI debugW(LPCWSTR text) {
    if (text) {
        char narrow[320]{};
        WideCharToMultiByte(CP_UTF8, 0, text, -1, narrow, 300, nullptr, nullptr);
        if (wwiseText(narrow)) logLine(std::string("Wwise ") + narrow);
    }
    if (originalDebugW) originalDebugW(text);
}
bool patchDebugImport(HMODULE module, const char* function, void* detour, void** original) {
    auto* base = reinterpret_cast<std::uint8_t*>(module);
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!dir.VirtualAddress) return false;
    auto* imports = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress);
    for (; imports->Name; ++imports) {
        if (_stricmp(reinterpret_cast<const char*>(base + imports->Name), "KERNEL32.dll") != 0) continue;
        auto* hint = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imports->OriginalFirstThunk);
        auto* iat = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imports->FirstThunk);
        for (; hint->u1.AddressOfData; ++hint, ++iat) {
            if (IMAGE_SNAP_BY_ORDINAL(hint->u1.Ordinal)) continue;
            const auto* imported = reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + hint->u1.AddressOfData);
            if (std::strcmp(imported->Name, function) != 0) continue;
            *original = reinterpret_cast<void*>(iat->u1.Function);
            DWORD old = 0;
            if (!VirtualProtect(&iat->u1.Function, sizeof(void*), PAGE_READWRITE, &old)) return false;
            iat->u1.Function = reinterpret_cast<ULONG_PTR>(detour);
            DWORD ignored = 0;
            VirtualProtect(&iat->u1.Function, sizeof(void*), old, &ignored);
            return true;
        }
    }
    return false;
}
using PadInfoFn = int(*)(int, void*);
using PadTypeFn = int(*)(int, int*);
using PadAudioFn = int(*)(int);
using PadPathFn = int(*)(int, int);
PadInfoFn originalPadInfo = nullptr;
PadTypeFn originalPadType = nullptr;
PadAudioFn originalPadAudio = nullptr;
PadPathFn originalPadPath = nullptr;
int padGetControllerInformation(int handle, void* info) {
    const int rc = originalPadInfo ? originalPadInfo(handle, info) : -1;
    static int logs = 0;
    if (logs < 8) {
        ++logs;
        trace("connection candidate scePadGetControllerInformation handle=" + std::to_string(handle) + " rc=" + std::to_string(rc) + (info ? " bytes=" + hexBytes(static_cast<const std::uint8_t*>(info), 32) : " info=null") + " caller=" + callerText(_ReturnAddress()));
    }
    return rc;
}
int padGetControllerType(int handle, int* type) {
    const int rc = originalPadType ? originalPadType(handle, type) : -1;
    static int logs = 0;
    if (logs < 8) {
        ++logs;
        trace("connection candidate scePadGetControllerType handle=" + std::to_string(handle) + " rc=" + std::to_string(rc) + " type=" + (type ? std::to_string(*type) : "null") + " caller=" + callerText(_ReturnAddress()));
    }
    return rc;
}
int padIsSupportedAudio(int handle) {
    const int rc = originalPadAudio ? originalPadAudio(handle) : -1;
    static int logs = 0;
    if (logs < 8) { ++logs; trace("connection candidate scePadIsSupportedAudioFunction handle=" + std::to_string(handle) + " rc=" + std::to_string(rc) + " caller=" + callerText(_ReturnAddress())); }
    return rc;
}
int padSetAudioOutPath(int handle, int path) {
    static int logs = 0;
    if (logs < 8) { ++logs; trace("connection candidate scePadSetAudioOutPath handle=" + std::to_string(handle) + " path=" + std::to_string(path) + " caller=" + callerText(_ReturnAddress())); }
    return originalPadPath ? originalPadPath(handle, path) : -1;
}
void ensureScePadObserve() {
    static std::atomic<bool> done{false};
    if (done.load(std::memory_order_acquire)) return;
    HMODULE module = GetModuleHandleW(L"libScePad.dll");
    if (!module) return;
    const auto status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) return;
    auto hook = [&](const char* name, void* detour, void** original) {
        auto* target = GetProcAddress(module, name);
        if (!target) { trace(std::string("connection candidate missing ") + name); return; }
        if (MH_CreateHook(target, detour, original) != MH_OK || MH_EnableHook(target) != MH_OK) trace(std::string("connection candidate hook failed ") + name);
        else trace(std::string("connection candidate hooked ") + name);
    };
    hook("scePadGetControllerInformation", reinterpret_cast<void*>(&padGetControllerInformation), reinterpret_cast<void**>(&originalPadInfo));
    hook("scePadGetControllerType", reinterpret_cast<void*>(&padGetControllerType), reinterpret_cast<void**>(&originalPadType));
    hook("scePadIsSupportedAudioFunction", reinterpret_cast<void*>(&padIsSupportedAudio), reinterpret_cast<void**>(&originalPadAudio));
    hook("scePadSetAudioOutPath", reinterpret_cast<void*>(&padSetAudioOutPath), reinterpret_cast<void**>(&originalPadPath));
    done.store(true, std::memory_order_release);
}
void ensureBtHapticHook() {
    ensureWorker();
    ensureScePadObserve();
    int expected = 0;
    if (!hookState.compare_exchange_strong(expected, 3)) return;
    HMODULE module = GetModuleHandleW(L"Returnal-AkAudio-Win64-Shipping.dll");
    if (!module) { hookState.store(0, std::memory_order_release); return; }
    auto* target = GetProcAddress(module, "?GetMMDeviceFromPadHandle@FAkAudioDevice@@QEAA_NHAEAPEAUIMMDevice@@@Z");
    if (!target) {
        hookState.store(2, std::memory_order_release);
        logLine("BtHaptics hook failed: GetMMDeviceFromPadHandle missing");
        return;
    }
    const auto status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) {
        hookState.store(0, std::memory_order_release);
        logLine("BtHaptics hook failed: MinHook unavailable");
        return;
    }
    if (MH_CreateHook(target, reinterpret_cast<void*>(&getMmDevice), reinterpret_cast<void**>(&originalGetMmDevice)) != MH_OK || MH_EnableHook(target) != MH_OK) {
        hookState.store(2, std::memory_order_release);
        logLine("BtHaptics hook failed: hook was not installed");
        return;
    }
    hookState.store(1, std::memory_order_release);
    auto* ole = GetModuleHandleW(L"ole32.dll");
    auto* create = ole ? GetProcAddress(ole, "CoCreateInstance") : nullptr;
    if (!create || MH_CreateHook(create, reinterpret_cast<void*>(&coCreate), reinterpret_cast<void**>(&originalCoCreate)) != MH_OK || MH_EnableHook(create) != MH_OK)
        logLine("BtHaptics enumerator hook failed");
    else logLine("BtHaptics enumerator hook installed");
    const bool debugAInstalled = patchDebugImport(module, "OutputDebugStringA", reinterpret_cast<void*>(&debugA), reinterpret_cast<void**>(&originalDebugA));
    const bool debugWInstalled = patchDebugImport(module, "OutputDebugStringW", reinterpret_cast<void*>(&debugW), reinterpret_cast<void**>(&originalDebugW));
    logLine(std::string("BtHaptics hook installed debugA=") + (debugAInstalled ? "1" : "0") + " debugW=" + (debugWInstalled ? "1" : "0"));
}
}
