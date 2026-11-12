#include "rg/bt_haptic.hpp"
#include "rg/dualsense_bt.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
namespace rg {
namespace {
std::atomic<bool> streamActive{false};
std::atomic<bool> speakerActive{false};
std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
std::int8_t narrow(int value) {
    value /= 256;
    if (value > 127) return 127;
    if (value < -128) return -128;
    return static_cast<std::int8_t>(value);
}
}
bool HapticDownsampler::push(std::int16_t left, std::int16_t right, std::int8_t& outLeft, std::int8_t& outRight) {
    sumLeft += left;
    sumRight += right;
    if (++count < 16) return false;
    outLeft = narrow(sumLeft / 16);
    outRight = narrow(sumRight / 16);
    sumLeft = sumRight = count = 0;
    return true;
}
bool buildBluetoothHapticReport(std::uint8_t* dst, const std::int8_t* interleavedStereo32, std::uint8_t seq, std::uint8_t block) {
    if (!dst || !interleavedStereo32) return false;
    (void)seq;
    std::memset(dst, 0, 142);
    dst[0] = 0x32;
    dst[1] = 0;
    dst[2] = 0x91;
    dst[3] = 0x07;
    dst[4] = 0xFE;
    dst[9] = 0xFF;
    dst[10] = block;
    dst[11] = 0x92;
    dst[12] = 0x40;
    std::memcpy(dst + 13, interleavedStereo32, 64);
    std::uint8_t seeded[139];
    seeded[0] = 0xA2;
    std::memcpy(seeded + 1, dst, 138);
    const std::uint32_t crc = crc32(seeded, 139);
    std::memcpy(dst + 138, &crc, sizeof crc);
    return true;
}
bool hapticReportCrcValid(const std::uint8_t* report) {
    if (!report || report[0] != 0x32) return false;
    std::uint8_t seeded[139];
    seeded[0] = 0xA2;
    std::memcpy(seeded + 1, report, 138);
    std::uint32_t expect = 0;
    std::memcpy(&expect, report + 138, sizeof expect);
    return crc32(seeded, 139) == expect;
}
#pragma pack(push, 1)
struct OutputCommon {
    std::uint8_t validFlag0;
    std::uint8_t validFlag1;
    std::uint8_t motorRight;
    std::uint8_t motorLeft;
    std::uint8_t headphone;
    std::uint8_t speaker;
    std::uint8_t mic;
    std::uint8_t audioFlags;
    std::uint8_t muteLed;
    std::uint8_t powerSave;
    std::uint8_t rightMode;
    std::uint8_t rightParam[10];
    std::uint8_t leftMode;
    std::uint8_t leftParam[10];
    std::uint32_t timestamp;
    std::uint8_t reduceMotorPower;
    std::uint8_t audioFlags2;
    std::uint8_t validFlag2;
    std::uint8_t hapticsFlags;
};
#pragma pack(pop)
static_assert(sizeof(OutputCommon) == 40, "DualSense output common block");
static_assert(offsetof(OutputCommon, powerSave) == 9, "power save");
static_assert(offsetof(OutputCommon, reduceMotorPower) == 36, "motor attenuation");
static_assert(offsetof(OutputCommon, hapticsFlags) == 39, "haptic flags");
void applyBluetoothHapticMode(std::uint8_t* report, bool fullPower) {
    if (!report || report[0] != 0x31) return;
    auto* common = reinterpret_cast<OutputCommon*>(report + 3);
    common->validFlag0 = static_cast<std::uint8_t>(common->validFlag0 & ~0x03);
    if (fullPower) common->validFlag1 = static_cast<std::uint8_t>(common->validFlag1 | 0x02 | 0x20 | 0x40);
    else common->validFlag1 = static_cast<std::uint8_t>((common->validFlag1 & ~std::uint8_t(0x02 | 0x40)) | 0x20);
    common->hapticsFlags = static_cast<std::uint8_t>(common->hapticsFlags | 0x01);
    if (fullPower) {
        common->powerSave = static_cast<std::uint8_t>(common->powerSave & ~((1u << 2) | (1u << 7)));
        common->reduceMotorPower = static_cast<std::uint8_t>(common->reduceMotorPower & ~0x07);
    }
}
bool buildBluetoothHapticModeReport(std::uint8_t* dst, std::uint8_t seq, bool fullPower) {
    std::uint8_t usb[48]{};
    usb[0] = 0x02;
    if (!buildBluetoothOutput(dst, usb, 48, seq)) return false;
    applyBluetoothHapticMode(dst, fullPower);
    refreshBluetoothOutputCrc(dst, 78);
    return true;
}
void setHapticStreamActive(bool active) { streamActive.store(active, std::memory_order_release); }
bool hapticStreamActive() { return streamActive.load(std::memory_order_acquire); }
void setSpeakerStreamActive(bool active) { speakerActive.store(active, std::memory_order_release); }
bool speakerStreamActive() { return speakerActive.load(std::memory_order_acquire); }
bool buildBluetoothSpeakerReport(std::uint8_t* dst, std::uint8_t seq, std::uint8_t counter, const std::uint8_t* opus200, const std::int8_t* haptic64, bool counterLikeHaptic) {
    if (!dst || !opus200) return false;
    std::memset(dst, 0, 334);
    dst[0] = 0x35;
    dst[1] = static_cast<std::uint8_t>((seq & 0x0f) << 4);
    dst[2] = 0x91;
    dst[3] = 0x07;
    dst[4] = haptic64 ? 0xFE : 0x7E;
    if (counterLikeHaptic) { dst[9] = 0xFF; dst[10] = counter; }
    else dst[9] = counter;
    dst[11] = 0x93;
    dst[12] = 0xC8;
    std::memcpy(dst + 13, opus200, 200);
    if (haptic64) {
        dst[213] = 0x92;
        dst[214] = 0x40;
        std::memcpy(dst + 215, haptic64, 64);
    }
    std::uint8_t seeded[331];
    seeded[0] = 0xA2;
    std::memcpy(seeded + 1, dst, 330);
    const std::uint32_t crc = crc32(seeded, 331);
    std::memcpy(dst + 330, &crc, sizeof crc);
    return true;
}
bool speakerReportCrcValid(const std::uint8_t* report) {
    if (!report || report[0] != 0x35) return false;
    std::uint8_t seeded[331];
    seeded[0] = 0xA2;
    std::memcpy(seeded + 1, report, 330);
    std::uint32_t expect = 0;
    std::memcpy(&expect, report + 330, sizeof expect);
    return crc32(seeded, 331) == expect;
}
void applyBluetoothSpeaker(std::uint8_t* report, std::uint8_t volume) {
    if (!report || report[0] != 0x31) return;
    auto* common = reinterpret_cast<OutputCommon*>(report + 3);
    // OR only. Flag0 0x04 and flag1 0x40/0x20 stay as the haptic merge left them.
    // Flag0 0x01/0x02 are rumble selects and take the actuators out of audio mode, so they are not set.
    // dualsense-neo control.rs routing(Speaker): flag0 0xF0 so headphone, speaker,
    // mic and audio-control bytes are honored. 0x10 is the bit that made a silent
    // pad audible (SPEC section 6.1). Rumble bits 0x01/0x02 stay off.
    common->validFlag0 = static_cast<std::uint8_t>(common->validFlag0 | 0x10 | 0x20 | 0x40 | 0x80);
    common->validFlag1 = static_cast<std::uint8_t>(common->validFlag1 | 0x80);
    common->speaker = volume;
    if (common->headphone == 0) common->headphone = volume > 0x7f ? 0x7f : volume;
    // Path 3 (0x30) is X-X-R, the internal speaker. Path 2 (0x20) is L-L-R and
    // left this Edge silent while haptics played. dualsense-neo PATH_SPEAKER.
    common->audioFlags = static_cast<std::uint8_t>((common->audioFlags & ~std::uint8_t{0x30}) | 0x30);
    // Preamp 5 is the loudest clean gain. 2 left the speaker under-driven (SPEC section 6.2).
    common->audioFlags2 = static_cast<std::uint8_t>((common->audioFlags2 & ~0x07) | 0x05);
}
unsigned renderBufferBytes(unsigned frames) { return frames * kHapticFrameBytes * 2u + kBufferGuardBytes; }
void paintBufferGuard(std::uint8_t* data, unsigned used, unsigned total) {
    if (!data || used > total) return;
    for (unsigned i = used; i < total; ++i) data[i] = kBufferGuardByte;
}
bool bufferGuardIntact(const std::uint8_t* data, unsigned used, unsigned total) {
    if (!data || used > total) return false;
    for (unsigned i = used; i < total; ++i) if (data[i] != kBufferGuardByte) return false;
    return true;
}
int hapticChannelPair(int setting, float peak12, float peak34) {
    constexpr float eps = 1e-6f;
    if (setting == 1) return 12;
    if (setting == 2) return 34;
    (void)peak12;
    if (peak34 > eps) return 34;
    return 0;
}
std::int16_t floatToPcm16(float value) {
    if (!std::isfinite(value)) return 0;
    value = std::clamp(value, -1.f, 1.f);
    return static_cast<std::int16_t>(std::lround(value * 32767.f));
}
}
