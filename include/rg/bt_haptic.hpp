#pragma once
#include <cstdint>
namespace rg {
// 48 kHz int16 frames become one 3 kHz int8 stereo sample every 16 frames (block mean).
struct HapticDownsampler {
    int sumLeft = 0, sumRight = 0, count = 0;
    bool push(std::int16_t left, std::int16_t right, std::int8_t& outLeft, std::int8_t& outRight);
};
// Report 0x32, 142 bytes. SAxense layout that vibrates a DualSense Edge over Bluetooth:
// sequence nibble stays 0, 91 07 fe 00 00 00 00 ff <counter> 92 40, 32 stereo int8
// samples, CRC32 of 0xA2 || bytes 0..137. https://github.com/egormanga/SAxense
bool buildBluetoothHapticReport(std::uint8_t* dst, const std::int8_t* interleavedStereo32, std::uint8_t seq, std::uint8_t block);
bool hapticReportCrcValid(const std::uint8_t* report);
// Report 0x35, 334 bytes. Opus at offset 13. haptic64, when set, appends one 0x12 frame
// after the Opus prefix (dualsense-neo SPEC section 3.1). Null leaves an audio-only report.
// counterLikeHaptic writes the working 0x32 control tail: byte 9 = 0xFF, byte 10 = counter.
// Otherwise byte 9 is the counter and byte 10 stays 0. Does not alter report 0x32.
bool buildBluetoothSpeakerReport(std::uint8_t* dst, std::uint8_t seq, std::uint8_t counter, const std::uint8_t* opus200, const std::int8_t* haptic64 = nullptr, bool counterLikeHaptic = true);
// Samples to discard so a haptic send keeps at most two 32-sample blocks. A hitch then cannot sit in the queue.
inline int hapticBacklogDrop(int queued) {
    constexpr int keep = 64;
    if (queued <= keep) return 0;
    return queued - keep;
}
bool speakerReportCrcValid(const std::uint8_t* report);
// ORs speaker volume and path 2 (0x20, L-L-R) into an existing 0x31. Does not clear haptic flags. Does not restamp the CRC.
void applyBluetoothSpeaker(std::uint8_t* report, std::uint8_t volume);
void setSpeakerStreamActive(bool active);
bool speakerStreamActive();
// 0x31 that keeps the actuators in audio mode. fullPower uses flag1 0x62; otherwise flag1 is 0x20.
bool buildBluetoothHapticModeReport(std::uint8_t* dst, std::uint8_t seq, bool fullPower = false);
// Merges that mode into an existing Bluetooth 0x31. Lightbar and triggers stay. Does not restamp the CRC.
void applyBluetoothHapticMode(std::uint8_t* report, bool fullPower = false);
void setHapticStreamActive(bool active);
bool hapticStreamActive();
// USB DualSense shared-mode mix is 4ch 48 kHz float32, 16 bytes per frame.
inline constexpr unsigned kHapticFrameBytes = 16;
inline constexpr unsigned kBufferGuardBytes = 16;
inline constexpr unsigned char kBufferGuardByte = 0xA5;
// frames * 16 * 2 plus a guard. The extra half is the overflow margin.
unsigned renderBufferBytes(unsigned frames);
void paintBufferGuard(std::uint8_t* data, unsigned used, unsigned total);
bool bufferGuardIntact(const std::uint8_t* data, unsigned used, unsigned total);
// 0 neither, 12 speaker pair, 34 actuator pair. setting 0 auto (actuators only), 1 force 12, 2 force 34.
int hapticChannelPair(int setting, float peak12, float peak34);
std::int16_t floatToPcm16(float value);
}
