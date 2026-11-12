#pragma once
#include "settings.hpp"
#include <cstddef>
#include <cstdint>
namespace rg {
struct BluetoothInputTrace {
    std::uint8_t btButtons[4]{}; // raw 0x31 bytes 9..12: buttons[0..3]
    std::uint8_t usbButtons[4]{}; // USB 0x01 bytes 8..11
    std::int16_t rawPitch{}; // gyro X, little endian, sign unchanged
    bool valid{};
};
// Bluetooth input 0x31 is rewritten as the 64-byte USB report the game already accepts.
// The common payload is not rearranged: USB byte N+1 is Bluetooth byte N+2.
// Edge paddle bits stay on USB byte 10: 0x10 L-Fn, 0x20 R-Fn, 0x40 L4, 0x80 R4.
// Other buffers are left unchanged and the original length is returned.
unsigned normalizeBluetoothInput(std::uint8_t* data, unsigned length);
BluetoothInputTrace lastBluetoothInputTrace();
// Last Edge buttons[2] byte seen in a Bluetooth input report (PS, mute, Fn, paddles).
std::uint8_t latestEdgePaddleByte();
// Sets the mapped face/shoulder/stick bits in a USB 0x01 report. Paddle bits stay.
void applyEdgeRemap(std::uint8_t* usb, unsigned length, const Settings& settings);
// USB output 0x02 becomes a 78-byte Bluetooth 0x31 report. seq is the low nibble.
// USB byte 1 valid_flag0 -> BT 3, byte 2 valid_flag1 -> BT 4, byte 3 right motor -> BT 5, byte 4 left motor -> BT 6.
// CRC32 is seeded with 0xA2.
bool buildBluetoothOutput(std::uint8_t* dst, const std::uint8_t* src, std::uint32_t srcLen, std::uint8_t seq);
bool bluetoothOutputCrcValid(const std::uint8_t* report);
// Rewrites the trailing CRC of a Bluetooth output report. Seed byte is 0xA2.
void refreshBluetoothOutputCrc(std::uint8_t* report, unsigned size);
}
