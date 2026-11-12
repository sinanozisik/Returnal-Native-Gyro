#include "rg/dualsense_bt.hpp"
#include "rg/remap.hpp"
#include <atomic>
#include <cstring>
namespace rg {
namespace {
std::atomic<std::uint8_t> edgePaddleByte{0};
BluetoothInputTrace inputTrace{};
std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
}
unsigned normalizeBluetoothInput(std::uint8_t* data, unsigned length) {
    if (!data || length < 0x4e || data[0] != 0x31) return length;
    BluetoothInputTrace trace{};
    std::memcpy(trace.btButtons, data + 9, 4);
    const std::uint8_t buttons2 = data[11];
    trace.rawPitch = static_cast<std::int16_t>(data[17] | (static_cast<unsigned>(data[18]) << 8));
    edgePaddleByte.store(buttons2, std::memory_order_relaxed);
    std::memmove(data + 1, data + 2, 63);
    data[0] = 0x01;
    // USB byte 10 keeps PS/touch/mute and takes the Edge nibble from the same BT byte.
    data[10] = static_cast<std::uint8_t>((data[10] & 0x0F) | (buttons2 & 0xF0));
    std::memcpy(trace.usbButtons, data + 8, 4);
    trace.valid = true;
    inputTrace = trace;
    return 0x40;
}
BluetoothInputTrace lastBluetoothInputTrace() { return inputTrace; }
std::uint8_t latestEdgePaddleByte() { return edgePaddleByte.load(std::memory_order_relaxed); }
void setUsbButton(std::uint8_t* usb, unsigned length, int id) {
    auto set = [&](unsigned index, std::uint8_t bit) { if (length > index) usb[index] = static_cast<std::uint8_t>(usb[index] | bit); };
    switch (id) {
    case 1: set(9, 0x01); break;
    case 2: set(9, 0x02); break;
    case 3: set(9, 0x40); break;
    case 4: set(9, 0x80); break;
    case 6: set(8, 0x20); break;
    case 7: set(8, 0x40); break;
    case 10: set(8, 0x10); break;
    case 11: set(8, 0x80); break;
    case 40: set(10, 0x02); break;
    case 41: set(9, 0x20); break;
    case 42: set(9, 0x10); break;
    case 5: if (length > 33) usb[33] = static_cast<std::uint8_t>(usb[33] & 0x7F); break;
    default: break;
    }
}
void applyEdgeRemap(std::uint8_t* usb, unsigned length, const Settings& settings) {
    if (!usb || usb[0] != 0x01 || length < 11) return;
    const std::uint8_t paddles = usb[10];
    const int sources[][2] = {{0x40, 20}, {0x80, 21}, {0x10, 22}, {0x20, 23}};
    for (auto& source : sources) {
        if ((paddles & source[0]) == 0) continue;
        setUsbButton(usb, length, remapOutput(settings, source[1]));
    }
}
bool buildBluetoothOutput(std::uint8_t* dst, const std::uint8_t* src, std::uint32_t srcLen, std::uint8_t seq) {
    if (!dst) return false;
    std::memset(dst, 0, 78);
    unsigned count = 0;
    if (src && srcLen > 1) {
        count = srcLen - 1;
        // 47 bytes is the whole common output block: flags, motors, triggers, lightbar.
        if (count > 47) count = 47;
        std::memcpy(dst + 3, src + 1, count);
    }
    dst[0] = 0x31;
    dst[1] = static_cast<std::uint8_t>((seq & 0x0f) << 4);
    dst[2] = 0x10;
    std::uint8_t seeded[75];
    seeded[0] = 0xA2;
    std::memcpy(seeded + 1, dst, 74);
    const std::uint32_t crc = crc32(seeded, 75);
    std::memcpy(dst + 0x4a, &crc, sizeof crc);
    return true;
}
bool bluetoothOutputCrcValid(const std::uint8_t* report) {
    if (!report || report[0] != 0x31) return false;
    std::uint8_t seeded[75];
    seeded[0] = 0xA2;
    std::memcpy(seeded + 1, report, 74);
    std::uint32_t expect = 0;
    std::memcpy(&expect, report + 0x4a, sizeof expect);
    return crc32(seeded, 75) == expect;
}
void refreshBluetoothOutputCrc(std::uint8_t* report, unsigned size) {
    if (!report || size < 8 || size > 547) return;
    std::uint8_t seeded[548];
    seeded[0] = 0xA2;
    std::memcpy(seeded + 1, report, size - 4);
    const std::uint32_t crc = crc32(seeded, size - 3);
    std::memcpy(report + size - 4, &crc, sizeof crc);
}
}
