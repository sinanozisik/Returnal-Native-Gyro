#include "rg/bt_haptic.hpp"
#include "rg/dualsense_bt.hpp"
#include "rg/settings.hpp"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
int main() {
    try {
        rg::HapticDownsampler sampler;
        std::int8_t left = 0, right = 0;
        bool produced = false;
        for (int i = 0; i < 16; ++i) produced = sampler.push(2560, -2560, left, right);
        check(produced && left == 10 && right == -10, "sixteen frames average down to 3 kHz");
        std::array<std::int8_t, 64> samples{};
        for (int i = 0; i < 32; ++i) { samples[i * 2] = 10; samples[i * 2 + 1] = -20; }
        std::array<std::uint8_t, 142> report{};
        check(rg::buildBluetoothHapticReport(report.data(), samples.data(), 3, 9), "haptic report builds");
        check(report[0] == 0x32 && report[1] == 0 && report[2] == 0x91 && report[3] == 0x07 && report[4] == 0xFE, "haptic header");
        check(report[5] == 0 && report[9] == 0xFF && report[10] == 9 && report[11] == 0x92 && report[12] == 0x40, "saxense counter and packet tag");
        check(report[13] == 10 && report[14] == static_cast<std::uint8_t>(-20), "first stereo sample");
        check(rg::hapticReportCrcValid(report.data()), "haptic crc uses the 0xA2 seed");
        std::array<std::uint8_t, 78> mode{};
        check(rg::buildBluetoothHapticModeReport(mode.data(), 1), "audio mode report builds");
        check(mode[0] == 0x31 && mode[1] == 0x10 && mode[2] == 0x10 && mode[3] == 0 && mode[4] == 0x20, "default mode is the low-pass flag only");
        bool speakerFieldsClear = mode[8] == 0 && mode[10] == 0 && mode[40] == 0;
        for (int i = 5; i < 42; ++i) if (mode[static_cast<std::size_t>(i)] != 0) speakerFieldsClear = false;
        check(speakerFieldsClear, "speaker off leaves the 0x31 body identical to the haptic-only report");
        check((mode[42] & 0x01) != 0, "low-pass bit is on");
        check(rg::buildBluetoothHapticModeReport(mode.data(), 1, true) && mode[4] == (0x02 | 0x20 | 0x40), "full mode sets flag1 0x62");
        check(rg::bluetoothOutputCrcValid(mode.data()), "audio mode crc matches output reports");
        std::array<std::uint8_t, 78> game{};
        game[0] = 0x31;
        game[3] = 0x0d;
        game[4] = 0x1c;
        game[12] = 0x84;
        game[39] = 0x17;
        game[42] = 0;
        game[47] = 0x40;
        rg::applyBluetoothHapticMode(game.data());
        check(game[3] == 0x0c && (game[4] & 0x20) != 0 && (game[4] & 0x42) == 0, "lpf merge sets flag1 0x20 and clears the full-power bits");
        check((game[39] & 0x10) != 0 && (game[42] & 0x01) != 0 && game[12] == 0x84, "lpf leaves trigger attenuation and the lightbar power byte");
        rg::applyBluetoothHapticMode(game.data(), true);
        check((game[4] & 0x62) == 0x62 && (game[12] & 0x84) == 0 && (game[39] & 0x07) == 0, "full merge restores flag1 0x62 and clears haptic mute");
        check(game[47] == 0x40, "lightbar survives the haptic merge");
        const unsigned frames = 4;
        const unsigned total = rg::renderBufferBytes(frames);
        check(total == frames * 16u * 2u + 16u, "float32 4ch buffer is two blocks plus a guard");
        std::vector<std::uint8_t> buffer(total, 0);
        const unsigned used = frames * 16u * 2u;
        rg::paintBufferGuard(buffer.data(), used, total);
        check(rg::bufferGuardIntact(buffer.data(), used, total), "fresh guard is intact");
        float quad[4] = {1.f, -1.f, 0.5f, -0.25f};
        std::memcpy(buffer.data(), quad, sizeof quad);
        check(rg::floatToPcm16(1.f) == 32767 && rg::floatToPcm16(-1.f) == -32767 && rg::floatToPcm16(2.f) == 32767, "float samples clamp into the int16 range");
        check(rg::hapticChannelPair(0, 0.00026f, 0.f) == 0, "auto does not fall back to the speaker pair");
        check(rg::hapticChannelPair(0, 0.f, 0.2f) == 34 && rg::hapticChannelPair(0, 0.2f, 0.2f) == 34, "auto uses the rear pair when it has signal");
        check(rg::hapticChannelPair(1, 0.f, 0.9f) == 12 && rg::hapticChannelPair(2, 0.9f, 0.f) == 34, "ini can force either pair");
        check(rg::Settings{}.BtHapticsChannelPair == 2, "rear pair is the default");
        check(rg::parseConfig("BtHapticsChannelPair=auto\n").settings.BtHapticsChannelPair == 0, "auto stays available");
        check(rg::parseConfig("BtHapticsChannelPair=12\n").settings.BtHapticsChannelPair == 1, "12 selects the front pair");
        check(rg::parseConfig("BtHapticsChannelPair=34\n").settings.BtHapticsChannelPair == 2, "34 selects the rear pair");
        check(rg::parseConfig("BtHapticsModeFlags=lpf\n").settings.BtHapticsModeFlags == 0 && rg::parseConfig("BtHapticsModeFlags=full\n").settings.BtHapticsModeFlags == 1, "mode flags parse lpf and full");
        check(rg::Settings{}.BtHapticsGain == 1, "haptic gain defaults to unity");
        check(rg::Settings{}.BtSpeaker && rg::Settings{}.BtSpeakerReport && rg::Settings{}.BtSpeakerFlags31 && rg::Settings{}.BtSpeakerCounterStyle == 0 && rg::Settings{}.BtSpeakerGain == 1 && rg::Settings{}.BtSpeakerVolume == 100, "speaker defaults to on, report on, 0x31 flags on, 0x32 counter, unity gain, PS5-max volume");
        check(rg::parseConfig("BtSpeakerCounterStyle=0x35\n").settings.BtSpeakerCounterStyle == 1, "0x35 selects the old byte9 counter");
        check(rg::parseConfig("BtSpeakerCounterStyle=0x32\n").settings.BtSpeakerCounterStyle == 0, "0x32 keeps the haptic counter");
        check(rg::hapticBacklogDrop(32) == 0 && rg::hapticBacklogDrop(64) == 0 && rg::hapticBacklogDrop(160) == 96, "a haptic queue above two blocks drops the stale head");
        const auto speakerOff = rg::parseConfig("BtSpeakerReport=0\nBtSpeakerFlags31=0\n").settings;
        check(!speakerOff.BtSpeakerReport && !speakerOff.BtSpeakerFlags31, "speaker report and 0x31 flags can be turned off");
        std::array<std::uint8_t, 200> opus{};
        opus[0] = 0x11;
        std::array<std::uint8_t, 334> speaker{};
        check(rg::buildBluetoothSpeakerReport(speaker.data(), 3, 4, opus.data()), "speaker report builds");
        check(speaker[0] == 0x35 && speaker[1] == 0x30 && speaker[2] == 0x91 && speaker[3] == 0x07 && speaker[4] == 0x7E, "speaker header");
        check(speaker[9] == 0xFF && speaker[10] == 4 && speaker[11] == 0x93 && speaker[12] == 0xC8 && speaker[13] == 0x11 && speaker[213] == 0, "default counter matches report 0x32");
        check(rg::speakerReportCrcValid(speaker.data()), "speaker crc uses the 0xA2 seed");
        check(rg::buildBluetoothSpeakerReport(speaker.data(), 3, 4, opus.data(), nullptr, false), "legacy counter report builds");
        check(speaker[9] == 4 && speaker[10] == 0, "0x35 counter style still counts on byte 9");
        check(rg::buildBluetoothSpeakerReport(speaker.data(), 3, 4, opus.data(), samples.data()), "combined report builds");
        check(speaker[0] == 0x35 && speaker[4] == 0xFE && speaker[9] == 0xFF && speaker[10] == 4 && speaker[11] == 0x93 && speaker[13] == 0x11, "combined report keeps one 0x11 and the opus prefix");
        check(speaker[213] == 0x92 && speaker[214] == 0x40 && speaker[215] == 10 && speaker[216] == static_cast<std::uint8_t>(-20) && speaker[279] == 0, "one 0x12 frame follows the opus frame");
        check(rg::speakerReportCrcValid(speaker.data()), "combined crc covers both frames with the 0xA2 seed");
        check(report[0] == 0x32 && report[1] == 0 && report[11] == 0x92 && report[12] == 0x40, "haptic report 0x32 is unchanged");
        std::array<std::uint8_t, 78> both = mode;
        check(rg::buildBluetoothHapticModeReport(both.data(), 1), "lpf report rebuilds");
        rg::applyBluetoothSpeaker(both.data(), 0x3d);
        check(both[3] == 0xf0 && both[4] == 0xa0 && both[7] == 0x3d && both[8] == 0x3d && both[10] == 0x30 && both[40] == 0x05 && both[42] == 0x01, "speaker uses path 3, preamp 5, and the volume enables");
        check((both[3] & 0x03) == 0, "speaker merge does not set the rumble bits");
        std::array<std::uint8_t, 78> kept{};
        kept[0] = 0x31;
        kept[3] = 0x0c;
        kept[4] = 0x60;
        kept[10] = 0x0c;
        kept[42] = 0x01;
        rg::applyBluetoothSpeaker(kept.data(), 0x3d);
        check(kept[3] == 0xfc && (kept[3] & 0x07) == 0x04, "speaker merge keeps flag0 0x04 and does not set rumble");
        check(kept[4] == 0xe0 && (kept[4] & 0x40) != 0 && (kept[4] & 0x20) != 0, "speaker merge keeps flag1 0x40 and the low-pass enable");
        check(kept[10] == 0x3c && kept[42] == 0x01, "speaker path replaces only the route bits and keeps the haptic byte");
        buffer.back() = 0;
        check(!rg::bufferGuardIntact(buffer.data(), used, total), "a smashed guard is reported instead of crashing");
        std::cout << "Bluetooth haptic report packing passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
