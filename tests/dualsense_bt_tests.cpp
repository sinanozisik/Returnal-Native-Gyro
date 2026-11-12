#include "rg/dualsense_bt.hpp"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
int main() {
    try {
        std::array<std::uint8_t, 78> input{};
        input[0] = 0x31;
        input[1] = 0x10;
        input[2] = 0xAB;
        input[11] = 0xF0; // buttons[2]: L5 R5 L4 R4
        input[17] = 0xFE; // gyro X = -2, little endian, must keep its sign
        input[18] = 0xFF;
        input[20] = 0x5A;
        check(rg::normalizeBluetoothInput(input.data(), 78) == 64, "bluetooth input becomes 64 bytes");
        check(input[0] == 0x01 && input[1] == 0xAB && input[19] == 0x5A, "payload shifts by one and keeps the marker");
        check(input[10] == 0xF0 && rg::latestEdgePaddleByte() == 0xF0, "edge paddle bits land on USB byte 10");
        check(input[16] == 0xFE && input[17] == 0xFF, "gyro X keeps its little-endian sign");
        check(rg::lastBluetoothInputTrace().rawPitch == -2, "raw pitch keeps the sensor sign");
        rg::Settings map;
        map.MapL4 = 3;
        map.GyroButton = 21;
        input[10] = 0x41; // PS + L4, R4 clear
        rg::applyEdgeRemap(input.data(), 64, map);
        check((input[9] & 0x40) != 0 && (input[10] & 0x40) != 0 && (input[10] & 0x80) == 0, "L4 maps to L3 and the paddle bit stays");
        std::array<std::uint8_t, 16> usb{};
        usb[0] = 0x01;
        check(rg::normalizeBluetoothInput(usb.data(), 16) == 16 && usb[0] == 0x01, "usb input stays untouched");

        std::array<std::uint8_t, 48> output{};
        output[0] = 0x02;
        output[1] = 0x03; // compatible vibration + haptics select
        output[2] = 0x04;
        output[3] = 0x40; // right motor
        output[4] = 0x7F; // left motor
        output[10] = 0x22;
        std::array<std::uint8_t, 78> bluetooth{};
        check(rg::buildBluetoothOutput(bluetooth.data(), output.data(), 48, 3), "output builds");
        check(bluetooth[0] == 0x31 && bluetooth[1] == 0x30 && bluetooth[2] == 0x10, "sequence and tag");
        check(bluetooth[3] == 0x03 && bluetooth[4] == 0x04 && bluetooth[5] == 0x40 && bluetooth[6] == 0x7F && bluetooth[12] == 0x22, "flags and rumble motors stay in the usb body");
        check(rg::bluetoothOutputCrcValid(bluetooth.data()), "crc covers the bluetooth output");
        std::cout << "DualSense Bluetooth report translation passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
