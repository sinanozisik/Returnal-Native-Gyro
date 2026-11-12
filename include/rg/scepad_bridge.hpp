#pragma once
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <cstdint>
// libScePad also writes Bluetooth 0x31 reports. Leave it off so it does not fight the DualSense module.
#ifndef RG_ENABLE_SCEPAD_BRIDGE
#define RG_ENABLE_SCEPAD_BRIDGE 0
#endif
namespace rg {
// tag must stay valid for the process lifetime. spoofEdgePid rewrites Edge 0x0DF2 as DualSense 0x0CE6.
bool installBridgeForModule(HMODULE module, const char* tag, bool spoofEdgePid);
// Sends one Bluetooth output report on the newest tracked DualSense handle.
// The sequence nibble is filled here and the CRC is stamped again. Report 0x32 is left untouched.
bool sendTrackedBluetoothReport(std::uint8_t* report, unsigned size);
// Path of the newest tracked Bluetooth HID interface, for a private overlapped handle.
bool copyNewestBluetoothPath(wchar_t* path, unsigned capacity);
}
