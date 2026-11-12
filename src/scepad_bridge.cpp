#include "rg/scepad_bridge.hpp"
#include "rg/bt_haptic.hpp"
#include "rg/dualsense_bt.hpp"
#include "rg/runtime.hpp"
#include "rg/stall_watch.hpp"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cwctype>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
namespace rg {
namespace {
struct HidAttributes { unsigned long size; unsigned short vendor, product, version; };
using CreateFileWFn = HANDLE(WINAPI*)(LPCWSTR, DWORD, DWORD, LPSECURITY_ATTRIBUTES, DWORD, DWORD, HANDLE);
using ReadFileFn = BOOL(WINAPI*)(HANDLE, LPVOID, DWORD, LPDWORD, LPOVERLAPPED);
using WriteFileFn = BOOL(WINAPI*)(HANDLE, LPCVOID, DWORD, LPDWORD, LPOVERLAPPED);
using CloseHandleFn = BOOL(WINAPI*)(HANDLE);
using GetAttributesFn = BOOLEAN(WINAPI*)(HANDLE, HidAttributes*);
using SetFeatureFn = BOOLEAN(WINAPI*)(HANDLE, PVOID, ULONG);
using GetFeatureFn = BOOLEAN(WINAPI*)(HANDLE, PVOID, ULONG);
using GetPreparsedFn = BOOLEAN(WINAPI*)(HANDLE, void**);
using FreePreparsedFn = BOOLEAN(WINAPI*)(void*);
using GetCapsFn = long(WINAPI*)(void*, void*);
using GetValueCapsFn = long(WINAPI*)(int, unsigned char*, unsigned long*, void*);
using GetProductFn = BOOLEAN(WINAPI*)(HANDLE, PVOID, ULONG);
using WaitFn = DWORD(WINAPI*)(HANDLE, DWORD);
using WaitExFn = DWORD(WINAPI*)(HANDLE, DWORD, BOOL);
struct HidCaps {
    unsigned short usage, usagePage, inputLength, outputLength, featureLength;
    unsigned short reserved[17];
    unsigned short linkNodes, inputButtons, inputValues, inputIndices;
    unsigned short outputButtons, outputValues, outputIndices;
    unsigned short featureButtons, featureValues, featureIndices;
};
constexpr int kSlots = 4;
constexpr int kDetail = 40;
constexpr int kDump = 5;
constexpr int kOverlapLogs = 20;
constexpr ULONG_PTR kIoPending = 0x103;
enum Kind { KCreate, KClose, KAttr, KProduct, KCaps, KValue, KGet, KSet, KWrite, KRead, KCount };
struct Slot {
    HMODULE module{};
    const char* tag{};
    bool spoofEdgePid{};
    std::uintptr_t base{};
    std::uintptr_t end{};
    std::atomic<int> phase{0};
    std::atomic<int> createBefore{0};
    CreateFileWFn originalCreate{};
    ReadFileFn originalRead{};
    WriteFileFn originalWrite{};
    CloseHandleFn originalClose{};
    GetAttributesFn originalAttributes{};
    SetFeatureFn originalFeature{};
    GetFeatureFn originalGetFeature{};
    GetPreparsedFn originalPreparsed{};
    FreePreparsedFn originalFreePreparsed{};
    GetCapsFn originalCaps{};
    GetValueCapsFn originalValueCaps{};
    GetProductFn originalProduct{};
    WaitFn originalWait{};
    WaitExFn originalWaitEx{};
};
struct Tracked {
    std::uint8_t seq{};
    std::uint32_t generation{};
    const char* tag{};
    ULONGLONG openedTick{};
    int inputDumps{};
    int overlapLogs{};
    std::wstring path;
};
struct Pending {
    int type{};
    HANDLE device{};
    HANDLE key{};
    OVERLAPPED* overlapped{};
    std::uint8_t* readBuffer{};
    LPDWORD readCount{};
    DWORD capacity{};
    DWORD callerSize{};
    std::unique_ptr<std::uint8_t[]> writeBuf;
    const char* tag{};
    std::atomic<bool> accept{false};
    std::atomic<bool> settled{false};
};
struct RateKey {
    std::uint64_t id{};
    int kind{};
    bool operator==(const RateKey& other) const { return id == other.id && kind == other.kind; }
};
struct RateKeyHash {
    std::size_t operator()(const RateKey& key) const { return std::hash<std::uint64_t>{}(key.id) ^ static_cast<std::size_t>(key.kind); }
};
struct Rate {
    int shown{};
    int extra{};
    ULONGLONG marked{};
    const char* tag{};
    const char* kind{};
    HANDLE handle{};
};
Slot slots[kSlots];
std::atomic<int> slotCount{0};
std::mutex handlesMutex;
std::recursive_mutex outputMutex;
std::unordered_map<HANDLE, Tracked> bluetoothHandles;
std::unordered_map<void*, HANDLE> preparsedHandles;
std::uint32_t generationCounter{};
std::mutex ioMutex;
std::unordered_map<HANDLE, std::shared_ptr<Pending>> pendingByKey;
std::mutex rateMutex;
std::unordered_map<RateKey, Rate, RateKeyHash> rates;
std::mutex bridgedMutex;
std::string bridgedModules;

void note(const std::string& text) { if (runtime) runtime->log(text); }
unsigned translateInput(std::uint8_t* data, unsigned length) {
    const unsigned produced = normalizeBluetoothInput(data, length);
    if (runtime && data && produced >= 11 && data[0] == 0x01) applyEdgeRemap(data, produced, runtime->snapshot());
    return produced;
}
std::string hexByte(std::uint8_t value) {
    static const char* digits = "0123456789abcdef";
    std::string text;
    text.push_back(digits[value >> 4]);
    text.push_back(digits[value & 0x0f]);
    return text;
}
std::string hexPtr(const void* value) {
    std::ostringstream out;
    out << std::hex << reinterpret_cast<std::uintptr_t>(value);
    return out.str();
}
std::string hexNum(unsigned long value) {
    std::ostringstream out;
    out << std::hex << value;
    return out.str();
}
std::string hexPrefix(const std::uint8_t* data, unsigned length) {
    static const char* digits = "0123456789abcdef";
    const unsigned count = std::min(length, 16u);
    std::string text;
    text.reserve(count * 3);
    for (unsigned i = 0; i < count; ++i) {
        if (i) text.push_back(' ');
        text.push_back(digits[data[i] >> 4]);
        text.push_back(digits[data[i] & 0x0f]);
    }
    return text;
}
std::wstring lowerPath(LPCWSTR path) {
    if (!path) return {};
    std::wstring text(path);
    std::transform(text.begin(), text.end(), text.begin(), ::towlower);
    return text;
}
bool contains(const std::wstring& text, const wchar_t* piece) { return text.find(piece) != std::wstring::npos; }
bool sonyBluetoothPath(const std::wstring& path) {
    const bool sony = contains(path, L"054c");
    const bool pad = contains(path, L"0ce6") || contains(path, L"0df2");
    const bool bluetooth = contains(path, L"bthenum") || contains(path, L"00001124-0000-1000-8000-00805f9b34fb") || contains(path, L"vid&0002054c");
    return sony && pad && bluetooth;
}
std::string narrow(const std::wstring& text) {
    if (text.empty()) return {};
    const int size = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
    if (size <= 0) return {};
    std::string out(static_cast<std::size_t>(size), '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), out.data(), size, nullptr, nullptr);
    return out;
}
bool keepToken(const std::string& token) {
    std::string lower(token.size(), '\0');
    std::transform(token.begin(), token.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower == "054c" || lower == "0ce6" || lower == "0df2" || lower == "0002054c" || lower == "00020ce6" || lower == "00020df2";
}
std::string maskPath(const std::wstring& path) {
    const std::string text = narrow(path);
    std::string out;
    bool brace = false;
    for (std::size_t i = 0; i < text.size();) {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        if (c == '{') brace = true;
        if (c == '}') { brace = false; out.push_back(text[i++]); continue; }
        if (!brace && std::isxdigit(c)) {
            std::size_t j = i;
            while (j < text.size() && std::isxdigit(static_cast<unsigned char>(text[j]))) ++j;
            const std::string token = text.substr(i, j - i);
            if (!keepToken(token) && token.size() >= 8) out += "****";
            else out += token;
            i = j;
            continue;
        }
        out.push_back(text[i++]);
    }
    return out;
}
Slot* slotFrom(void* ret) {
    const auto addr = reinterpret_cast<std::uintptr_t>(ret);
    const int count = slotCount.load(std::memory_order_acquire);
    for (int i = 0; i < count; ++i) if (addr >= slots[i].base && addr < slots[i].end) return &slots[i];
    return count ? &slots[0] : nullptr;
}
template<class Fn>
Fn originalOf(Fn Slot::* member) {
    const int count = slotCount.load(std::memory_order_acquire);
    for (int i = 0; i < count; ++i) if (slots[i].*member) return slots[i].*member;
    return nullptr;
}
bool copyTracked(HANDLE handle, Tracked& out) {
    std::lock_guard lock(handlesMutex);
    const auto found = bluetoothHandles.find(handle);
    if (found == bluetoothHandles.end()) return false;
    out = found->second;
    return true;
}
void remember(HANDLE handle, const char* tag, const std::wstring& path) {
    std::lock_guard lock(handlesMutex);
    Tracked device;
    device.generation = ++generationCounter;
    device.tag = tag;
    device.openedTick = GetTickCount64();
    device.path = path;
    bluetoothHandles.emplace(handle, device);
}
bool forget(HANDLE handle, Tracked& out) {
    std::lock_guard lock(handlesMutex);
    const auto found = bluetoothHandles.find(handle);
    if (found == bluetoothHandles.end()) return false;
    out = found->second;
    bluetoothHandles.erase(found);
    return true;
}
std::uint8_t nextSequence(HANDLE handle) {
    std::lock_guard lock(handlesMutex);
    auto& device = bluetoothHandles[handle];
    const std::uint8_t current = device.seq;
    device.seq = static_cast<std::uint8_t>((device.seq + 1) & 0x0f);
    return current;
}
int claimDump(HANDLE handle) {
    std::lock_guard lock(handlesMutex);
    const auto found = bluetoothHandles.find(handle);
    if (found == bluetoothHandles.end() || found->second.inputDumps >= kDump) return -1;
    return found->second.inputDumps++;
}
bool claimOverlap(HANDLE handle) {
    std::lock_guard lock(handlesMutex);
    const auto found = bluetoothHandles.find(handle);
    if (found == bluetoothHandles.end() || found->second.overlapLogs >= kOverlapLogs) return false;
    ++found->second.overlapLogs;
    return true;
}
std::uint64_t rateId(HANDLE handle, std::uint32_t generation) {
    if (generation) return generation;
    return 0x8000000000000000ull | reinterpret_cast<std::uintptr_t>(handle);
}
void eraseRates(std::uint64_t id) {
    std::lock_guard lock(rateMutex);
    for (int kind = 0; kind < KCount; ++kind) rates.erase(RateKey{id, kind});
}
bool allowDetail(HANDLE handle, std::uint32_t generation, Kind kind, const char* tag, const char* kindName, bool force, std::string& summary) {
    std::lock_guard lock(rateMutex);
    auto& rate = rates[RateKey{rateId(handle, generation), kind}];
    rate.tag = tag;
    rate.kind = kindName;
    rate.handle = handle;
    summary.clear();
    const auto flush = [&] {
        if (!rate.extra) return;
        summary = std::string(rate.tag) + " " + rate.kind + " handle=" + hexPtr(rate.handle) + " +" + std::to_string(rate.extra) + "/s";
        rate.extra = 0;
        rate.marked = GetTickCount64();
    };
    if (rate.shown < kDetail) { ++rate.shown; return true; }
    const auto now = GetTickCount64();
    if (!rate.marked) rate.marked = now;
    if (now - rate.marked >= 1000) flush();
    if (force) return true;
    ++rate.extra;
    if (now - rate.marked >= 1000) flush();
    return false;
}
void logEvent(HANDLE handle, std::uint32_t generation, Kind kind, const char* tag, const char* kindName, const std::string& line, bool force = false) {
    std::string summary;
    const bool detail = allowDetail(handle, generation, kind, tag, kindName, force, summary);
    if (detail) note(std::string(tag) + " " + line);
    if (!summary.empty()) note(summary);
}
void rememberPending(const std::shared_ptr<Pending>& pending) {
    std::lock_guard lock(ioMutex);
    pendingByKey[pending->key] = pending;
}
std::shared_ptr<Pending> settlePending(HANDLE key, bool requireAccept) {
    std::lock_guard lock(ioMutex);
    const auto found = pendingByKey.find(key);
    if (found == pendingByKey.end()) return {};
    const auto pending = found->second;
    if (requireAccept && !pending->accept.load(std::memory_order_acquire)) return {};
    if (!pending->overlapped || pending->overlapped->Internal == kIoPending) return {};
    if (pending->settled.exchange(true)) return {};
    pendingByKey.erase(found);
    return pending;
}
void dropPending(HANDLE key) {
    std::lock_guard lock(ioMutex);
    const auto found = pendingByKey.find(key);
    if (found == pendingByKey.end()) return;
    found->second->settled.store(true);
    pendingByKey.erase(found);
}
void logInput(const char* tag, HANDLE handle, DWORD requested, unsigned rawLength, unsigned normalized, bool overlapped, DWORD error, bool ok, const std::uint8_t* raw, const std::uint8_t* cooked) {
    Tracked device{};
    const bool known = copyTracked(handle, device);
    const char* module = known && device.tag ? device.tag : tag;
    const int dump = ok ? claimDump(handle) : -1;
    const bool force = !ok || (overlapped && claimOverlap(handle));
    std::string line = "readFile handle=" + hexPtr(handle) + " id=" + std::to_string(raw && rawLength ? raw[0] : 0) +
        " len=" + std::to_string(rawLength) + "->" + std::to_string(normalized) + " size=" + std::to_string(requested) +
        " ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error) + " overlapped=" + (overlapped ? "1" : "0");
    if (dump >= 0) line += " raw=" + hexPrefix(raw, rawLength) + " usb=" + hexPrefix(cooked, normalized);
    if (ok && cooked && cooked[0] == 0x01) {
        const auto trace = lastBluetoothInputTrace();
        if (trace.valid) {
            static std::uint8_t seenNibble = 0xFF;
            const std::uint8_t paddleBt = trace.btButtons[2];
            if ((paddleBt & 0xF0) != (seenNibble & 0xF0)) {
                seenNibble = paddleBt;
                const std::uint8_t paddleUsb = trace.usbButtons[2];
                note(std::string(module) + " Edge paddle bt[11]=" + hexByte(paddleBt) + " usb[10]=" + hexByte(paddleUsb)
                    + " L4=" + ((paddleUsb & 0x40) ? "1" : "0") + " R4=" + ((paddleUsb & 0x80) ? "1" : "0")
                    + " LFn=" + ((paddleUsb & 0x10) ? "1" : "0") + " RFn=" + ((paddleUsb & 0x20) ? "1" : "0"));
            }
            static int pitchSign = 0;
            static int pitchLogs = 0;
            const int pitchMagnitude = trace.rawPitch < 0 ? -trace.rawPitch : trace.rawPitch;
            if (pitchMagnitude >= 2000 && pitchLogs < 40) {
                const int sign = trace.rawPitch < 0 ? -1 : 1;
                if (sign != pitchSign) {
                    pitchSign = sign;
                    ++pitchLogs;
                    note(std::string(module) + " rawPitch=" + std::to_string(trace.rawPitch));
                }
            }
        }
    }
    logEvent(handle, known ? device.generation : 0, KRead, module, "readFile", line, force);
}
unsigned finishRead(const char* tag, HANDLE handle, std::uint8_t* buffer, DWORD capacity, DWORD reported, OVERLAPPED* overlapped, LPDWORD readCount, bool wasOverlapped) {
    unsigned count = reported;
    if (overlapped && overlapped->InternalHigh) count = static_cast<unsigned>(overlapped->InternalHigh);
    if (count > capacity) count = capacity;
    std::uint8_t raw[16]{};
    const unsigned rawCopy = std::min(count, 16u);
    if (buffer && rawCopy) std::memcpy(raw, buffer, rawCopy);
    unsigned normalized = count;
    if (buffer && count >= 0x4e && capacity >= 0x4e) normalized = translateInput(buffer, count);
    if (overlapped) overlapped->InternalHigh = normalized;
    if (readCount) *readCount = normalized;
    logInput(tag, handle, capacity, count, normalized, wasOverlapped, 0, true, raw, buffer);
    return normalized;
}
void finishWrite(const std::shared_ptr<Pending>& pending, BOOL ok, DWORD error) {
    if (ok && pending->overlapped) pending->overlapped->InternalHigh = pending->callerSize;
    Tracked device{};
    const bool known = copyTracked(pending->device, device);
    const char* tag = known && device.tag ? device.tag : pending->tag;
    logEvent(pending->device, known ? device.generation : 0, KWrite, tag, "WriteFile",
        "WriteFile handle=" + hexPtr(pending->device) + " id=2->31 size=" + std::to_string(pending->callerSize) + "->78 ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error) + " overlapped=1", ok == FALSE);
}
void completeIo(HANDLE key) {
    const auto pending = settlePending(key, true);
    if (!pending) return;
    const bool success = pending->overlapped->Internal < 0x80000000ull;
    if (pending->type == 0) {
        if (success) finishRead(pending->tag, pending->device, pending->readBuffer, pending->capacity, static_cast<DWORD>(pending->overlapped->InternalHigh), pending->overlapped, pending->readCount, true);
        else logInput(pending->tag, pending->device, pending->capacity, 0, 0, true, static_cast<DWORD>(pending->overlapped->Internal), false, nullptr, nullptr);
    } else finishWrite(pending, success ? TRUE : FALSE, success ? 0 : static_cast<DWORD>(pending->overlapped->Internal));
}
HANDLE WINAPI createFile(LPCWSTR path, DWORD access, DWORD share, LPSECURITY_ATTRIBUTES security, DWORD disposition, DWORD flags, HANDLE templateFile) {
    Slot* slot = slotFrom(_ReturnAddress());
    if (slot && slot->phase.load(std::memory_order_acquire) != 2) slot->createBefore.fetch_add(1, std::memory_order_relaxed);
    auto original = slot && slot->originalCreate ? slot->originalCreate : originalOf(&Slot::originalCreate);
    if (!original) return INVALID_HANDLE_VALUE;
    const HANDLE handle = original(path, access, share, security, disposition, flags, templateFile);
    const DWORD error = GetLastError();
    const std::wstring lowered = lowerPath(path);
    const bool watch = handle != INVALID_HANDLE_VALUE && sonyBluetoothPath(lowered);
    const char* tag = slot && slot->tag ? slot->tag : "bridge";
    if (watch) remember(handle, tag, path ? path : L"");
    const bool sonyPad = contains(lowered, L"054c") && (contains(lowered, L"0ce6") || contains(lowered, L"0df2"));
    if (handle != INVALID_HANDLE_VALUE && sonyPad) {
        static int transports = 0;
        if (transports < 6) {
            ++transports;
            const bool radio = contains(lowered, L"bthenum") || contains(lowered, L"00001124-0000-1000-8000-00805f9b34fb") || contains(lowered, L"vid&0002054c");
            note(std::string(tag) + " connection candidate CreateFile transport=" + (radio ? "bluetooth" : "usb") + " path=" + maskPath(lowered));
        }
    }
    Tracked device{};
    const bool known = watch && copyTracked(handle, device);
    logEvent(handle, known ? device.generation : 0, KCreate, tag, "CreateFileW",
        "CreateFileW handle=" + hexPtr(handle) + " tracked=" + (watch ? "1" : "0") + " ok=" + (handle != INVALID_HANDLE_VALUE ? "1" : "0") + " err=" + std::to_string(handle != INVALID_HANDLE_VALUE ? 0 : error) + " path=" + maskPath(lowered));
    if (handle == INVALID_HANDLE_VALUE) SetLastError(error);
    return handle;
}
BOOL WINAPI closeHandle(HANDLE handle) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalClose ? slot->originalClose : originalOf(&Slot::originalClose);
    Tracked device{};
    const bool watch = forget(handle, device);
    if (watch) {
        const auto life = GetTickCount64() - device.openedTick;
        note(std::string(device.tag ? device.tag : "bridge") + " CloseHandle handle=" + hexPtr(handle) + " life=" + std::to_string(life) + "ms");
        eraseRates(device.generation);
    } else eraseRates(rateId(handle, 0));
    if (!original) return FALSE;
    return original(handle);
}
BOOL WINAPI readFile(HANDLE handle, LPVOID buffer, DWORD size, LPDWORD read, LPOVERLAPPED overlapped) {
    Stall stall("HID ReadFile");
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalRead ? slot->originalRead : originalOf(&Slot::originalRead);
    if (!original) return FALSE;
    const char* tag = slot && slot->tag ? slot->tag : "bridge";
    Tracked device{};
    const bool watch = copyTracked(handle, device);
    if (watch && buffer && overlapped) {
        auto pending = std::make_shared<Pending>();
        pending->type = 0;
        pending->device = handle;
        pending->key = overlapped->hEvent ? overlapped->hEvent : handle;
        pending->overlapped = overlapped;
        pending->readBuffer = static_cast<std::uint8_t*>(buffer);
        pending->readCount = read;
        pending->capacity = size;
        pending->tag = tag;
        rememberPending(pending);
        const BOOL ok = original(handle, buffer, size, read, overlapped);
        const DWORD error = GetLastError();
        pending->accept.store(true, std::memory_order_release);
        if (ok || error != ERROR_IO_PENDING) {
            if (!ok) {
                dropPending(pending->key);
                logInput(tag, handle, size, 0, 0, true, error, false, nullptr, nullptr);
                SetLastError(error);
                return FALSE;
            }
            if (settlePending(pending->key, false))
                finishRead(tag, handle, pending->readBuffer, size, read ? *read : static_cast<DWORD>(overlapped->InternalHigh), overlapped, read, true);
            return TRUE;
        }
        if (overlapped->Internal != kIoPending) {
            if (settlePending(pending->key, false))
                finishRead(tag, handle, pending->readBuffer, size, static_cast<DWORD>(overlapped->InternalHigh), overlapped, read, true);
        }
        SetLastError(ERROR_IO_PENDING);
        return FALSE;
    }
    if (watch && buffer && !overlapped && size >= 64 && size < 78) {
        std::uint8_t full[78];
        DWORD got = 0;
        BOOL wide = original(handle, full, sizeof full, &got, nullptr);
        DWORD error = GetLastError();
        if (!wide) {
            wide = original(handle, buffer, size, read, nullptr);
            error = GetLastError();
            if (!wide) {
                logInput(tag, handle, size, 0, 0, false, error, false, nullptr, nullptr);
                SetLastError(error);
                return FALSE;
            }
            const unsigned count = read ? *read : 0;
            std::uint8_t raw[16]{};
            if (count) std::memcpy(raw, buffer, std::min<unsigned>(count, 16));
            const unsigned normalized = translateInput(static_cast<std::uint8_t*>(buffer), count);
            if (read) *read = normalized;
            logInput(tag, handle, size, count, normalized, false, 0, true, raw, static_cast<std::uint8_t*>(buffer));
            return TRUE;
        }
        std::uint8_t raw[16]{};
        std::memcpy(raw, full, std::min<DWORD>(got, 16));
        const unsigned normalized = translateInput(full, got);
        const unsigned copy = std::min<unsigned>(normalized, size);
        std::memcpy(buffer, full, copy);
        if (read) *read = copy;
        logInput(tag, handle, size, got, copy, false, 0, true, raw, static_cast<std::uint8_t*>(buffer));
        return TRUE;
    }
    const BOOL ok = original(handle, buffer, size, read, overlapped);
    const DWORD error = GetLastError();
    if (watch) {
        const unsigned count = ok && read ? *read : 0;
        std::uint8_t raw[16]{};
        if (ok && buffer && count) std::memcpy(raw, buffer, std::min<unsigned>(count, 16));
        unsigned normalized = count;
        if (ok && buffer) normalized = translateInput(static_cast<std::uint8_t*>(buffer), count);
        if (ok && read && normalized != count) *read = normalized;
        if (ok && overlapped && normalized != count) overlapped->InternalHigh = normalized;
        logInput(tag, handle, size, count, normalized, overlapped != nullptr, error, ok == TRUE, raw, static_cast<std::uint8_t*>(buffer));
        if (!ok) SetLastError(error);
    }
    return ok;
}
void logWriteHex(const char* tag, const std::uint8_t* usb, DWORD usbLen, const std::uint8_t* bluetooth) {
    static int noted = 0;
    if (!usb || !bluetooth || usbLen < 5 || noted >= 5) return;
    ++noted;
    note(std::string(tag) + " WriteFile #" + std::to_string(noted) + " flag0=" + hexNum(usb[1]) + "->" + hexNum(bluetooth[3]) + " flag1=" + hexNum(usb[2]) + "->" + hexNum(bluetooth[4]) + " right=" + hexNum(usb[3]) + "->" + hexNum(bluetooth[5]) + " left=" + hexNum(usb[4]) + "->" + hexNum(bluetooth[6]) + " crc=" + (bluetoothOutputCrcValid(bluetooth) ? "1" : "0") + " usb=" + hexPrefix(usb, usbLen) + " bt=" + hexPrefix(bluetooth, 16));
}
void keepAudioMode(std::uint8_t* report) {
    if (!report || report[0] != 0x31 || !runtime) return;
    const auto settings = runtime->snapshot();
    const bool haptic = hapticStreamActive() && settings.BtHapticsModeMerge;
    const bool speaker = speakerStreamActive() && settings.BtSpeaker && settings.BtSpeakerFlags31;
    if (!haptic && !speaker) return;
    const std::uint8_t flag0 = report[3], flag1 = report[4], power = report[12], reduce = report[39], hapticByte = report[42];
    const std::uint8_t audio = report[10], audio2 = report[40], volume = report[8];
    if (haptic) applyBluetoothHapticMode(report, settings.BtHapticsModeFlags != 0);
    if (speaker) applyBluetoothSpeaker(report, static_cast<std::uint8_t>(std::clamp(settings.BtSpeakerVolume, 0, 127)));
    refreshBluetoothOutputCrc(report, 78);
    static int logged = 0;
    static std::uint8_t last0 = 0xff, last1 = 0xff, lastPower = 0xff, lastAudio = 0xff, lastVolume = 0xff;
    const bool changed = flag0 != last0 || flag1 != last1 || power != lastPower || audio != lastAudio || volume != lastVolume || report[10] != audio || report[8] != volume;
    if (logged >= 16 && !changed) return;
    if (logged >= 48) return;
    ++logged;
    last0 = flag0;
    last1 = flag1;
    lastPower = power;
    lastAudio = audio;
    lastVolume = volume;
    auto hex48 = [](const std::uint8_t* data) {
        static const char* digits = "0123456789abcdef";
        std::string text(96, '0');
        for (unsigned i = 0; i < 48; ++i) {
            text[i * 2] = digits[data[i] >> 4];
            text[i * 2 + 1] = digits[data[i] & 0x0f];
        }
        return text;
    };
    note("BtHaptics game 0x31 merge haptic=" + std::string(haptic ? "1" : "0") + " speaker=" + (speaker ? "1" : "0")
        + " flag0=" + hexNum(flag0) + "->" + hexNum(report[3]) + " flag1=" + hexNum(flag1) + "->" + hexNum(report[4])
        + " vol=" + hexNum(volume) + "->" + hexNum(report[8]) + " audio=" + hexNum(audio) + "->" + hexNum(report[10])
        + " audio2=" + hexNum(audio2) + "->" + hexNum(report[40])
        + " power=" + hexNum(power) + "->" + hexNum(report[12]) + " reduce=" + hexNum(reduce) + "->" + hexNum(report[39])
        + " hapticByte=" + hexNum(hapticByte) + "->" + hexNum(report[42])
        + " raw=" + hex48(report));
}
bool writeBluetooth(WriteFileFn original, HANDLE handle, const std::uint8_t* src, DWORD srcLen, LPDWORD written, LPOVERLAPPED overlapped, const char* tag) {
    std::lock_guard gate(outputMutex);
    auto report = std::make_unique<std::uint8_t[]>(78);
    if (!buildBluetoothOutput(report.get(), src, srcLen, nextSequence(handle))) return false;
    keepAudioMode(report.get());
    logWriteHex(tag, src, srcLen, report.get());
    if (!overlapped) {
        DWORD sent = 0;
        BOOL ok;
        { Stall stall("HID WriteFile"); ok = original(handle, report.get(), 78, &sent, nullptr); }
        const DWORD error = GetLastError();
        if (ok && written) *written = srcLen;
        Tracked device{};
        copyTracked(handle, device);
        logEvent(handle, device.generation, KWrite, tag, "WriteFile", "WriteFile handle=" + hexPtr(handle) + " id=2->31 size=" + std::to_string(srcLen) + "->78 ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error) + " overlapped=0", ok == FALSE);
        if (!ok) SetLastError(error);
        return ok == TRUE;
    }
    auto pending = std::make_shared<Pending>();
    pending->type = 1;
    pending->device = handle;
    pending->key = overlapped->hEvent ? overlapped->hEvent : handle;
    pending->overlapped = overlapped;
    pending->callerSize = srcLen;
    pending->tag = tag;
    pending->writeBuf = std::move(report);
    rememberPending(pending);
    BOOL ok;
    { Stall stall("HID WriteFile"); ok = original(handle, pending->writeBuf.get(), 78, written, overlapped); }
    const DWORD error = GetLastError();
    pending->accept.store(true, std::memory_order_release);
    if (ok || error != ERROR_IO_PENDING) {
        dropPending(pending->key);
        if (ok && written) *written = srcLen;
        if (ok) overlapped->InternalHigh = srcLen;
        Tracked device{};
        copyTracked(handle, device);
        logEvent(handle, device.generation, KWrite, tag, "WriteFile", "WriteFile handle=" + hexPtr(handle) + " id=2->31 size=" + std::to_string(srcLen) + "->78 ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error) + " overlapped=1", ok == FALSE);
        if (!ok) SetLastError(error);
        return ok == TRUE;
    }
    if (overlapped->Internal != kIoPending) {
        if (auto done = settlePending(pending->key, false)) finishWrite(done, TRUE, 0);
    }
    SetLastError(ERROR_IO_PENDING);
    return FALSE;
}
BOOL WINAPI writeFile(HANDLE handle, LPCVOID buffer, DWORD size, LPDWORD written, LPOVERLAPPED overlapped) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalWrite ? slot->originalWrite : originalOf(&Slot::originalWrite);
    if (!original) return FALSE;
    const auto* bytes = static_cast<const std::uint8_t*>(buffer);
    const char* tag = slot && slot->tag ? slot->tag : "bridge";
    Tracked device{};
    const bool watch = copyTracked(handle, device);
    std::unique_lock<std::recursive_mutex> gate(outputMutex, std::defer_lock);
    if (watch && !gate.try_lock()) { Stall stall("outputMutex"); gate.lock(); }
    if (watch && bytes && size >= 2 && size <= 64 && bytes[0] == 0x02) return writeBluetooth(original, handle, bytes, size, written, overlapped, device.tag ? device.tag : tag) ? TRUE : FALSE;
    const BOOL ok = original(handle, buffer, size, written, overlapped);
    const DWORD error = GetLastError();
    if (watch) logEvent(handle, device.generation, KWrite, device.tag ? device.tag : tag, "WriteFile", "WriteFile handle=" + hexPtr(handle) + " id=" + std::to_string(bytes && size ? bytes[0] : 0) + " size=" + std::to_string(size) + " ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error) + " overlapped=" + (overlapped ? "1" : "0"), ok == FALSE);
    if (!ok) SetLastError(error);
    return ok;
}
BOOLEAN WINAPI getFeature(HANDLE handle, PVOID buffer, ULONG size) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalGetFeature ? slot->originalGetFeature : originalOf(&Slot::originalGetFeature);
    if (!original) return FALSE;
    const BOOLEAN ok = original(handle, buffer, size);
    const DWORD error = GetLastError();
    Tracked device{};
    if (!copyTracked(handle, device)) return ok;
    unsigned long version = 0;
    int raised = 0;
    if (ok && buffer && size > 0x1f && static_cast<unsigned char*>(buffer)[0] == 0x20) {
        auto* bytes = static_cast<unsigned char*>(buffer);
        version = bytes[0x1c] | (static_cast<unsigned long>(bytes[0x1d]) << 8) | (static_cast<unsigned long>(bytes[0x1e]) << 16) | (static_cast<unsigned long>(bytes[0x1f]) << 24);
        if (version < 0x1003e) {
            bytes[0x1c] = 0x3e;
            bytes[0x1d] = 0x00;
            bytes[0x1e] = 0x01;
            bytes[0x1f] = 0x00;
            version = 0x1003e;
            raised = 1;
        }
    }
    std::string line = "GetFeature handle=" + hexPtr(handle) + " id=" + std::to_string(buffer && size ? static_cast<unsigned char*>(buffer)[0] : 0) + " size=" + std::to_string(size) + " ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error);
    if (ok && buffer && size > 0x1f && static_cast<unsigned char*>(buffer)[0] == 0x20) line += " version=" + hexNum(version) + " raised=" + std::to_string(raised);
    logEvent(handle, device.generation, KGet, device.tag ? device.tag : "bridge", "GetFeature", line, ok == FALSE);
    if (!ok) SetLastError(error);
    return ok;
}
BOOLEAN WINAPI setFeature(HANDLE handle, PVOID buffer, ULONG size) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalFeature ? slot->originalFeature : originalOf(&Slot::originalFeature);
    if (!original) return FALSE;
    const auto* bytes = static_cast<const std::uint8_t*>(buffer);
    const char* tag = slot && slot->tag ? slot->tag : "bridge";
    Tracked device{};
    const bool watch = copyTracked(handle, device);
    const char* module = watch && device.tag ? device.tag : tag;
    if (watch && bytes && size >= 1 && bytes[0] == 0x08) {
        logEvent(handle, device.generation, KSet, module, "SetFeature", "SetFeature swallowed id=8 handle=" + hexPtr(handle) + " size=" + std::to_string(size) + " ok=1 err=0");
        SetLastError(0);
        return TRUE;
    }
    if (watch && bytes && size >= 2 && size <= 64 && bytes[0] == 0x02) {
        std::lock_guard gate(outputMutex);
        std::uint8_t report[78];
        if (buildBluetoothOutput(report, bytes, size, nextSequence(handle))) {
            keepAudioMode(report);
            BOOLEAN ok;
            { Stall stall("HID SetFeature"); ok = original(handle, report, sizeof report); }
            const DWORD error = GetLastError();
            logEvent(handle, device.generation, KSet, module, "SetFeature", "SetFeature handle=" + hexPtr(handle) + " id=2->31 size=" + std::to_string(size) + "->78 ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error), ok == FALSE);
            if (!ok) SetLastError(error);
            return ok;
        }
    }
    const BOOLEAN ok = original(handle, buffer, size);
    const DWORD error = GetLastError();
    if (watch) logEvent(handle, device.generation, KSet, module, "SetFeature", "SetFeature handle=" + hexPtr(handle) + " id=" + std::to_string(bytes && size ? bytes[0] : 0) + " size=" + std::to_string(size) + " ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error), ok == FALSE);
    if (!ok) SetLastError(error);
    return ok;
}
BOOLEAN WINAPI getProduct(HANDLE handle, PVOID buffer, ULONG size) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalProduct ? slot->originalProduct : originalOf(&Slot::originalProduct);
    if (!original) return FALSE;
    const BOOLEAN ok = original(handle, buffer, size);
    const DWORD error = GetLastError();
    std::string product;
    if (ok && buffer && size >= sizeof(wchar_t)) {
        const auto* text = static_cast<const wchar_t*>(buffer);
        const std::size_t cap = size / sizeof(wchar_t);
        std::size_t length = 0;
        while (length < cap && text[length]) ++length;
        product = narrow(std::wstring(text, length));
    }
    Tracked device{};
    const bool watch = copyTracked(handle, device);
    const char* tag = watch && device.tag ? device.tag : (slot && slot->tag ? slot->tag : "bridge");
    logEvent(handle, watch ? device.generation : 0, KProduct, tag, "GetProductString", "GetProductString handle=" + hexPtr(handle) + " ok=" + (ok ? "1" : "0") + " err=" + std::to_string(ok ? 0 : error) + " product=" + product);
    if (ok && !product.empty()) {
        static int products = 0;
        if (products < 4) { ++products; note(std::string(tag) + " connection candidate GetProductString product=" + product); }
    }
    if (!ok) SetLastError(error);
    return ok;
}
BOOLEAN WINAPI getPreparsed(HANDLE handle, void** data) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalPreparsed ? slot->originalPreparsed : originalOf(&Slot::originalPreparsed);
    if (!original) return FALSE;
    const BOOLEAN ok = original(handle, data);
    Tracked device{};
    if (ok && data && *data && copyTracked(handle, device)) {
        std::lock_guard lock(handlesMutex);
        preparsedHandles[*data] = handle;
    }
    return ok;
}
BOOLEAN WINAPI freePreparsed(void* data) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalFreePreparsed ? slot->originalFreePreparsed : originalOf(&Slot::originalFreePreparsed);
    {
        std::lock_guard lock(handlesMutex);
        preparsedHandles.erase(data);
    }
    return original ? original(data) : FALSE;
}
long WINAPI getValueCaps(int reportType, unsigned char* caps, unsigned long* count, void* preparsed) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalValueCaps ? slot->originalValueCaps : originalOf(&Slot::originalValueCaps);
    if (!original) return 0xC0000001;
    const long status = original(reportType, caps, count, preparsed);
    if (status != 0x00110000 || !caps || !count) return status;
    HANDLE handle{};
    {
        std::lock_guard lock(handlesMutex);
        const auto found = preparsedHandles.find(preparsed);
        if (found != preparsedHandles.end()) handle = found->second;
    }
    if (!handle) return status;
    Tracked device{};
    if (!copyTracked(handle, device)) return status;
    const char* tag = device.tag ? device.tag : "bridge";
    const unsigned char replacement = reportType == 1 ? 0x02 : 0x01;
    bool matched = false;
    for (unsigned long i = 0; i < *count && i < 64; ++i) {
        unsigned char* cap = caps + static_cast<std::size_t>(i) * 72;
        std::uint16_t page = 0, reportCount = 0, usage = 0;
        std::memcpy(&page, cap, 2);
        std::memcpy(&reportCount, cap + 0x14, 2);
        std::memcpy(&usage, cap + 0x38, 2);
        logEvent(handle, device.generation, KValue, tag, "GetValueCaps", "GetValueCaps handle=" + hexPtr(handle) + " type=" + std::to_string(reportType) + " index=" + std::to_string(i) + " id=" + std::to_string(cap[2]) + " page=" + hexNum(page) + " count=" + hexNum(reportCount) + " usage=" + hexNum(usage));
        if (cap[2] == 0x31) cap[2] = replacement;
        if (page == 0xff00 && cap[2] == 0x20 && reportCount == 0x3f && usage == 0x26) matched = true;
    }
    if (reportType == 2 && *count > 0) {
        auto stamp = [](unsigned char* cap, unsigned char id, std::uint16_t reportCount, std::uint16_t usage) {
            const std::uint16_t page = 0xff00;
            std::memcpy(cap, &page, 2);
            cap[2] = id;
            std::memcpy(cap + 0x14, &reportCount, 2);
            std::memcpy(cap + 0x38, &usage, 2);
        };
        int id5 = -1, id20 = -1;
        for (unsigned long i = 0; i < *count && i < 64; ++i) {
            const unsigned char id = caps[static_cast<std::size_t>(i) * 72 + 2];
            if (id == 5 && id5 < 0) id5 = static_cast<int>(i);
            if (id == 0x20 && id20 < 0) id20 = static_cast<int>(i);
        }
        if (!matched) {
            int pick = id20;
            if (pick < 0) pick = (id5 == 0 && *count > 1) ? 1 : (id5 == 0 ? -1 : 0);
            if (pick >= 0) stamp(caps + static_cast<std::size_t>(pick) * 72, 0x20, 0x3f, 0x26);
        }
        if (id5 >= 0) stamp(caps + static_cast<std::size_t>(id5) * 72, 5, 0x28, 0x33);
        logEvent(handle, device.generation, KValue, tag, "GetValueCaps", "GetValueCaps handle=" + hexPtr(handle) + " stamped=1");
    }
    return status;
}
long WINAPI getCaps(void* preparsed, void* rawCaps) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalCaps ? slot->originalCaps : originalOf(&Slot::originalCaps);
    if (!original) return 0xC0000001;
    const long status = original(preparsed, rawCaps);
    auto* caps = static_cast<HidCaps*>(rawCaps);
    if (status != 0x00110000 || !caps) return status;
    HANDLE handle{};
    {
        std::lock_guard lock(handlesMutex);
        const auto found = preparsedHandles.find(preparsed);
        if (found != preparsedHandles.end()) handle = found->second;
    }
    if (!handle) return status;
    Tracked device{};
    if (!copyTracked(handle, device)) return status;
    const unsigned short inBefore = caps->inputLength, outBefore = caps->outputLength, featureBefore = caps->featureLength;
    if (caps->usagePage == 1 && caps->usage == 5) {
        if (caps->inputLength > 64) caps->inputLength = 64;
        if (caps->outputLength > 48) caps->outputLength = 48;
        if (caps->featureLength > 64) caps->featureLength = 64;
    }
    logEvent(handle, device.generation, KCaps, device.tag ? device.tag : "bridge", "GetCaps",
        "GetCaps handle=" + hexPtr(handle) + " in=" + std::to_string(inBefore) + "->" + std::to_string(caps->inputLength) + " out=" + std::to_string(outBefore) + "->" + std::to_string(caps->outputLength) + " feature=" + std::to_string(featureBefore) + "->" + std::to_string(caps->featureLength));
    {
        static int capsNoted = 0;
        if (capsNoted < 4) {
            ++capsNoted;
            note(std::string(device.tag ? device.tag : "bridge") + " connection candidate GetCaps usagePage=" + std::to_string(caps->usagePage) + " usage=" + std::to_string(caps->usage) + " in=" + std::to_string(inBefore) + " out=" + std::to_string(outBefore) + " feature=" + std::to_string(featureBefore));
        }
    }
    return status;
}
BOOLEAN WINAPI getAttributes(HANDLE handle, HidAttributes* attributes) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalAttributes ? slot->originalAttributes : originalOf(&Slot::originalAttributes);
    if (!original) return FALSE;
    const BOOLEAN ok = original(handle, attributes);
    const DWORD error = GetLastError();
    unsigned short product = attributes ? attributes->product : 0;
    int spoofed = 0;
    if (ok && attributes && slot && slot->spoofEdgePid) {
        Tracked device{};
        if (copyTracked(handle, device) && attributes->vendor == 0x054c && attributes->product == 0x0df2) {
            attributes->product = 0x0ce6;
            spoofed = 1;
        }
    }
    Tracked device{};
    const bool watch = copyTracked(handle, device);
    const char* tag = watch && device.tag ? device.tag : (slot && slot->tag ? slot->tag : "bridge");
    if (ok && attributes) {
        logEvent(handle, watch ? device.generation : 0, KAttr, tag, "GetAttributes", "GetAttributes handle=" + hexPtr(handle) + " vid=" + hexNum(attributes->vendor) + " pid=" + hexNum(product) + (spoofed ? "->0ce6" : "") + " ver=" + hexNum(attributes->version) + " ok=1 err=0");
        if (attributes->vendor == 0x054c || product == 0x054c) {
            static int noted = 0;
            if (noted < 4) { ++noted; note(std::string(tag) + " connection candidate GetAttributes vid=" + hexNum(attributes->vendor) + " pid=" + hexNum(product) + (spoofed ? " spoofed=0ce6" : "")); }
        }
    } else logEvent(handle, watch ? device.generation : 0, KAttr, tag, "GetAttributes", "GetAttributes handle=" + hexPtr(handle) + " ok=0 err=" + std::to_string(error));
    if (!ok) SetLastError(error);
    return ok;
}
DWORD WINAPI waitForSingle(HANDLE object, DWORD milliseconds) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalWait ? slot->originalWait : originalOf(&Slot::originalWait);
    if (!original) return WAIT_FAILED;
    const DWORD result = original(object, milliseconds);
    if (result == WAIT_OBJECT_0) completeIo(object);
    return result;
}
DWORD WINAPI waitForSingleEx(HANDLE object, DWORD milliseconds, BOOL alertable) {
    Slot* slot = slotFrom(_ReturnAddress());
    auto original = slot && slot->originalWaitEx ? slot->originalWaitEx : originalOf(&Slot::originalWaitEx);
    if (!original) return WAIT_FAILED;
    const DWORD result = original(object, milliseconds, alertable);
    if (result == WAIT_OBJECT_0) completeIo(object);
    return result;
}
bool patchImport(HMODULE module, const char* dllName, const char* function, void* detour, void** original) {
    if (!module || !original || !detour) return false;
    auto* base = reinterpret_cast<std::uint8_t*>(module);
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE) return false;
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!dir.VirtualAddress) return false;
    auto* imports = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress);
    for (; imports->Name; ++imports) {
        const char* name = reinterpret_cast<const char*>(base + imports->Name);
        if (_stricmp(name, dllName) != 0) continue;
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
std::uintptr_t imageEnd(HMODULE module) {
    auto* base = reinterpret_cast<std::uint8_t*>(module);
    auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
    auto* nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    return reinterpret_cast<std::uintptr_t>(base) + nt->OptionalHeader.SizeOfImage;
}
}
bool copyNewestBluetoothPath(wchar_t* path, unsigned capacity) {
    if (!path || capacity < 2) return false;
    path[0] = L'\0';
    std::lock_guard lock(handlesMutex);
    unsigned long long newest = 0;
    const Tracked* found = nullptr;
    for (const auto& [handle, device] : bluetoothHandles) {
        (void)handle;
        if (device.openedTick >= newest && !device.path.empty()) { newest = device.openedTick; found = &device; }
    }
    if (!found || found->path.size() + 1 > capacity) return false;
    std::memcpy(path, found->path.c_str(), (found->path.size() + 1) * sizeof(wchar_t));
    return true;
}
bool sendTrackedBluetoothReport(std::uint8_t* report, unsigned size) {
    if (!report || size < 8 || size > 547) return false;
    if (runtime && !runtime->snapshot().BtHapticsSend) return false;
    HANDLE handle{};
    WriteFileFn write{};
    {
        std::lock_guard gate(outputMutex);
        std::lock_guard lock(handlesMutex);
        unsigned long long newest = 0;
        for (const auto& [candidate, device] : bluetoothHandles) {
            if (device.openedTick >= newest) { newest = device.openedTick; handle = candidate; }
        }
        if (!handle) return false;
        auto& tracked = bluetoothHandles[handle];
        write = originalOf(&Slot::originalWrite);
        if (report[0] != 0x32) {
            const std::uint8_t seq = tracked.seq;
            tracked.seq = static_cast<std::uint8_t>((tracked.seq + 1) & 0x0f);
            report[1] = static_cast<std::uint8_t>((seq & 0x0f) << 4);
            refreshBluetoothOutputCrc(report, size);
        }
    }
    if (!write) return false;
    // The mutex is released before the driver call. A stuck write must not hold the game's 0x31 updates.
    Stall stall("haptic HID WriteFile");
    DWORD sent = 0;
    return write(handle, report, size, &sent, nullptr) == TRUE;
}
bool installBridgeForModule(HMODULE module, const char* tag, bool spoofEdgePid) {
    if (!module || !tag) return false;
    const int existing = slotCount.load(std::memory_order_acquire);
    for (int i = 0; i < existing; ++i) if (slots[i].module == module) return true;
    if (existing >= kSlots) {
        note(std::string(tag) + " bridge skipped: no free slot");
        return false;
    }
    Slot& slot = slots[existing];
    slot.module = module;
    slot.tag = tag;
    slot.spoofEdgePid = spoofEdgePid;
    slot.base = reinterpret_cast<std::uintptr_t>(module);
    slot.end = imageEnd(module);
    startStallWatch();
    slot.phase.store(1, std::memory_order_release);
    slotCount.store(existing + 1, std::memory_order_release);
    struct Patch { const char* dll; const char* name; void* detour; void** original; bool required; };
    const Patch patches[] = {
        {"KERNEL32.dll", "CreateFileW", reinterpret_cast<void*>(&createFile), reinterpret_cast<void**>(&slot.originalCreate), true},
        {"KERNEL32.dll", "ReadFile", reinterpret_cast<void*>(&readFile), reinterpret_cast<void**>(&slot.originalRead), true},
        {"KERNEL32.dll", "WriteFile", reinterpret_cast<void*>(&writeFile), reinterpret_cast<void**>(&slot.originalWrite), true},
        {"KERNEL32.dll", "CloseHandle", reinterpret_cast<void*>(&closeHandle), reinterpret_cast<void**>(&slot.originalClose), true},
        {"KERNEL32.dll", "WaitForSingleObject", reinterpret_cast<void*>(&waitForSingle), reinterpret_cast<void**>(&slot.originalWait), true},
        {"KERNEL32.dll", "WaitForSingleObjectEx", reinterpret_cast<void*>(&waitForSingleEx), reinterpret_cast<void**>(&slot.originalWaitEx), false},
        {"HID.DLL", "HidD_GetAttributes", reinterpret_cast<void*>(&getAttributes), reinterpret_cast<void**>(&slot.originalAttributes), true},
        {"HID.DLL", "HidD_GetFeature", reinterpret_cast<void*>(&getFeature), reinterpret_cast<void**>(&slot.originalGetFeature), true},
        {"HID.DLL", "HidD_SetFeature", reinterpret_cast<void*>(&setFeature), reinterpret_cast<void**>(&slot.originalFeature), true},
        {"HID.DLL", "HidD_GetPreparsedData", reinterpret_cast<void*>(&getPreparsed), reinterpret_cast<void**>(&slot.originalPreparsed), true},
        {"HID.DLL", "HidD_FreePreparsedData", reinterpret_cast<void*>(&freePreparsed), reinterpret_cast<void**>(&slot.originalFreePreparsed), true},
        {"HID.DLL", "HidP_GetCaps", reinterpret_cast<void*>(&getCaps), reinterpret_cast<void**>(&slot.originalCaps), true},
        {"HID.DLL", "HidP_GetValueCaps", reinterpret_cast<void*>(&getValueCaps), reinterpret_cast<void**>(&slot.originalValueCaps), true},
        {"HID.DLL", "HidD_GetProductString", reinterpret_cast<void*>(&getProduct), reinterpret_cast<void**>(&slot.originalProduct), true},
    };
    bool ok = true;
    for (const auto& patch : patches) {
        if (patchImport(module, patch.dll, patch.name, patch.detour, patch.original)) continue;
        note(std::string(tag) + " missing import " + patch.dll + "!" + patch.name);
        if (patch.required) ok = false;
    }
    slot.phase.store(2, std::memory_order_release);
    const int raced = slot.createBefore.load(std::memory_order_acquire);
    note(std::string(tag) + " CreateFileW before patch: " + (raced ? "yes" : "no"));
    {
        std::lock_guard lock(bridgedMutex);
        if (!bridgedModules.empty()) bridgedModules += " ";
        bridgedModules += tag;
        note("bridged modules: " + bridgedModules);
    }
    note(std::string(tag) + (ok ? " bridge installed" : " bridge incomplete") + " spoofEdge=" + (spoofEdgePid ? "1" : "0"));
    return ok;
}
}
