#include "rg/stall_watch.hpp"
#include "rg/runtime.hpp"
#include <windows.h>
#include <atomic>
#include <cstdio>
#include <string>
#include <thread>
namespace rg {
namespace {
struct Frame { const char* name; unsigned long long since; };
struct Local { DWORD id{}; int depth{}; int slot{-1}; Frame stack[6]{}; };
thread_local Local local{};
struct Published {
    std::atomic<DWORD> thread{0};
    std::atomic<unsigned long long> since{0};
    std::atomic<const char*> name{nullptr};
    std::atomic<unsigned long long> reported{0};
};
Published published[48]{};
long long ticks() { LARGE_INTEGER n{}; QueryPerformanceCounter(&n); return n.QuadPart; }
long long tickHz() { static long long hz = [] { LARGE_INTEGER n{}; QueryPerformanceFrequency(&n); return n.QuadPart; }(); return hz; }
int claimSlot() {
    static std::atomic<int> next{0};
    int slot = next.fetch_add(1);
    if (slot >= 48) return -1;
    published[slot].thread.store(GetCurrentThreadId());
    return slot;
}
void publish() {
    if (local.slot < 0) return;
    auto& slot = published[local.slot];
    if (local.depth <= 0) { slot.since.store(0); return; }
    const auto& top = local.stack[local.depth - 1];
    slot.name.store(top.name);
    slot.since.store(top.since);
}
using LookupFn = PRUNTIME_FUNCTION(WINAPI*)(DWORD64, PDWORD64, PUNWIND_HISTORY_TABLE);
using UnwindFn = PEXCEPTION_ROUTINE(WINAPI*)(DWORD, DWORD64, DWORD64, PRUNTIME_FUNCTION, PCONTEXT, PVOID*, PDWORD64, PKNONVOLATILE_CONTEXT_POINTERS);
LookupFn lookupFn{};
UnwindFn unwindFn{};
void loadUnwind() {
    if (lookupFn) return;
    auto* ntdll = GetModuleHandleW(L"ntdll.dll");
    if (!ntdll) return;
    lookupFn = reinterpret_cast<LookupFn>(GetProcAddress(ntdll, "RtlLookupFunctionEntry"));
    unwindFn = reinterpret_cast<UnwindFn>(GetProcAddress(ntdll, "RtlVirtualUnwind"));
}
std::string frameText(void* address) {
    HMODULE module = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, static_cast<LPCWSTR>(address), &module);
    wchar_t path[260]{};
    const wchar_t* file = L"?";
    unsigned long long rva = reinterpret_cast<unsigned long long>(address);
    if (module && GetModuleFileNameW(module, path, 260)) {
        const wchar_t* slash = wcsrchr(path, L'\\');
        file = slash ? slash + 1 : path;
        rva -= reinterpret_cast<unsigned long long>(module);
    }
    char name[120]{};
    WideCharToMultiByte(CP_UTF8, 0, file, -1, name, 100, nullptr, nullptr);
    char line[180]{};
    sprintf_s(line, "  %s+0x%llX", name, rva);
    return line;
}
std::string stackOf(DWORD threadId) {
    loadUnwind();
    if (!lookupFn || !unwindFn) return "  stack unavailable";
    HANDLE thread = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE, threadId);
    if (!thread) return "  thread unavailable";
    void* frames[24]{};
    int count = 0;
    if (SuspendThread(thread) != static_cast<DWORD>(-1)) {
        CONTEXT context{};
        context.ContextFlags = CONTEXT_FULL;
        if (GetThreadContext(thread, &context)) {
            for (; count < 24 && context.Rip; ++count) {
                frames[count] = reinterpret_cast<void*>(context.Rip);
                DWORD64 image = 0;
                auto* entry = lookupFn(context.Rip, &image, nullptr);
                if (!entry) break;
                PVOID data = nullptr;
                DWORD64 frame = 0;
                unwindFn(0, image, context.Rip, entry, &context, &data, &frame, nullptr);
            }
        }
        ResumeThread(thread);
    }
    CloseHandle(thread);
    std::string text;
    for (int i = 0; i < count; ++i) text += frameText(frames[i]) + "\n";
    if (text.empty()) text = "  empty stack\n";
    return text;
}
void watch() {
    for (;;) {
        Sleep(100);
        const long long now = ticks();
        const long long limit = tickHz() / 2;
        for (auto& slot : published) {
            const auto since = slot.since.load();
            const auto thread = slot.thread.load();
            if (!since || !thread || now - static_cast<long long>(since) < limit) continue;
            if (slot.reported.load() == since) continue;
            slot.reported.store(since);
            const char* name = slot.name.load();
            const auto stuckMs = (now - static_cast<long long>(since)) * 1000 / tickHz();
            std::string message = "stall " + std::string(name ? name : "?") + " tid=" + std::to_string(thread) + " ms=" + std::to_string(stuckMs) + "\n" + stackOf(thread);
            if (runtime) runtime->flushCrash(message);
        }
    }
}
}
Stall::Stall(const char* name) {
    if (!local.id) local.id = GetCurrentThreadId();
    if (local.slot < 0) local.slot = claimSlot();
    if (local.depth < 6) local.stack[local.depth] = {name, static_cast<unsigned long long>(ticks())};
    ++local.depth;
    publish();
}
Stall::~Stall() {
    if (local.depth > 0) --local.depth;
    publish();
}
void startStallWatch() {
    static std::atomic<bool> started{false};
    if (started.exchange(true)) return;
    std::thread(watch).detach();
}
}
