#pragma once
namespace rg {
// Marks the current thread as inside a call that can block. A watchdog writes
// the stack and the recent log if the mark stays up for more than 500 ms.
struct Stall {
    explicit Stall(const char* name);
    ~Stall();
    Stall(const Stall&) = delete;
    Stall& operator=(const Stall&) = delete;
};
void startStallWatch();
}
