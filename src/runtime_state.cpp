#include "rg/runtime.hpp"
#include <deque>
#include <fstream>
namespace rg {
namespace {
std::deque<std::string> recentLog;
}
Runtime* runtime{};
Settings Runtime::snapshot(){std::lock_guard lock(settingsMutex);return settings;}
void Runtime::update(const Settings& value){
    // Apply the same validation to UI and file changes.
    auto valid=parseConfig(serializeConfig(value));
    {std::lock_guard lock(settingsMutex);settings=valid.settings;}
    mixedFlags.store((valid.settings.MixedInput?1u:0u)|(valid.settings.IgnoreMouseMotion?2u:0u)|(static_cast<unsigned>(valid.settings.HudMode)<<2));
    revision.fetch_add(1,std::memory_order_release);
}
void Runtime::log(const std::string& message){
    std::lock_guard lock(logMutex);
    recentLog.push_back(std::to_string(monotonicNs()/1'000'000)+" "+message);
    if (recentLog.size()>200) recentLog.pop_front();
    logFile<<recentLog.back()<<'\n'<<std::flush;
}
void Runtime::flushCrash(const std::string& message){
    if (directory.empty()) return;
    std::ofstream out(directory/"ReturnalGyro-crash.log", std::ios::app);
    out<<message<<'\n';
    std::unique_lock lock(logMutex, std::try_to_lock);
    if (lock.owns_lock()) for (const auto& line : recentLog) out<<line<<'\n';
    out.flush();
    if (lock.owns_lock()) { logFile<<message<<'\n'; logFile.flush(); }
}
}
