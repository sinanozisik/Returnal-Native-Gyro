#pragma once
#include "controller_buttons.hpp"
#include "settings.hpp"
#include <algorithm>
#include <array>
#include <vector>
namespace rg {
// Rear Edge buttons map to a button the game already understands. A rear button
// selected as the gyro button keeps its saved target, but the target stays inactive.
inline constexpr std::array<int,4> remapSources{20,21,22,23};
// 5 touch contact, 40 touchpad click, 41 menu/options, 42 share/create.
inline constexpr int kTouchClick=40,kMenuButton=41,kShareButton=42;
inline constexpr std::array<int,13> remapTargets{0,1,2,3,4,5,6,7,10,11,kTouchClick,kMenuButton,kShareButton};
inline bool remapTargetAllowed(int id){for(int target:remapTargets)if(target==id)return true;return false;}
inline int& remapValue(Settings& s,int source){switch(source){case 20:return s.MapL4;case 21:return s.MapR4;case 22:return s.MapL5;default:return s.MapR5;}}
inline int remapValue(const Settings& s,int source){Settings copy=s;return remapValue(copy,source);}
inline bool remapLocked(const Settings& s,int source){return s.GyroButton==source||s.LeftSnap==source;}
inline int remapOutput(const Settings& s,int source){
    if(source<20||source>23||remapLocked(s,source))return 0;
    int value=remapValue(s,source);return remapTargetAllowed(value)?value:0;
}
// Physical rear buttons also set their mapped target bit, before gyro, snap, and activation read the mask.
inline std::uint32_t withRemappedButtons(std::uint32_t buttons,const Settings& s){
    auto mapped=buttons;
    for(int source:remapSources){
        int target=remapOutput(s,source);
        if(target>0&&gyroButtonHeld(buttons,source))mapped|=buttonMask(target);
    }
    return mapped;
}
// Active targets. A physical button that is already down is left to the game.
inline std::vector<int> remapDesired(const Settings& s,std::uint32_t buttons,bool fresh){
    std::vector<int> want;
    if(!fresh)return want;
    for(int source:remapSources){
        int target=remapOutput(s,source);
        if(target<=0||!gyroButtonHeld(buttons,source))continue;
        if(target<12&&gyroButtonHeld(buttons,target))continue;
        if(std::find(want.begin(),want.end(),target)==want.end())want.push_back(target);
    }
    return want;
}
}
