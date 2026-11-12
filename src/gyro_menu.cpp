#include "rg/gyro_menu.hpp"
#include "rg/remap.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>
namespace rg {
std::span<const GyroOption> gyroOptions(){static constexpr GyroOption options[]={
 {"GyroEnabled",1,2},{"SensitivityX",.05,201},{"SensitivityY",.05,201},{"AimMultiplier",.05,201},{"AltFireMultiplier",.05,201},
 {"GyroSpace",1,4},{"ActivationMode",1,5},{"ActivationButton",1,static_cast<int>(activationButtonOrder.size())},{"DisableWhileRightStick",1,2},{"InvertX",1,2},{"InvertY",1,2},
 {"AutomaticCalibration",1,3},{"Smoothing",5,101},{"Acceleration",1,4},{"FlickStick",1,4},{"FlickSpinDurationMs",10,101}};return options;}
std::span<const GyroOption> nativeGyroOptions(){static const auto options=[](){std::vector<GyroOption> result;for(auto& o:gyroOptions()){if(std::string_view(o.key)=="DisableWhileRightStick")continue;if(std::string_view(o.key)=="ActivationButton"){result.push_back({"GyroButton",1,12});result.push_back({"GyroTouchpad",1,5});result.push_back({"GyroStickSensor",1,5});result.push_back({"GyroGripSensor",1,5});result.push_back({"GyroStick",1,5});}else result.push_back(o);}result.push_back({"LeftSnap",1,1});result.push_back({"LeftSnapForwardDeadZoneDeg",5,71});result.push_back({"MapL4",1,8});result.push_back({"MapR4",1,8});result.push_back({"MapL5",1,8});result.push_back({"MapR5",1,8});return result;}();return options;}
int remapOptionSource(const GyroOption& o){const char* names[]={"MapL4","MapR4","MapL5","MapR5"};for(int i=0;i<4;++i)if(std::string_view(o.key)==names[i])return remapSources[static_cast<size_t>(i)];return 0;}
static bool leftSnapRow(const GyroOption& o){return std::string_view(o.key)=="LeftSnap";}
static std::vector<int> leftSnapChoices(const Settings& s,std::uint32_t available){auto ids=bindingChoices(1,available);std::erase(ids,s.GyroButton);ids.insert(ids.begin(),0);return ids;}
int bindingFamily(int id){
 if(id==0)return 0;if(id==5||id==12||id==13||id==24)return 2;
 if((id>=14&&id<=16)||id==25)return 3;if((id>=17&&id<=19)||id==26)return 4;
 if(id==3||id==4||id==27||id==28)return 5;return 1;
}
std::vector<int> bindingChoices(int family,std::uint32_t available){
 available=pairedButtonCapabilities(available);std::vector<int> order;
 switch(family){case 0:return {0};case 1:order={1,2,3,4,7,10,11,20,21,22,23};break;
 case 2:order=(available&(buttonMask(12)|buttonMask(13)))?std::vector<int>{12,13,5,24}:std::vector<int>{5};break;
 case 3:order={14,15,16,25};break;case 4:order={17,18,19,26};break;case 5:{
  bool left=(available&buttonMask(29))!=0,right=(available&buttonMask(30))!=0;
  if(left)order.push_back(3);if(right)order.push_back(4);if(left&&right){order.push_back(27);order.push_back(28);}return order;
 }default:return {};}
 std::erase_if(order,[&](int id){return !(available&buttonMask(id));});return order;
}
std::vector<int> bindingFamilies(std::uint32_t available){std::vector<int> result;for(int family=1;family<=5;++family)if(!bindingChoices(family,available).empty())result.push_back(family);return result;}
int optionFamily(const GyroOption& o){const char* names[]={"GyroButton","GyroTouchpad","GyroStickSensor","GyroGripSensor","GyroStick"};for(int i=0;i<5;++i)if(std::string_view(o.key)==names[i])return i+1;return 0;}
static int& familyValue(Settings& s,int f){switch(f){case 1:return s.GyroButton;case 2:return s.GyroTouchpad;case 3:return s.GyroStickSensor;case 4:return s.GyroGripSensor;default:return s.GyroStick;}}
static int familyValue(const Settings& s,int f){auto copy=s;return familyValue(copy,f);}
static std::vector<int> familyOptions(int family,std::uint32_t a){auto ids=bindingChoices(family,a);ids.insert(ids.begin(),0);return ids;}
bool nativeOptionVisible(const GyroOption& o,std::uint32_t a,const Settings* settings){if(std::string_view(o.key)=="LeftSnapForwardDeadZoneDeg")return settings&&settings->LeftSnap!=0;if(int source=remapOptionSource(o))return (a&buttonMask(source))!=0;int f=optionFamily(o);return !f||!bindingChoices(f,a).empty();}
std::vector<const GyroOption*> nativeVisibleOptions(std::uint32_t available,const Settings* settings){
 std::vector<const GyroOption*> result;for(auto& option:nativeGyroOptions())if(nativeOptionVisible(option,available,settings))result.push_back(&option);return result;
}
int nativeFocusIndex(std::string_view key,std::uint32_t available,const Settings* settings){
 int visible=0;bool found=key.empty();for(auto& option:nativeGyroOptions()){
  if(key==option.key)found=true;
  if(nativeOptionVisible(option,available,settings)){if(found)return visible;++visible;}
 }
 return key.empty()?0:std::max(0,visible-1);
}
int nativeOptionCount(const Settings& s,const GyroOption& o,std::uint32_t a){if(leftSnapRow(o))return static_cast<int>(leftSnapChoices(s,a).size());if(int source=remapOptionSource(o))return remapLocked(s,source)?1:static_cast<int>(remapTargets.size());int f=optionFamily(o);return f?static_cast<int>(familyOptions(f,a).size()):optionCount(o,a);}
int nativeOptionIndex(const Settings& s,const GyroOption& o,std::uint32_t a){if(leftSnapRow(o)){auto ids=leftSnapChoices(s,a);auto it=std::find(ids.begin(),ids.end(),s.LeftSnap);return it==ids.end()?0:static_cast<int>(it-ids.begin());}if(int source=remapOptionSource(o)){if(remapLocked(s,source))return 0;int value=remapValue(s,source);auto it=std::find(remapTargets.begin(),remapTargets.end(),remapTargetAllowed(value)?value:0);return it==remapTargets.end()?0:static_cast<int>(it-remapTargets.begin());}int f=optionFamily(o);if(!f)return optionIndex(s,o,a);auto ids=familyOptions(f,a);auto it=std::find(ids.begin(),ids.end(),familyValue(s,f));return it==ids.end()?0:static_cast<int>(it-ids.begin());}
void setNativeOptionIndex(Settings& s,const GyroOption& o,int index,std::uint32_t a){index=std::clamp(index,0,nativeOptionCount(s,o,a)-1);if(leftSnapRow(o)){s.LeftSnap=leftSnapChoices(s,a)[static_cast<size_t>(index)];return;}if(int source=remapOptionSource(o)){if(!remapLocked(s,source))remapValue(s,source)=remapTargets[static_cast<size_t>(index)];return;}int f=optionFamily(o);if(f){familyValue(s,f)=familyOptions(f,a)[index];s.ActivationButton=0;if(s.LeftSnap==s.GyroButton)s.LeftSnap=0;return;}setOptionIndex(s,o,index,a);}
std::string nativeOptionValue(const Settings& s,const GyroOption& o,int index,std::string_view lang,ControllerLayout layout,std::uint32_t a){
 index=std::clamp(index,0,nativeOptionCount(s,o,a)-1);if(leftSnapRow(o)){int id=leftSnapChoices(s,a)[static_cast<size_t>(index)];return id?buttonLabel(id,lang,layout,true):localize("off",lang);}if(int source=remapOptionSource(o)){if(remapLocked(s,source))return localize("remap.gyro",lang);int id=remapTargets[static_cast<size_t>(index)];if(!id)return localize("off",lang);if(id==5)return localize("map.touch",lang);if(id==kTouchClick)return localize("map.click",lang);if(id==kMenuButton)return localize("map.menu",lang);if(id==kShareButton)return localize("map.share",lang);return buttonLabel(id,lang,layout,true);}int family=optionFamily(o);
 if(family){int id=familyOptions(family,a)[index];if(!id)return localize("off",lang);if(family==1)return buttonLabel(id,lang,layout,true);
  if(family==2&&!(a&(buttonMask(12)|buttonMask(13))))return localize("on",lang);
  if(id==3||id==12||id==14||id==17)return localize("choice.left",lang);
  if(id==4||id==13||id==15||id==18)return localize("choice.right",lang);
  if(id==24||id==25||id==26||id==28)return localize("choice.both",lang);
  return localize("choice.either",lang);
 }
 return optionValue(o,index,lang,layout,(a&buttonMask(5))!=0,a);
}
static const SettingInfo& field(const GyroOption& option){for(const auto& f:settingsSchema())if(std::string_view(f.name)==option.key)return f;std::terminate();}
static std::vector<int> buttonChoices(std::uint32_t available){std::vector<int> ids;for(int id:activationButtonOrder)if(id==0||(available&buttonMask(id)))ids.push_back(id);return ids;}
int optionCount(const GyroOption& option,std::uint32_t available){return std::string_view(option.key)=="ActivationButton"?static_cast<int>(buttonChoices(available).size()):option.count;}
int optionIndex(const Settings& settings,const GyroOption& option,std::uint32_t available){
 int value=static_cast<int>(std::lround(field(option).get(settings)/option.step));
 if(std::string_view(option.key)=="AutomaticCalibration")value=value==1?2:value==2?1:0;
 if(std::string_view(option.key)=="ActivationMode")value=value==5?4:value==4?0:value;
 if(std::string_view(option.key)=="ActivationButton"){auto ids=buttonChoices(available);auto it=std::find(ids.begin(),ids.end(),value);return it==ids.end()?0:static_cast<int>(it-ids.begin());}
 return std::clamp(value,0,option.count-1);
}
void setOptionIndex(Settings& settings,const GyroOption& option,int index,std::uint32_t available){
 index=std::clamp(index,0,optionCount(option,available)-1);double value=index*option.step;
 if(std::string_view(option.key)=="AutomaticCalibration"){
  value=index==1?2:index==2?1:0;
 }
 if(std::string_view(option.key)=="ActivationMode"&&index==4)value=5;
 if(std::string_view(option.key)=="ActivationButton")value=buttonChoices(available)[index];
 field(option).set(settings,value);settings.LinkXY=false;settings.RatchetButton=0;
}
void resetGyroOptions(Settings& settings){Settings defaults;for(const auto& option:gyroOptions())field(option).set(settings,field(option).get(defaults));settings.GyroButton=settings.GyroTouchpad=settings.GyroStickSensor=settings.GyroGripSensor=settings.GyroStick=0;settings.MapL4=settings.MapR4=settings.MapL5=settings.MapR5=0;settings.LeftSnap=0;settings.LeftSnapForwardDeadZoneDeg=45;settings.ActivationButton=0;settings.LinkXY=false;settings.RatchetButton=0;}
std::string buttonLabel(int id,std::string_view language,ControllerLayout layout,bool touchpad){
 id=std::clamp(id,0,28);
 if(id>=20&&id<=23){const char* keys[]={"L4","R4","L5","R5"};return keys[id-20];}
 if(id>=24){const char* keys[]={"contact.pad.both","contact.stick.both","contact.grip.both","click.stick.either","click.stick.both"};return localize(keys[id-24],language);}
 if(id>=12){const char* keys[]={"contact.pad.left","contact.pad.right","contact.stick.left","contact.stick.right","contact.stick.either","contact.grip.left","contact.grip.right","contact.grip.either"};return localize(keys[id-12],language);}
 if(id==0)return localize("none",language);
 if(id==5&&layout==ControllerLayout::Steam)return localize("contact.pad.either",language);
 if(id==5)return localize(touchpad?"touchpad":"touchpad.unavailable",language);
 const char* sony[]={"none","L1","R1","L3","R3","touchpad","cross","circle","L2","R2","square","triangle"};
 const char* xbox[]={"none","LB","RB","LS","RS","touchpad","A","B","LT","RT","X","Y"};
 const char* nintendo[]={"none","L","R","button.left_stick","button.right_stick","touchpad","B","A","ZL","ZR","Y","X"};
 const char* generic[]={"none","button.left_shoulder","button.right_shoulder","button.left_stick","button.right_stick","touchpad","button.south","button.east","button.left_trigger","button.right_trigger","button.west","button.north"};
 const char* steam[]={"none","L1","R1","L3","R3","touchpad","A","B","L2","R2","X","Y"};
 const auto* keys=layout==ControllerLayout::Steam?steam:layout==ControllerLayout::Sony?sony:layout==ControllerLayout::Xbox?xbox:layout==ControllerLayout::Nintendo?nintendo:generic;
 return localize(keys[id],language);
}
std::string optionValue(const GyroOption& option,int index,std::string_view language,ControllerLayout layout,bool touchpad,std::uint32_t available){index=std::clamp(index,0,optionCount(option,available)-1);std::string_view key=option.key;
 if(key=="FlickSpinDurationMs")return std::to_string(static_cast<int>(index*option.step))+" ms";
 if(key=="LeftSnapForwardDeadZoneDeg"){int degrees=static_cast<int>(index*option.step);return degrees?std::to_string(degrees)+"°":localize("off",language);}
 if(key=="AutomaticCalibration"){const char* keys[]={"off","calibration.menus","calibration.anytime"};return localize(keys[index],language);}
 if(key=="FlickStick"){const char* keys[]={"stick.game","stick.hip","stick.flick","dpad"};return localize(keys[index],language);}
 if(option.step<1){std::ostringstream out;out<<std::fixed<<std::setprecision(2)<<index*option.step;return out.str();}
 if(key=="GyroSpace"){const char* keys[]={"space.player","space.yaw","space.roll","space.world"};return localize(keys[index],language);}
 if(key=="ActivationMode"){const char* keys[]={"mode.always","mode.aim","mode.hip","mode.hold","mode.toggle"};return localize(keys[index],language);}
 if(key=="ActivationButton")return buttonLabel(buttonChoices(available)[index],language,layout,touchpad);
 if(key=="Smoothing")return index?std::to_string(static_cast<int>(index*option.step))+" ms":localize("off",language);
 if(key=="Acceleration"){const char* keys[]={"off","low","medium","high"};return localize(keys[index],language);}
 return localize(index?"on":"off",language);
}
}
