// Report interpretation is based on SDL's zlib-licensed PS4/PS5 HIDAPI drivers.
// USB is read through a shared HID handle. Bluetooth is read with Raw Input, the
// same passive path daKun uses, so this process never sends an output report.
#include "rg/backend.hpp"
#include "rg/dualsense_bt.hpp"
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <SDL3/SDL_hidapi.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <vector>
namespace rg {
std::uint64_t monotonicNs(){return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());}
namespace {
int signed16(const unsigned char* b){return static_cast<std::int16_t>(static_cast<unsigned>(b[0])|(static_cast<unsigned>(b[1])<<8));}
std::uint32_t unsigned32(const unsigned char* b){return b[0]|(std::uint32_t(b[1])<<8)|(std::uint32_t(b[2])<<16)|(std::uint32_t(b[3])<<24);}
std::uint32_t crc32(std::uint32_t crc,const unsigned char* data,size_t size){for(size_t i=0;i<size;++i){crc^=data[i];for(int n=0;n<8;++n)crc=(crc>>1)^(0xedb88320u&static_cast<unsigned>(-static_cast<int>(crc&1)));}return crc;}
bool crcValid(const unsigned char* data,size_t size){unsigned char header=0xa1;auto crc=crc32(0xffffffff,&header,1);crc=crc32(crc,data,size-4)^0xffffffff;return crc==unsigned32(data+size-4);}
class SonyBackend;
SonyBackend* rawListener{};
class SonyBackend final:public MotionBackend {
    SDL_hid_device* device_{};
    DeviceInfo info_;
    std::string error_;
    bool ps5_{},primed_{},rawMode_{};
    std::uint32_t previousTick_{};
    std::uint64_t ticks_{};
    std::array<float,6> bias_{},scale_{};
    std::uint64_t lastReportNs_{};
    HWND rawWindow_{};
    std::vector<RAWINPUTDEVICE> rawRegistered_;
    std::vector<std::pair<HWND,WNDPROC>> rawWatched_;
    std::vector<std::pair<int,std::array<unsigned char,96>>> rawQueue_;
    void closeHid(){if(device_)SDL_hid_close(device_);device_=nullptr;}
    void stopRaw(){
        for(auto& watched:rawWatched_)if(IsWindow(watched.first))SetWindowLongPtrW(watched.first,GWLP_WNDPROC,reinterpret_cast<LONG_PTR>(watched.second));
        rawWatched_.clear();if(rawListener==this)rawListener=nullptr;
        if(!rawRegistered_.empty()){for(auto& device:rawRegistered_){device.dwFlags=RIDEV_REMOVE;device.hwndTarget=nullptr;}RegisterRawInputDevices(rawRegistered_.data(),static_cast<UINT>(rawRegistered_.size()),sizeof(RAWINPUTDEVICE));rawRegistered_.clear();}
        if(rawWindow_){DestroyWindow(rawWindow_);rawWindow_=nullptr;}
        rawQueue_.clear();rawMode_=false;
    }
    void close(){closeHid();stopRaw();}
    void factoryCalibration() {
        bias_.fill(0);scale_={1.0f/16,1.0f/16,1.0f/16,1.0f/8192,1.0f/8192,1.0f/8192};
        if(!ps5_)scale_[3]=scale_[4]=scale_[5]=1.0f/8192; // Sony accelerometers: +/-4g, 8192 units/g
        std::array<unsigned char,64> data{};data[0]=ps5_?0x05:(info_.bluetooth?0x05:0x02);
        int n=SDL_hid_get_feature_report(device_,data.data(),ps5_?41:(info_.bluetooth?41:37));
        if(n<35){error_="Factory calibration unavailable; nominal scale and manual bias calibration used";return;}
        std::array<float,6> bias{},scale{};
        float speed=static_cast<float>(signed16(data.data()+19)+signed16(data.data()+21));
        if(speed<=0)return;
        for(int i=0;i<3;++i) {
            bias[i]=static_cast<float>(signed16(data.data()+1+i*2));
            int plus=(!ps5_&&info_.bluetooth)?7+i*2:7+i*4;
            int minus=(!ps5_&&info_.bluetooth)?13+i*2:9+i*4;
            float range=ps5_?static_cast<float>(signed16(data.data()+plus)-signed16(data.data()+minus)):
                std::abs(signed16(data.data()+plus)-bias[i])+std::abs(signed16(data.data()+minus)-bias[i]);
            if(range<=0)return;
            scale[i]=speed/range;
            int hi=signed16(data.data()+23+i*4),lo=signed16(data.data()+25+i*4);
            if(hi<=lo)return;
            bias[i+3]=static_cast<float>(hi+lo)*0.5f;scale[i+3]=2.0f/static_cast<float>(hi-lo);
        }
        for(int i=0;i<6;++i){float nominal=i<3?1.0f/16:1.0f/8192;if(std::abs(bias[i])>1024||std::abs(scale[i]/nominal-1)>0.5f)return;}
        bias_=bias;scale_=scale;info_.factoryCalibration=true;
    }
    static LRESULT CALLBACK rawProc(HWND window,UINT message,WPARAM wparam,LPARAM lparam){
        auto* self=reinterpret_cast<SonyBackend*>(GetWindowLongPtrW(window,GWLP_USERDATA));
        if(message==WM_NCCREATE){auto* created=reinterpret_cast<CREATESTRUCTW*>(lparam);SetWindowLongPtrW(window,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(created->lpCreateParams));return DefWindowProcW(window,message,wparam,lparam);}
        if(message==WM_INPUT&&self)self->onRaw(reinterpret_cast<HRAWINPUT>(lparam));
        return DefWindowProcW(window,message,wparam,lparam);
    }
    static LRESULT CALLBACK rawSink(HWND window,UINT message,WPARAM wparam,LPARAM lparam){
        auto* self=rawListener;
        WNDPROC previous{};if(self)for(auto& watched:self->rawWatched_)if(watched.first==window)previous=watched.second;
        if(message==WM_INPUT&&self)self->onRaw(reinterpret_cast<HRAWINPUT>(lparam));
        if(previous)return CallWindowProcW(previous,window,message,wparam,lparam);
        return DefWindowProcW(window,message,wparam,lparam);
    }
    void onRaw(HRAWINPUT input){
        UINT size=0;if(GetRawInputData(input,RID_INPUT,nullptr,&size,sizeof(RAWINPUTHEADER))!=0||size<sizeof(RAWINPUTHEADER)||size>4096)return;
        std::vector<unsigned char> packet(size);
        if(GetRawInputData(input,RID_INPUT,packet.data(),&size,sizeof(RAWINPUTHEADER))==static_cast<UINT>(-1))return;
        auto* raw=reinterpret_cast<RAWINPUT*>(packet.data());
        if(raw->header.dwType!=RIM_TYPEHID||raw->data.hid.dwSizeHid==0)return;
        RID_DEVICE_INFO ident{};ident.cbSize=sizeof(ident);UINT identSize=sizeof(ident);
        if(GetRawInputDeviceInfoW(raw->header.hDevice,RIDI_DEVICEINFO,&ident,&identSize)==static_cast<UINT>(-1))return;
        if(ident.dwType!=RIM_TYPEHID||ident.hid.dwVendorId!=0x054c||ident.hid.dwProductId!=info_.product)return;
        const auto* bytes=raw->data.hid.bRawData;DWORD one=raw->data.hid.dwSizeHid;DWORD count=raw->data.hid.dwCount?raw->data.hid.dwCount:1;
        if(static_cast<std::size_t>(one)*count+sizeof(RAWINPUTHEADER)>packet.size())return;
        for(DWORD i=0;i<count;++i){int n=static_cast<int>(one);if(n<=0||n>96)continue;std::array<unsigned char,96> copy{};std::memcpy(copy.data(),bytes+static_cast<std::size_t>(i)*one,static_cast<std::size_t>(n));rawQueue_.push_back({n,copy});}
        if(rawQueue_.size()>32)rawQueue_.erase(rawQueue_.begin(),rawQueue_.end()-32);
    }
    bool startRaw(unsigned product){
        stopRaw();info_.product=product;
        HMODULE module{};GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,reinterpret_cast<LPCWSTR>(&rawProc),&module);
        WNDCLASSW windowClass{};windowClass.lpfnWndProc=rawProc;windowClass.hInstance=module;windowClass.lpszClassName=L"ReturnalGyroRaw";
        RegisterClassW(&windowClass);
        rawWindow_=CreateWindowExW(WS_EX_NOACTIVATE|WS_EX_TOOLWINDOW,windowClass.lpszClassName,L"",WS_POPUP,0,0,0,0,nullptr,nullptr,module,this);
        if(!rawWindow_){error_="Cannot create the Bluetooth motion window";return false;}
        UINT count=0;GetRawInputDeviceList(nullptr,&count,sizeof(RAWINPUTDEVICELIST));
        std::vector<RAWINPUTDEVICELIST> list(count);
        if(count){UINT got=GetRawInputDeviceList(list.data(),&count,sizeof(RAWINPUTDEVICELIST));if(got==static_cast<UINT>(-1))list.clear();else list.resize(got);}
        std::vector<std::uint32_t> usages;
        for(const auto& item:list){if(item.dwType!=RIM_TYPEHID)continue;RID_DEVICE_INFO ident{};ident.cbSize=sizeof(ident);UINT identSize=sizeof(ident);if(GetRawInputDeviceInfoW(item.hDevice,RIDI_DEVICEINFO,&ident,&identSize)==static_cast<UINT>(-1))continue;if(ident.hid.dwVendorId!=0x054c||ident.hid.dwProductId!=product)continue;std::uint32_t key=(std::uint32_t(ident.hid.usUsagePage)<<16)|ident.hid.usUsage;if(std::find(usages.begin(),usages.end(),key)==usages.end())usages.push_back(key);}
        if(usages.empty()){error_="Bluetooth controller is not visible to Raw Input";DestroyWindow(rawWindow_);rawWindow_=nullptr;return false;}
        UINT registeredCount=0;GetRegisteredRawInputDevices(nullptr,&registeredCount,sizeof(RAWINPUTDEVICE));
        std::vector<RAWINPUTDEVICE> existing(registeredCount);
        if(registeredCount){UINT got=GetRegisteredRawInputDevices(existing.data(),&registeredCount,sizeof(RAWINPUTDEVICE));if(got==static_cast<UINT>(-1))existing.clear();else existing.resize(got);}
        std::vector<RAWINPUTDEVICE> add;
        for(std::uint32_t key:usages){
            USHORT page=static_cast<USHORT>(key>>16),usage=static_cast<USHORT>(key&0xffff);
            const RAWINPUTDEVICE* found{};
            for(const auto& registration:existing)if(registration.usUsagePage==page&&registration.usUsage==usage&&(registration.dwFlags&RIDEV_REMOVE)==0){found=&registration;break;}
            if(found&&found->hwndTarget&&found->hwndTarget!=rawWindow_){
                bool seen=false;for(auto& watched:rawWatched_)if(watched.first==found->hwndTarget)seen=true;
                if(!seen){auto previous=reinterpret_cast<WNDPROC>(SetWindowLongPtrW(found->hwndTarget,GWLP_WNDPROC,reinterpret_cast<LONG_PTR>(rawSink)));if(previous&&previous!=rawSink)rawWatched_.push_back({found->hwndTarget,previous});}
                continue;
            }
            if(found&&found->hwndTarget==rawWindow_)continue;
            RAWINPUTDEVICE device{};device.usUsagePage=page;device.usUsage=usage;device.dwFlags=RIDEV_INPUTSINK|RIDEV_DEVNOTIFY;device.hwndTarget=rawWindow_;add.push_back(device);
        }
        if(!add.empty()&&!RegisterRawInputDevices(add.data(),static_cast<UINT>(add.size()),sizeof(RAWINPUTDEVICE))){error_="Cannot register Bluetooth motion reports";stopRaw();return false;}
        rawListener=this;rawRegistered_=std::move(add);rawMode_=true;return true;
    }
    void pumpRaw(){
        if(!rawWindow_)return;
        MsgWaitForMultipleObjects(0,nullptr,FALSE,5,QS_ALLINPUT);
        MSG message;while(PeekMessageW(&message,nullptr,0,0,PM_REMOVE)){TranslateMessage(&message);DispatchMessageW(&message);}
    }
    std::optional<GyroSample> decode(const unsigned char* bytes,int n){
        std::array<unsigned char,97> wrapped{};const unsigned char* data=bytes;int size=n;
        if(bytes[0]!=0x01&&bytes[0]!=0x31&&n>=48&&n<=96){wrapped[0]=0x01;std::memcpy(wrapped.data()+1,bytes,static_cast<std::size_t>(n));data=wrapped.data();size=n+1;}
        size_t offset=0;
        if(ps5_){if(data[0]==1&&size>=64)offset=1;else if(data[0]==0x31&&size>=78&&crcValid(data,78))offset=2;else return {};}
        else {if(data[0]==1&&size>=64)offset=1;else if(data[0]==0x11&&size>=78&&crcValid(data,78))offset=3;else return {};}
        lastReportNs_=monotonicNs();
        const auto* p=data+offset;std::uint32_t tick=ps5_?unsigned32(p+27):static_cast<std::uint16_t>(signed16(p+9));
        if(!primed_){previousTick_=tick;primed_=true;return {};}
        std::uint32_t delta=ps5_?tick-previousTick_:static_cast<std::uint16_t>(tick-previousTick_);
        if(delta==0)return {};
        previousTick_=tick;
        const double periodNs=ps5_?1000.0/3:16000.0/3;
        if(delta*periodNs>100'000'000){ticks_+=delta;return {};}
        ticks_+=delta;GyroSample s;s.sensorNs=static_cast<std::uint64_t>(ticks_*periodNs);s.arrivalNs=lastReportNs_;
        const size_t gyroOffset=ps5_?15:12,accelOffset=ps5_?21:18;
        float g[3],a[3];for(size_t i=0;i<3;++i){g[i]=(signed16(p+gyroOffset+i*2)-bias_[i])*scale_[i];a[i]=(signed16(p+accelOffset+i*2)-bias_[i+3])*scale_[i+3];}
        s.degreesPerSecond={g[0],g[1],g[2]};s.accelG={a[0],a[1],a[2]};
        s.buttons=sonyButtons({p,static_cast<size_t>(size)-offset},ps5_,info_.product==0x0df2);
        if(info_.bluetooth&&info_.product==0x0df2){
            const auto paddle=latestEdgePaddleByte();
            if(paddle&0x40)s.buttons|=buttonMask(20);
            if(paddle&0x80)s.buttons|=buttonMask(21);
            if(paddle&0x10)s.buttons|=buttonMask(22);
            if(paddle&0x20)s.buttons|=buttonMask(23);
        }
        s.leftStickMagnitude=std::min(1.0f,std::hypot((p[0]-127.5f)/127.5f,(p[1]-127.5f)/127.5f));
        s.rightStickMagnitude=std::min(1.0f,std::hypot((p[2]-127.5f)/127.5f,(p[3]-127.5f)/127.5f));
        return s;
    }
public:
    SonyBackend(){SDL_hid_init();}
    ~SonyBackend() override {close();SDL_hid_exit();}
    bool connect(int index) override {
        close();error_.clear();info_={};primed_=false;ticks_=0;
        struct Candidate{std::string path;unsigned product;bool bluetooth;std::string name;};
        std::vector<Candidate> candidates;
        auto list=SDL_hid_enumerate(0x054c,0);
        for(auto d=list;d;d=d->next) {
            if(d->usage_page!=1||d->usage!=5)continue;
            if(d->product_id!=0x05c4&&d->product_id!=0x09cc&&d->product_id!=0x0ce6&&d->product_id!=0x0df2)continue;
            std::string name=d->product_id==0x0df2?"DualSense Edge":d->product_id==0x0ce6?"DualSense":"DualShock 4";
            candidates.push_back({d->path,d->product_id,d->bus_type==SDL_HID_API_BUS_BLUETOOTH,name});
        }
        SDL_hid_free_enumeration(list);
        std::sort(candidates.begin(),candidates.end(),[](auto& a,auto& b){return a.path<b.path;});
        if(index<0||static_cast<size_t>(index)>=candidates.size()){error_="No selected Sony motion controller connected";return false;}
        auto c=candidates[static_cast<size_t>(index)];
        info_={c.name,c.path,0x054c,c.product,c.bluetooth,false};ps5_=c.product==0x0ce6||c.product==0x0df2;
        info_.diagram=ps5_?ControllerDiagram::DualSense:ControllerDiagram::DualShock;info_.layout=ControllerLayout::Sony;info_.touchpad=true;info_.availableButtons=analogStickCapabilities|standardGyroButtons|buttonMask(5)|(c.product==0x0df2?rearGyroButtons:0);
        if(c.bluetooth){
            // The game owns the HID handle through scePad. Opening it here would contend with triggers and haptics.
            bias_.fill(0);scale_={1.0f/16,1.0f/16,1.0f/16,1.0f/8192,1.0f/8192,1.0f/8192};
            if(!startRaw(c.product))return false;
            info_.name+=" Bluetooth";
        }else{
            device_=SDL_hid_open_path(c.path.c_str());
            if(!device_){error_="Cannot open Sony controller with shared HID access";return false;}
            factoryCalibration();
        }
        lastReportNs_=monotonicNs();return true;
    }
    std::optional<GyroSample> read() override {
        if(!device_&&!rawMode_)return {};
        if(monotonicNs()-lastReportNs_>2'000'000'000){error_=rawMode_?"No Bluetooth motion reports":"No valid extended Sony motion reports; reconnect or use USB";close();return {};}
        if(rawMode_){
            pumpRaw();
            while(!rawQueue_.empty()){auto next=rawQueue_.front();rawQueue_.erase(rawQueue_.begin());if(auto sample=decode(next.second.data(),next.first))return sample;}
            return {};
        }
        std::array<unsigned char,128> bytes{};
        int n=SDL_hid_read_timeout(device_,bytes.data(),bytes.size(),5);
        if(n<0){error_="Sony controller disconnected";close();return {};}
        if(n==0)return {};
        return decode(bytes.data(),n);
    }
    bool connected()const override{return device_!=nullptr||rawMode_;}
    DeviceInfo info()const override{return info_;}
    std::string error()const override{return error_;}
};
}
std::unique_ptr<MotionBackend> makeSonyPassiveBackend(){return std::make_unique<SonyBackend>();}
}
