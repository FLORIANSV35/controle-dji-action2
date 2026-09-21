// =============================================================
// DJI Action 2 — Standalone BLE control
// V11 — V10 + Backpack OSD + ARM detected via MSP FC (UART0)
// Required library: NimBLE-Arduino (h2zero) 2.x branch
// FC Betaflight : RX=GPIO20  TX=GPIO21  115200 baud
// =============================================================

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include "freertos/semphr.h"
#include "mbedtls/md5.h"
#include <NimBLEDevice.h>

// =========================
// BIND PHRASE ELRS BACKPACK
// To change the bind phrase: edit this line then reflash the ESP32.
// The UID is recomputed automatically at boot (see computeBindUID()).
// =========================
// REPLACE with your own ELRS bind phrase (the same as on your goggles/backpack)
#define BIND_PHRASE "MY_BIND_PHRASE"

// =========================
// MSP FC (remapped UART0)
// =========================
#define MSP_STATUS  101
#define MSP_BAUD    115200
#define MSP_RX_PIN  20
#define MSP_TX_PIN  21

HardwareSerial FC(0);

static uint8_t msp_crc(uint8_t*b, uint8_t len){
  uint8_t c=0; for(uint8_t i=0;i<len;i++) c^=b[i]; return c;
}
static bool msp_send(uint8_t cmd){
  uint8_t b[6]={'$','M','<',0,cmd,0};
  b[5]=msp_crc(&b[3],2); FC.write(b,6); return true;
}
static bool msp_read(uint8_t*payload, uint8_t&len, uint8_t&cmd){
  static enum{s0,s1,s2,s3,s4,s5,s6}st=s0;
  static uint8_t cs=0,p=0;
  while(FC.available()){
    uint8_t c=FC.read();
    switch(st){
      case s0: if(c=='$') st=s1; break;
      case s1: st=(c=='M')?s2:s0; break;
      case s2: st=(c=='>')?s3:s0; break;
      case s3: len=c; cs=c; p=0; st=s4; break;
      case s4: cmd=c; cs^=c; st=len?s5:s6; break;
      case s5: payload[p++]=c; cs^=c; if(p>=len) st=s6; break;
      case s6: st=s0; return(cs==c);
    }
  }
  return false;
}
static bool msp_get(uint8_t id, uint8_t*buf, uint8_t&sz){
  sz=0; msp_send(id);
  uint32_t t=millis(); uint8_t cmd=0;
  while(millis()-t<50){ if(msp_read(buf,sz,cmd)&&cmd==id) return true; }
  return false;
}
static bool g_armed = false; // declared here for readMSP (also used in ble_task/updateLed)

// Reads MSP_STATUS and updates g_armed
static void readMSP(){
  uint8_t buf[64], sz=0;
  if(msp_get(MSP_STATUS,buf,sz)&&sz>=10){
    uint32_t flags=*(uint32_t*)&buf[6];
    g_armed = (flags & 1);
  }
}

// =========================
// LED GPIO8 active LOW
// =========================
#define LED_PIN 8
inline void setLed(bool on){ digitalWrite(LED_PIN, on ? LOW : HIGH); }

// =========================
// BLE / DUML
// =========================
static const uint16_t DJI_COMPANY_ID  = 0x08AA;
static const char *SERVICE_UUID       = "0000fff0-0000-1000-8000-00805f9b34fb";
static const char *CHAR_WRITE_UUID    = "0000fff5-0000-1000-8000-00805f9b34fb";
static const char *CHAR_NOTIFY_UUID   = "0000fff4-0000-1000-8000-00805f9b34fb";

// ---------- CRC ----------
static uint32_t reverse_bits(uint32_t x,uint8_t w){
  uint32_t r=0; for(uint8_t i=0;i<w;i++) if(x&(1UL<<i)) r|=1UL<<(w-1-i); return r;
}
static uint32_t crc_generic(const uint8_t*d,size_t l,uint8_t w,uint32_t poly,uint32_t init,bool ri,bool ro){
  uint32_t top=1UL<<(w-1), mask=(w==32)?0xFFFFFFFFUL:((1UL<<w)-1), crc=init;
  for(size_t i=0;i<l;i++){
    uint8_t b=d[i]; if(ri) b=(uint8_t)reverse_bits(b,8);
    crc^=((uint32_t)b<<(w-8))&mask;
    for(int bit=0;bit<8;bit++) crc=(crc&top)?((crc<<1)^poly)&mask:(crc<<1)&mask;
  }
  if(ro) crc=reverse_bits(crc,w); return crc;
}
static uint8_t  crc8_dji (const uint8_t*d,size_t l){ return (uint8_t) crc_generic(d,l,8, 0x31,  0xEE,  true,true); }
static uint16_t crc16_dji(const uint8_t*d,size_t l){ return (uint16_t)crc_generic(d,l,16,0x1021,0x496C,true,true); }

static uint8_t g_duml_buf[64];
static size_t duml_build(uint8_t src,uint8_t dst,uint16_t id,uint8_t flags,uint8_t cs,uint8_t ci,const uint8_t*p,uint8_t pl){
  uint16_t tot=(uint16_t)pl+13; uint8_t*o=g_duml_buf;
  o[0]=0x55; o[1]=(uint8_t)(tot&0xFF); o[2]=(uint8_t)((1<<2)|((tot>>8)&0x03)); o[3]=crc8_dji(o,3);
  o[4]=src; o[5]=dst; o[6]=(uint8_t)((id>>8)&0xFF); o[7]=(uint8_t)(id&0xFF);
  o[8]=flags; o[9]=cs; o[10]=ci;
  if(pl) memcpy(&o[11],p,pl);
  uint16_t c=crc16_dji(o,11+pl); o[11+pl]=(uint8_t)(c&0xFF); o[12+pl]=(uint8_t)((c>>8)&0xFF);
  return (size_t)tot;
}

// ---------- State ----------
static NimBLEClient               *g_client        = nullptr;
static NimBLERemoteCharacteristic *g_writeChar     = nullptr;
static NimBLERemoteCharacteristic *g_notifyChar    = nullptr;
static bool                        g_connected     = false;
static uint16_t                    g_msg_id        = 0x0001;
static NimBLEAddress              *g_camera_addr   = nullptr;
static bool                        g_camera_addr_known = false;
static NimBLEAddress              *g_resolved_addr = nullptr;
static bool                        g_last_recording_known = false;
static bool                        g_last_recording       = false;
static bool                        g_armed_triggered      = false;
static bool                        g_camera_disabled      = false;
static bool                        g_gatt_cached          = false;

static const uint8_t g_empty[1] = {0x00};
static const uint8_t g_status_poll[1] = {0x01};

// =============================================================
// ELRS BACKPACK OSD via ESP-NOW
// Messages automatically cleared after OSD_DURATION_MS.
// =============================================================
#define OSD_ROW          2
#define MSP_DISPLAYPORT  182
#define OSD_DURATION_MS  2000 // display duration before auto-clear
#define OSD_SCREEN_COLS  50   // screen width (columns) — HD goggles such as HDZero
#define OSD_RIGHT_MARGIN 0    // margin from the right edge (negative = shifted further right)

static const uint8_t  BP_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // ELRS bind address
static uint8_t        g_bp_uid[6];
static bool           g_bp_ready  = false;
static uint8_t        g_msp_buf[80];
static SemaphoreHandle_t g_osd_mutex = nullptr;

// Computes the ELRS UID from BIND_PHRASE, exactly as the
// ExpressLRS configurator at compile time (build_flags.py):
// MD5('-DMY_BINDING_PHRASE="<phrase>"')[0:6], then uid[0] &= ~0x01
// (NOT MD5(phrase) alone — that was the bug in the old phraseToUID version).
static void computeBindUID(uint8_t uid[6]){
  char define_str[64];
  snprintf(define_str, sizeof(define_str), "-DMY_BINDING_PHRASE=\"%s\"", BIND_PHRASE);
  uint8_t hash[16];
  mbedtls_md5_context ctx;
  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts(&ctx);
  mbedtls_md5_update(&ctx,(const uint8_t*)define_str, strlen(define_str));
  mbedtls_md5_finish(&ctx, hash);
  mbedtls_md5_free(&ctx);
  memcpy(uid, hash, 6);
  uid[0] &= ~0x01; // valid unicast MAC (LSB bit at 0)
}

// CRC8-DVB-S2 (poly 0xD5) — identical to GENERIC_CRC8 in the Backpack's lib/CRC/crc.cpp
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a){
  crc ^= a;
  for(int i=0;i<8;i++) crc = (crc & 0x80) ? (uint8_t)((crc<<1)^0xD5) : (uint8_t)(crc<<1);
  return crc;
}

// Builds a NATIVE MSPv2 frame: $ X < flags fnLo fnHi szLo szHi payload... crc
// (NOT MSPv1 "$M<" — lib/MSP/msp.cpp of the ELRS Backpack ONLY accepts this format,
// used by both Tx_main.cpp and Vrx_main.cpp for ESP-NOW).
static size_t buildMSP(uint16_t function, const uint8_t*pl, uint16_t plen){
  size_t p=0;
  g_msp_buf[p++]='$'; g_msp_buf[p++]='X'; g_msp_buf[p++]='<';
  uint8_t crc=0;
  uint8_t flags=0;
  g_msp_buf[p]=flags;                          crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)(function&0xFF);       crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)((function>>8)&0xFF);  crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)(plen&0xFF);           crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  g_msp_buf[p]=(uint8_t)((plen>>8)&0xFF);      crc=crc8_dvb_s2(crc,g_msp_buf[p]); p++;
  for(uint16_t i=0;i<plen;i++){ g_msp_buf[p]=pl[i]; crc=crc8_dvb_s2(crc,pl[i]); p++; }
  g_msp_buf[p++]=crc;
  return p;
}

// Builds and sends ONE DisplayPort sub-command, WITHOUT taking the mutex:
// the caller must already hold g_osd_mutex (see osd_radio_lock/unlock).
// Serves as a building block to group several sub-commands
// (clear+writes+draw) under ONE single mutex take (optimization).
static void osd_raw_send(uint8_t subcmd, const uint8_t*extra=nullptr, uint8_t extra_len=0){
  if(!g_bp_ready) return;
  uint8_t payload[3+OSD_SCREEN_COLS+4]; // wide enough to blank a whole line (OSD_SCREEN_COLS spaces)
  payload[0]=subcmd;
  if(extra_len) memcpy(&payload[1],extra,extra_len);
  size_t ml=buildMSP(MSP_DISPLAYPORT, payload, 1+extra_len);
  // ESP-NOW has no application-level acknowledgement: we burst each
  // packet 3x (small spacing) to maximize the chances of reception
  // by the goggles.
  for(uint8_t burst=0; burst<3; burst++){
    esp_err_t serr=esp_now_send(g_bp_uid, g_msp_buf, ml);
    if(serr!=ESP_OK) Serial.printf("[BP] esp_now_send FAILED err=%d\n",(int)serr);
    vTaskDelay(pdMS_TO_TICKS(3));
  }
}

static void osd_radio_lock(){ xSemaphoreTake(g_osd_mutex, portMAX_DELAY); }
static void osd_radio_unlock(){ xSemaphoreGive(g_osd_mutex); vTaskDelay(pdMS_TO_TICKS(20)); }

// Sends an isolated DisplayPort subcommand (2=clear,3=write,4=draw), protected by
// mutex. To be used for a one-off send outside a batch.
static void osd_send_subcmd(uint8_t subcmd, const uint8_t*extra=nullptr, uint8_t extra_len=0){
  if(!g_bp_ready) return;
  osd_radio_lock();
  osd_raw_send(subcmd, extra, extra_len);
  osd_radio_unlock();
}

// Computes the column to align a text of length tlen on the
// right edge of the screen (OSD_SCREEN_COLS columns), with a small margin.
static uint8_t osd_right_col(uint8_t tlen){
  int16_t col=(int16_t)OSD_SCREEN_COLS-(int16_t)tlen-(int16_t)OSD_RIGHT_MARGIN;
  return (col>0)?(uint8_t)col:0;
}

// Writes spaces over the WHOLE width of "row" (col 0 to OSD_SCREEN_COLS):
// some OSD receivers do not visually erase the text already drawn
// with a simple clear_screen (subcmd 2) — they only redraw the
// cells that are explicitly rewritten. So we overwrite the whole line with
// spaces (and not only where the text was) since the text is now
// right-aligned, at a different position depending on its length.
// "raw" variant (no mutex): to be used in a batch where the caller
// already holds the lock (osd_radio_lock()).
static void osd_blank_row_raw(uint8_t row){
  uint8_t extra[3+OSD_SCREEN_COLS]; extra[0]=row; extra[1]=0; extra[2]=0; // row,col=0,attr
  memset(&extra[3], ' ', OSD_SCREEN_COLS);
  osd_raw_send(3, extra, 3+OSD_SCREEN_COLS);
}

// Isolated variant (takes the mutex itself) for a one-off call outside a batch.
static void osd_blank_row(uint8_t row){
  osd_radio_lock();
  osd_blank_row_raw(row);
  osd_radio_unlock();
}

// Generation counter: incremented on every new display. If two
// ARM/DISARM arrive less than OSD_DURATION_MS apart, several tasks
// auto-clear end up scheduled at the same time; without this safeguard,
// each one blanks the screen in turn -> double flash effect. Only the
// task whose generation still matches the latest one acts.
static volatile uint32_t g_osd_generation = 0;

// Parameters passed to osd_autoclear_task: captured at send time (no
// shared global variable) to avoid any race if a new
// display overwrites the lines before the previous auto-clear fires.
struct OsdAutoclearParam{ uint32_t delay_ms; uint8_t rows[4]; uint8_t count; uint32_t generation; };

// One-shot task: waits delay_ms then blanks the displayed lines + clear/draw,
// only if no new display happened in the meantime.
static void osd_autoclear_task(void*param){
  OsdAutoclearParam*p=(OsdAutoclearParam*)param;
  vTaskDelay(pdMS_TO_TICKS(p->delay_ms));
  if(p->generation==g_osd_generation){
    // All the blanking + clear + draw under ONE single mutex take.
    osd_radio_lock();
    for(uint8_t i=0;i<p->count;i++) osd_blank_row_raw(p->rows[i]);
    osd_raw_send(2); // clear
    osd_raw_send(4); // draw
    osd_radio_unlock();
    Serial.println("[BP] OSD auto-clear");
  } else {
    Serial.println("[BP] OSD auto-clear cancelled (new display in the meantime)");
  }
  delete p;
  vTaskDelete(nullptr);
}

struct OsdLine{ const char*text; uint8_t row; };

// Displays 1..n lines in a single clear/draw, then schedules the automatic
// clearing after duration_ms (0 = no auto-clear).
static void sendOSD_lines(const OsdLine*lines, uint8_t n, uint32_t duration_ms=OSD_DURATION_MS){
  if(!g_bp_ready) return;
  uint32_t my_gen=++g_osd_generation; // this display becomes the most recent
  // clear + all the lines + draw under ONE single mutex take.
  osd_radio_lock();
  osd_raw_send(2); // clear once for the whole batch
  for(uint8_t i=0;i<n;i++){
    uint8_t tlen=strlen(lines[i].text);
    uint8_t col=osd_right_col(tlen); // right-aligned on the screen
    uint8_t extra[3+64]; extra[0]=lines[i].row; extra[1]=col; extra[2]=0; // row,col,attr
    memcpy(&extra[3],lines[i].text,tlen);
    osd_raw_send(3, extra, 3+tlen);
    Serial.printf("[BP] OSD -> \"%s\" row %d col %d\n",lines[i].text,lines[i].row,col);
  }
  osd_raw_send(4); // draw once
  osd_radio_unlock();
  if(duration_ms>0){
    OsdAutoclearParam*p=new OsdAutoclearParam();
    p->delay_ms=duration_ms;
    p->count=(n>4)?4:n;
    for(uint8_t i=0;i<p->count;i++) p->rows[i]=lines[i].row;
    p->generation=my_gen;
    // xTaskCreate was never checked: on the very first ARM, the heap is
    // under heavy pressure right after the 1st BLE/GATT connection to the
    // camera, and creating the auto-clear task can fail
    // silently -> the screen stays frozen forever. We detect
    // the failure and do an immediate fallback clear (better than nothing).
    BaseType_t tcr=xTaskCreate(osd_autoclear_task,"osd_clr",2048,p,1,nullptr);
    if(tcr!=pdPASS){
      Serial.printf("[BP] xTaskCreate(osd_clr) FAILED free_heap=%u -> immediate clear\n",(unsigned)ESP.getFreeHeap());
      delete p;
      osd_radio_lock();
      for(uint8_t i=0;i<n;i++) osd_blank_row_raw(lines[i].row);
      osd_raw_send(2);
      osd_raw_send(4);
      osd_radio_unlock();
    }
  }
}

// Compat: a single message on one line
static void sendOSD(const char*text, uint8_t row=OSD_ROW, uint32_t duration_ms=OSD_DURATION_MS){
  OsdLine l={text,row};
  sendOSD_lines(&l,1,duration_ms);
}

static void initBackpack(){
  g_osd_mutex=xSemaphoreCreateMutex();
  // UID computed from BIND_PHRASE (see computeBindUID above)
  computeBindUID(g_bp_uid);
  Serial.printf("[BP] UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
    g_bp_uid[0],g_bp_uid[1],g_bp_uid[2],g_bp_uid[3],g_bp_uid[4],g_bp_uid[5]);
  // CRITICAL: our STA MAC must = UID (the VRX checks mac_addr == firmwareOptions.uid).
  // esp_wifi_set_mac() can fail silently depending on the WiFi driver state
  // -> we re-read the real MAC right after to confirm it took effect.
  esp_err_t merr=esp_wifi_set_mac(WIFI_IF_STA, g_bp_uid);
  if(merr!=ESP_OK) Serial.printf("[BP] esp_wifi_set_mac FAILED err=%d\n",(int)merr);
  uint8_t mac_check[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac_check);
  Serial.printf("[BP] actual STA MAC after set: %02X:%02X:%02X:%02X:%02X:%02X %s\n",
    mac_check[0],mac_check[1],mac_check[2],mac_check[3],mac_check[4],mac_check[5],
    (memcmp(mac_check,g_bp_uid,6)==0)?"(OK = UID)":"(!!! DOES NOT MATCH THE UID !!!)");
  if(esp_now_init()!=ESP_OK){ Serial.println("[BP] ESP-NOW init FAILED"); return; }
  // Peer = UID (MAC of the VRX backpack). channel=0 = "current WiFi channel":
  // esp_now_add_peer() rejects a fixed peer.channel that does not match
  // the radio channel active at the time of the call — 0 avoids any risk of
  // mismatch (even though the channel is already set to 1 before this call).
  esp_now_peer_info_t peer={}; memcpy(peer.peer_addr,g_bp_uid,6);
  peer.channel=0; peer.encrypt=false;
  esp_err_t perr=esp_now_add_peer(&peer);
  if(perr!=ESP_OK) Serial.printf("[BP] esp_now_add_peer FAILED err=%d\n",(int)perr);
  // Broadcast peer, required to send an MSP_ELRS_BIND (see sendBackpackBind)
  esp_now_peer_info_t bpeer={}; memcpy(bpeer.peer_addr,BP_BROADCAST,6);
  bpeer.channel=0; bpeer.encrypt=false;
  esp_err_t berr=esp_now_add_peer(&bpeer);
  if(berr!=ESP_OK) Serial.printf("[BP] esp_now_add_peer(broadcast) FAILED err=%d\n",(int)berr);
  g_bp_ready=true; Serial.println("[BP] Backpack ready");
}

// =============================================================
// BIND ELRS BACKPACK
// The VRX Backpack only accepts an MSP_ELRS_BIND (function=0x09) if it is
// in binding mode (never configured with a bind phrase, or put back in
// binding via 3 quick power cycles — see checkIfInBindingMode
// in Vrx_main.cpp). In this state it ignores the MAC filter and adopts
// the payload (6 bytes) directly as its new group/UID, then
// restarts on it. So we broadcast our own UID (g_bp_uid)
// several times to maximize the chances of it being received.
// =============================================================
#define MSP_ELRS_BIND 0x09

static void sendBackpackBind(){
  if(!g_bp_ready){ Serial.println("[BP] Bind impossible: backpack not ready"); return; }
  Serial.println("[BP] Sending BIND (broadcast) - make sure the VRX is in binding mode");
  for(int i=0;i<10;i++){
    xSemaphoreTake(g_osd_mutex, portMAX_DELAY);
    size_t ml=buildMSP(MSP_ELRS_BIND, g_bp_uid, 6);
    esp_err_t serr=esp_now_send(BP_BROADCAST, g_msp_buf, ml);
    if(serr!=ESP_OK) Serial.printf("[BP] bind esp_now_send FAILED err=%d\n",(int)serr);
    xSemaphoreGive(g_osd_mutex);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.println("[BP] BIND sent (10x). The VRX should reboot if it worked.");
}

// ---------- Combined LED ----------
// Solid     = no cam, armed or not
// Slow 1Hz  = cam ready (gatt cached), not armed
// Fast 5Hz  = cam ready + armed (recording)
// g_armed declared above (before readMSP)
void updateLed(){
  if(!g_gatt_cached){ setLed(true); return; }
  if(g_armed){ setLed((millis()%200)<100); return; }
  setLed((millis()%1000)<500);
}

// ---------- Notify ----------
static const char* decode_rec(uint8_t b){
  switch(b){ case 0x01:return "IDLE"; case 0x41:return "STARTING"; case 0x81:return "RECORDING"; case 0xc1:return "STOPPING"; default:return "?"; }
}
static void notifyCallback(NimBLERemoteCharacteristic*,uint8_t*d,size_t l,bool){
  if(l<19||d[9]!=0x02||d[10]!=0x70) return;
  uint8_t s=d[12]; bool rec=(s==0x81);
  if(!g_last_recording_known||rec!=g_last_recording||s==0x41||s==0xc1)
    Serial.printf("[DJI] STATE: %s\n",decode_rec(s));
  g_last_recording=rec; g_last_recording_known=true;
}

// ---------- BLE helpers ----------
static void poll_status(){
  if(!g_connected||!g_writeChar) return;
  size_t l=duml_build(0x53,0x01,g_msg_id++,0x20,0x02,0x70,g_status_poll,1);
  g_writeChar->writeValue(g_duml_buf,l,false);
}
static void send_heartbeats(){
  if(!g_connected||!g_writeChar) return;
  size_t l=duml_build(0x53,0x28,g_msg_id++,0x20,0x00,0x00,g_empty,0);
  g_writeChar->writeValue(g_duml_buf,l,false);
  l=duml_build(0x53,0x05,g_msg_id++,0x20,0x0d,0x02,g_empty,0);
  g_writeChar->writeValue(g_duml_buf,l,false);
}
static void send_take_record(bool start){
  if(!g_connected||!g_writeChar) return;
  uint8_t p[1]={(uint8_t)(start?0x01:0x00)};
  size_t l=duml_build(0x53,0x01,g_msg_id++,0x20,0x02,0x02,p,1);
  g_writeChar->writeValue(g_duml_buf,l,false);
}

static bool looks_like_dji(const NimBLEAdvertisedDevice*dev){
  if(dev->isAdvertisingService(NimBLEUUID((uint16_t)0xFFF0))) return true;
  if(dev->haveManufacturerData()){
    std::string m=dev->getManufacturerData();
    if(m.length()>=2&&((uint8_t)m[0]|((uint8_t)m[1]<<8))==DJI_COMPANY_ID) return true;
  }
  return false;
}
static bool discover_camera(uint32_t sec=2){
  Serial.printf("[DJI] scan BLE (%us)...\n",sec);
  NimBLEScan*s=NimBLEDevice::getScan(); s->setActiveScan(false);
  NimBLEScanResults r=s->getResults(sec*1000,false);
  NimBLEAddress best; int best_rssi=-999; bool found=false;
  for(int i=0;i<r.getCount();i++){
    const NimBLEAdvertisedDevice*d=r.getDevice(i);
    if(!looks_like_dji(d)) continue;
    int rssi=d->getRSSI();
    Serial.printf("[DJI] found: %s RSSI=%d\n",d->getAddress().toString().c_str(),rssi);
    if(!found||rssi>best_rssi){ best=d->getAddress(); best_rssi=rssi; found=true; }
  }
  s->clearResults();
  if(!found){ Serial.println("[DJI] no camera"); return false; }
  if(g_camera_addr) delete g_camera_addr;
  g_camera_addr=new NimBLEAddress(best); g_camera_addr_known=true;
  Serial.printf("[DJI] camera: %s RSSI=%d\n",g_camera_addr->toString().c_str(),best_rssi);
  return true;
}

class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*,int){ g_connected=false; }
};
static ClientCB g_clientCB;

static bool try_connect_once(int n){
  if(!g_camera_addr_known){ if(!discover_camera()) return false; }
  Serial.printf("[DJI] connecting to %s (attempt %d)...\n",g_camera_addr->toString().c_str(),n);
  if(!g_client){ g_client=NimBLEDevice::createClient(); g_client->setClientCallbacks(&g_clientCB,false); }
  if(g_client->isConnected()){ g_client->disconnect(); vTaskDelay(pdMS_TO_TICKS(200)); }
  g_client->setConnectionParams(48,48,0,400);
  bool cached=(g_writeChar&&g_resolved_addr&&*g_resolved_addr==*g_camera_addr);
  if(!g_client->connect(*g_camera_addr,!cached)){ Serial.println("[DJI] connection failed"); g_camera_addr_known=false; return false; }
  if(!cached){
    vTaskDelay(pdMS_TO_TICKS(250));
    if(!g_client->isConnected()) return false;
    auto*svc=g_client->getService(SERVICE_UUID);
    if(!svc){ if(g_client->isConnected()) g_client->disconnect(); return false; }
    g_writeChar =svc->getCharacteristic(CHAR_WRITE_UUID);
    g_notifyChar=svc->getCharacteristic(CHAR_NOTIFY_UUID);
    if(!g_writeChar||!g_writeChar->canWriteNoResponse()){ if(g_client->isConnected()) g_client->disconnect(); return false; }
    if(g_resolved_addr) delete g_resolved_addr;
    g_resolved_addr=new NimBLEAddress(*g_camera_addr);
  }
  if(g_notifyChar&&g_notifyChar->canNotify()) g_notifyChar->subscribe(true,notifyCallback);
  g_connected=true;
  Serial.printf("[DJI] params: interval=%.1fms latency=%d\n",
    g_client->getConnInfo().getConnInterval()*1.25f,
    g_client->getConnInfo().getConnLatency());
  return true;
}
static bool connect_camera(){
  for(int i=1;i<=4;i++){ if(try_connect_once(i)) return true; if(i<4) vTaskDelay(pdMS_TO_TICKS(300)); }
  Serial.println("[DJI] giving up"); return false;
}

static bool wait_for_recording_state(bool want){
  unsigned long t=millis();
  while(millis()-t<5000){
    if(!g_connected) return false;
    poll_status();
    unsigned long p=millis();
    while(millis()-p<300){ vTaskDelay(pdMS_TO_TICKS(20)); if(g_last_recording_known&&g_last_recording==want) return true; }
  }
  return false;
}

static bool connect_send_disconnect(bool start){
  if(!start){
    // Immediate, unconditional clear on disarm: previously, if the camera
    // was unreachable (BLE disabled or connect_camera() failed), the
    // function returned further down WITHOUT ever calling sendOSD -> no clear.
    osd_radio_lock(); osd_raw_send(2); osd_raw_send(4); osd_radio_unlock();
  }
  if(g_camera_disabled){ Serial.println("[DJI] BLE disabled"); return false; }
  if(!start){ Serial.println("[DJI] DISARMED -> immediate STOP"); }
  bool first=(start&&!g_armed_triggered);
  if(start) g_armed_triggered=true;
  Serial.printf("[DJI] %s\n",start?"ARMED -> START":"DISARMED -> STOP");
  g_last_recording_known=false;
  if(!connect_camera()){
    if(first){ g_camera_disabled=true; Serial.println("[DJI] no camera -> BLE disabled"); sendOSD("NO C"); }
    return false;
  }
  send_take_record(start);
  bool ok=wait_for_recording_state(start);
  Serial.printf("[DJI] %s\n",ok?"CONFIRMED":"NOT CONFIRMED");
  // "C REC" reflects a permanent state while recording -> no
  // auto-clear (duration_ms=0). "C STP" keeps the default auto-clear,
  // and its own clear_screen clears the previous "C REC" along the way.
  if(ok){
    if(start) sendOSD("C REC", OSD_ROW, 0);
    else      sendOSD("C STP");
  }
  else   sendOSD("ERR");
  if(g_client&&g_client->isConnected()) g_client->disconnect();
  g_connected=false; return ok;
}

static void prewarm_camera_cache(){
  if(g_armed_triggered) return;
  if(!discover_camera()) return;
  vTaskDelay(pdMS_TO_TICKS(300));
  if(try_connect_once(1)){
    Serial.println("[DJI] GATT cache ready");
    send_heartbeats();
    vTaskDelay(pdMS_TO_TICKS(300));
    if(g_client&&g_client->isConnected()) g_client->disconnect();
    g_connected=false; g_gatt_cached=true;
    // duration_ms=0: no auto-clear, the message stays displayed until the
    // first arming (sendOSD_lines does a clear_screen before writing
    // "REC", so "C OK" is naturally cleared at that point).
    sendOSD("C OK", OSD_ROW, 0);
  } else {
    g_camera_addr_known=true;
  }
}

// =========================
// TÂCHE FREERTOS BLE
// =========================
static void ble_task(void*){
  prewarm_camera_cache();
  bool prev=false;
  while(true){
    bool cur=g_armed;
    if(cur!=prev){ connect_send_disconnect(cur); prev=cur; }
    if(!g_armed_triggered&&!g_gatt_cached) prewarm_camera_cache();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =========================
// MAIN
// =========================
void setup(){
  Serial.begin(115200);
  delay(300);
  FC.begin(MSP_BAUD, SERIAL_8N1, MSP_RX_PIN, MSP_TX_PIN);
  pinMode(LED_PIN,OUTPUT); setLed(true);
  WiFi.mode(WIFI_MODE_STA);
  // Channel 1: the ELRS VRX Backpack listens to ESP-NOW on this channel (hardcoded)
  // (see SetSoftMACAddress in Vrx_main.cpp/Tx_main.cpp: WiFi.begin(...,1))
  // No DGAC beacon here so no channel 6 constraint -> we stay on 1.
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(78); // 19.5 dBm = maximum exposed by the API (units of 0.25dBm)
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  initBackpack();
  NimBLEDevice::init("");
  NimBLEDevice::setPowerLevel(ESP_PWR_LVL_P9);
  xTaskCreate(ble_task,"ble_dji",8192,nullptr,1,nullptr);
  Serial.println("[DJI V11] ready - ARM/DISARM via MSP FC (GPIO20/21) + Backpack OSD");
}

void loop(){
  readMSP();   // reads MSP_STATUS -> updates g_armed
  while(Serial.available()){
    int c=Serial.read();
    if(c=='b'||c=='B'){ sendBackpackBind(); } // starts the ELRS Backpack bind
  }
  updateLed();
  delay(10);
}
