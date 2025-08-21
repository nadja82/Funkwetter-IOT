/*
 * Funkwetter – ESP8266 NodeMCU + SSD1309 (SPI)
 * - OLED + Web + mDNS + NTP
 * - VHF/UHF (2 m, 70 cm): dN/dz (2 Höhen) oder Heuristik (1 Höhe)
 * - 10 m: On-Air-Proxy oder Fallback (F10.7/Kp/Tag/Saison)
 * - Solar (F10.7, Kp) + Flare (GOES)
 * - Remote Sensor Pull (zweiter ESP: /sensors.json) -> P/T/RH/z1 (2 Versuche)
 * - EEPROM persist (inkl. Remote- & Overlay-Einstellungen)
 * - ASCII-sicherer Ticker + RAW-Overlay (Ticker läuft in beiden Screens)
 */

#include <Arduino.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClientSecureBearSSL.h>
using TLSClient = BearSSL::WiFiClientSecure;

#include <time.h>
#include <U8g2lib.h>
#include <EEPROM.h>

// -------- Standort / Zeit --------
const double LAT = 51.50;
const double LON = 7.50;
const char*  TZ_EU_BERLIN = "CET-1CEST,M3.5.0/02,M10.5.0/03";

// -------- WLAN / mDNS -------------
const char* WIFI_SSID = "Wilma2001_Ext";
const char* WIFI_PASS = "14D12k82";
const char* HOSTNAME  = "funkwetter";
WiFiServer httpServer(80);

// -------- Eingaben (lokal) --------
float L_P1=NAN, L_T1=NAN, L_RH1=NAN, L_z1=NAN;
float L_P2=NAN, L_T2=NAN, L_RH2=NAN, L_z2=NAN;
float H_SPOTS15=NAN, H_DXKM=NAN, H_SNRDB=NAN;

// -------- Resultate ----------------
float L_N1=NAN, L_N2=NAN, L_dNdz=NAN, L_k=NAN;
String L_amp_2m="unbekannt", L_amp_70cm="unbekannt";
int    L_score2m=0, L_score70=0;
String L_score2m_lbl="unbekannt", L_score70_lbl="unbekannt";

int    H_score10=0;
String H_label10="unbekannt";

// -------- Flare / Solar ------------
bool   g_flare_enabled = false;
bool   g_flare_dayOnly = true;
String FL_class=""; String FL_peak="";
int    FL_penalty=0;
unsigned long FL_fetched_ms=0;

bool   g_solar_enabled = true;
bool   g_solar_adjust10 = true;
unsigned long SOL_fetched_ms=0;
float  SOL_f107=NAN, SOL_kp=NAN;
int    SOL_score=0; String SOL_label="unbekannt";
int    SOL_applied_delta10=0;

// -------- Skala / Optionen ----------
int  g_score_max = 10;     // 10 oder 5
bool g_splash_enabled = true;

bool g_vhf_heuristic_enable = true;
bool FLAG_VHF_HEUR_USED=false;
bool FLAG_HF10_FALLBACK_USED=false;

bool g_today_override_vhf=false;
int  g_today_score_vhf10=8;

// -------- Zeit / Licht --------------
bool   g_time_ok=false;
String g_time_str="", g_rise_str="", g_set_str="";
bool   g_is_daylight=true;

// -------- OLED SPI SSD1309 ----------
#define OLED_CS   5   // D1
#define OLED_DC   4   // D2
#define OLED_RST 16  // D0
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);

float disp2m=0, disp70=0, disp10=0, dispSolar=0;
unsigned long lastDispMs=0;
int OLED_CONTRAST=200;

// ---- Sanfte Solar-Animation (Throttle) ----
unsigned long SOL_ANIM_MS = 0;
uint16_t SOL_ANIM_PHASE = 0;

// ---- Ticker -------------------------
String OLED_TICKER=""; int OLED_TICKER_X=128; unsigned long OLED_TICKER_MS=0;

// ---- Overlay-Timer (RAW-Anzeige) ----
bool     g_overlay_enabled = true;         // persistiert
uint32_t OVERLAY_PERIOD_MS = 10000;        // persistiert
uint32_t OVERLAY_DURATION_MS = 2000;       // persistiert
bool OLED_OVERLAY=false; 
unsigned long OLED_OVERLAY_END=0; 
unsigned long OLED_OVERLAY_NEXT=0; 

// Bootstrap nach Splash
bool g_bootstrap_done=false; unsigned long g_bootstrap_at_ms=0;

// -------- Remote Sensor -------------
bool   g_remote_enabled=false;
String g_remote_host   ="funk-remote.local";
bool   REMOTE_OK=false; String REMOTE_ERR=""; unsigned long REMOTE_LAST_MS=0; uint8_t REMOTE_FAILS=0;
float  R_P_hPa=NAN, R_T_C=NAN, R_RH_pct=NAN, R_z_m=NAN;

// -------- Störungen (%) -------------
int    DIST_prob_pct=0; String DIST_label="niedrig";

// -------- EEPROM-State --------------
const uint32_t STATE_MAGIC=0xF00C57A1;
struct SavedState {
  uint32_t magic;
  float P1,T1,RH1,z1, P2,T2,RH2,z2;
  float SPOTS15, DXKM, SNRDB;
  uint8_t flags;        // bit0 flare,1 dayonly,2 solar,3 solar_adj10,4 splash,5 vhf_heur,6 today_override
  uint8_t score_max;    // 5/10
  uint8_t today_score;  // 1..10
  uint8_t contrast;     // 0..255
  char   remote_host[48];
  uint8_t remote_enabled;
  // Overlay-Settings
  uint8_t overlay_enabled;   // 0/1
  uint16_t overlay_period_s; // 3..600 s
  uint16_t overlay_dur_ms;   // 500..15000 ms
};

void saveState(){
  SavedState s{}; s.magic=STATE_MAGIC;
  s.P1=L_P1; s.T1=L_T1; s.RH1=L_RH1; s.z1=L_z1;
  s.P2=L_P2; s.T2=L_T2; s.RH2=L_RH2; s.z2=L_z2;
  s.SPOTS15=H_SPOTS15; s.DXKM=H_DXKM; s.SNRDB=H_SNRDB;
  s.flags=(g_flare_enabled?1:0)|(g_flare_dayOnly?2:0)|(g_solar_enabled?4:0)|(g_solar_adjust10?8:0)|
          (g_splash_enabled?16:0)|(g_vhf_heuristic_enable?32:0)|(g_today_override_vhf?64:0);
  s.score_max=(uint8_t)g_score_max; s.today_score=(uint8_t)g_today_score_vhf10; s.contrast=(uint8_t)OLED_CONTRAST;
  memset(s.remote_host,0,sizeof(s.remote_host)); g_remote_host.toCharArray(s.remote_host,sizeof(s.remote_host));
  s.remote_enabled = g_remote_enabled ? 1 : 0;
  // Overlay
  s.overlay_enabled = g_overlay_enabled ? 1 : 0;
  s.overlay_period_s = (uint16_t)min<uint32_t>(OVERLAY_PERIOD_MS/1000UL, 65535);
  s.overlay_dur_ms   = (uint16_t)min<uint32_t>(OVERLAY_DURATION_MS, 65535);

  EEPROM.begin(512);
  const uint8_t* p=(const uint8_t*)&s; for (size_t i=0;i<sizeof(SavedState);++i) EEPROM.write(i,p[i]);
  EEPROM.commit();
}
bool loadState(){
  EEPROM.begin(512); SavedState s; uint8_t* p=(uint8_t*)&s; for (size_t i=0;i<sizeof(SavedState);++i) p[i]=EEPROM.read(i);
  if (s.magic!=STATE_MAGIC) return false;
  L_P1=s.P1; L_T1=s.T1; L_RH1=s.RH1; L_z1=s.z1;
  L_P2=s.P2; L_T2=s.T2; L_RH2=s.RH2; L_z2=s.z2;
  H_SPOTS15=s.SPOTS15; H_DXKM=s.DXKM; H_SNRDB=s.SNRDB;
  g_flare_enabled=(s.flags&1); g_flare_dayOnly=(s.flags&2); g_solar_enabled=(s.flags&4); g_solar_adjust10=(s.flags&8);
  g_splash_enabled=(s.flags&16); g_vhf_heuristic_enable=(s.flags&32); g_today_override_vhf=(s.flags&64);
  if (s.score_max==5||s.score_max==10) g_score_max=s.score_max;
  if (s.today_score>=1&&s.today_score<=10) g_today_score_vhf10=s.today_score;
  if (s.contrast>0){ OLED_CONTRAST=s.contrast; u8g2.setContrast(OLED_CONTRAST); }
  if (s.remote_host[0]) g_remote_host=String(s.remote_host);
  g_remote_enabled=(s.remote_enabled==1);
  // Overlay
  g_overlay_enabled = (s.overlay_enabled==1);
  if (s.overlay_period_s>=3 && s.overlay_period_s<=600) OVERLAY_PERIOD_MS = (uint32_t)s.overlay_period_s*1000UL;
  if (s.overlay_dur_ms>=500 && s.overlay_dur_ms<=15000) OVERLAY_DURATION_MS = (uint32_t)s.overlay_dur_ms;
  if (OVERLAY_DURATION_MS > OVERLAY_PERIOD_MS-250) OVERLAY_DURATION_MS = OVERLAY_PERIOD_MS-250;
  return true;
}

// -------- Helpers -------------------
float clamp01(float x){ if(x<0)return 0; if(x>1)return 1; return x; }
String f2(float v,int d=1){ return isnan(v)?"—":String(v,d); }

// ASCII-Sanitizer für Ticker
String ascii_de(String s){
  s.replace("\xC3\xA4","ae"); s.replace("\xC3\xB6","oe"); s.replace("\xC3\xBC","ue");
  s.replace("\xC3\x84","Ae"); s.replace("\xC3\x96","Oe"); s.replace("\xC3\x9C","Ue");
  s.replace("\xC3\x9F","ss");
  s.replace("\xC2\xB0"," deg");
  s.replace("\xE2\x80\x93","-"); s.replace("\xE2\x80\x94","-"); s.replace("\xE2\x80\xA6","...");
  s.replace("Tropo möglich","Tropo moeglich");
  s.replace("bedingt Tropo möglich","bedingt Tropo moeglich");
  s.replace("leicht erhöht","leicht erhoeht");
  s.replace("gedämpft","gedaempft");
  s.replace("überdurchschnittlich","ueberdurchschnittlich");
  s.replace("Störungen","Stoerungen");
  s.replace("°C"," C");
  return s;
}

int scoreFromSScaled(float S){
  S=clamp01(S);
  if(g_score_max==10){ int s=(int)round(1+9*S); s=constrain(s,1,10); return s; }
  int s=(int)round(5*S); s=constrain(s,0,5); return s;
}
int scoreForLabels10(int s){ if(g_score_max==10) return max(1,s); int m=(int)round(s*10.0/5.0); return constrain(m,1,10); }
int scaleDeltaFrom10(int d10){ if(g_score_max==10) return d10; int d=(int)round(d10*(5.0/10.0)); if(d==0&&d10!=0)d=(d10>0?1:-1); return d; }

String labelForVHFScore10(int s){ if(s>=9)return "Tropo möglich"; if(s>=7)return "bedingt Tropo möglich"; if(s>=4)return "leicht erhöht"; return "normal"; }
String labelForHF10(int s){ if(s>=9)return "Band offen"; if(s>=7)return "teils offen"; if(s>=4)return "mäßig"; return "gedämpft/geschlossen"; }
String labelForSolar(int s){ if(s>=9)return "hoch"; if(s>=7)return "überdurchschnittlich"; if(s>=4)return "moderat"; return "niedrig"; }

// -------- Physik --------------------
double Es_saturation(double T_C){ return 6.112*exp((17.62*T_C)/(243.12+T_C)); }
double refractivityN(double P_hPa,double T_C,double RH_percent){
  double Tk=T_C+273.15; double e=(RH_percent/100.0)*Es_saturation(T_C);
  return 77.6*(P_hPa/Tk) + 3.73e5*(e/(Tk*Tk));
}

// -------- VHF/UHF (2 Höhen) --------
void computeLocalTroposphere(){
  if (isnan(L_P1)||isnan(L_T1)||isnan(L_RH1)||isnan(L_z1)||
      isnan(L_P2)||isnan(L_T2)||isnan(L_RH2)||isnan(L_z2)){
    L_N1=L_N2=L_dNdz=L_k=NAN; L_amp_2m=L_amp_70cm="(unvollständig)";
    L_score2m=L_score70=0; L_score2m_lbl=L_score70_lbl="unbekannt"; return;
  }
  L_N1=refractivityN(L_P1,L_T1,L_RH1);
  L_N2=refractivityN(L_P2,L_T2,L_RH2);
  double dz=(L_z2-L_z1); if(fabs(dz)<1e-3){ L_dNdz=L_k=NAN; L_amp_2m=L_amp_70cm="(z2=z1)"; L_score2m=L_score70=0; L_score2m_lbl=L_score70_lbl="unbekannt"; return; }
  L_dNdz=(L_N2-L_N1)/dz*1000.0; // N/km
  L_k=157.0/(157.0+L_dNdz);

  if(L_dNdz<-157.0){ L_amp_2m="Tropo möglich"; L_amp_70cm="Tropo möglich (konservativ)"; }
  else if(L_dNdz<-79.0){ L_amp_2m="erhöhte Reichweite"; L_amp_70cm="erhöhte Reichweite"; }
  else { L_amp_2m=L_amp_70cm="normal"; }

  float S=clamp01((float)((-L_dNdz-39.0)/(157.0-39.0)));
  L_score2m=scoreFromSScaled(S*0.95f);
  L_score70=scoreFromSScaled(S*1.05f);
  L_score2m_lbl=labelForVHFScore10(scoreForLabels10(L_score2m));
  L_score70_lbl=labelForVHFScore10(scoreForLabels10(L_score70));
}

// -------- VHF Heuristik (1 Höhe) ---
void computeVHFHeuristicIfSingleHeight(){
  FLAG_VHF_HEUR_USED=false; if(!g_vhf_heuristic_enable) return;
  bool have1=!(isnan(L_P1)||isnan(L_T1)||isnan(L_RH1)); bool have2=!(isnan(L_P2)||isnan(L_T2)||isnan(L_RH2)||isnan(L_z2));
  if(!have1||have2||!isnan(L_dNdz)) return;
  float Hpress=clamp01((L_P1-1013.0f)/12.0f);
  float Night=g_is_daylight?0.0f:1.0f;
  float favRH; if(isnan(L_RH1)) favRH=0.5f; else { float d=fabsf(L_RH1-60.0f); favRH=1.0f-clamp01(d/40.0f); }
  float S=clamp01(0.6f*Hpress+0.3f*Night+0.1f*favRH);
  L_score2m=scoreFromSScaled(S*0.95f); L_score70=scoreFromSScaled(S*1.05f);
  L_score2m_lbl=labelForVHFScore10(scoreForLabels10(L_score2m)); L_score70_lbl=labelForVHFScore10(scoreForLabels10(L_score70));
  L_amp_2m=L_amp_70cm="(Heuristik)"; FLAG_VHF_HEUR_USED=true;
}

// -------- Heute-Override ------------
void applyTodayOverrideVHF(){
  if(!g_today_override_vhf) return;
  int s10=constrain(g_today_score_vhf10,1,10);
  int sScaled=(g_score_max==10)?s10:(int)round(s10*5.0/10.0);
  L_score2m=L_score70=sScaled;
  L_score2m_lbl=L_score70_lbl=labelForVHFScore10(s10);
}

// -------- Zeit / Daylight -----------
bool getLocalTimeSafe(struct tm &out){ time_t now=time(nullptr); if(now<8*3600UL) return false; localtime_r(&now,&out); return true; }

void updateDaylight(){
  struct tm lt; g_time_ok=getLocalTimeSafe(lt);
  if(!g_time_ok){ g_time_str="(Zeit unsynchron)"; g_is_daylight=true; g_rise_str=g_set_str="—"; return; }
  char buf[32]; strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S %Z",&lt); g_time_str=String(buf);
  int doy=lt.tm_yday+1; double hour=lt.tm_hour+lt.tm_min/60.0+lt.tm_sec/3600.0;
  double gamma=2.0*M_PI/365.0*(doy-1+(hour-12.0)/24.0);
  double EoT=229.18*(0.000075+0.001868*cos(gamma)-0.032077*sin(gamma)-0.014615*cos(2*gamma)-0.040849*sin(2*gamma));
  double decl=0.006918-0.399912*cos(gamma)+0.070257*sin(gamma)-0.006758*cos(2*gamma)+0.000907*sin(2*gamma)-0.002697*cos(3*gamma)+0.00148*sin(3*gamma);
  int tz_hours=1+(lt.tm_isdst>0?1:0); double Lst=15.0*tz_hours;
  double h0=-4.0*M_PI/180.0, phi=LAT*M_PI/180.0;
  double cosH0=(sin(h0)-sin(phi)*sin(decl))/(cos(phi)*cos(decl)); cosH0=constrain(cosH0,-1.0,1.0);
  double H0deg=acos(cosH0)*180.0/M_PI;
  double LST_rise=12.0-H0deg/15.0, LST_set=12.0+H0deg/15.0;
  double corr=(EoT+4.0*(LON-Lst))/60.0;
  auto wrap=[](double h){while(h<0)h+=24;while(h>=24)h-=24;return h;};
  double LT_rise=wrap(LST_rise-corr), LT_set=wrap(LST_set-corr);
  auto hhmm=[&](double h){int hh=(int)floor(h+1e-6); int mm=(int)round((h-hh)*60.0); if(mm==60){hh=(hh+1)%24;mm=0;} char b[6]; snprintf(b,sizeof(b),"%02d:%02d",hh,mm); return String(b);};
  g_rise_str=hhmm(LT_rise); g_set_str=hhmm(LT_set);
  double nowH=hour; g_is_daylight=(LT_rise<LT_set)?(nowH>=LT_rise && nowH<LT_set):!(nowH>=LT_set && nowH<LT_rise);
}

// Tages-/Jahreszeit-Faktoren
float dayTimeFactor(){
  struct tm lt; if(!getLocalTimeSafe(lt)) return 0.5f;
  float hour=lt.tm_hour+lt.tm_min/60.0f;
  if(!g_is_daylight) return 0.12f;
  const float mu=13.0f, sigma=4.5f; float z=(hour-mu)/sigma; float v=expf(-0.5f*z*z); return clamp01(v);
}
float seasonEquinoxFactor(){
  struct tm lt; if(!getLocalTimeSafe(lt)) return 0.6f;
  int doy=lt.tm_yday+1; float x=4.0f*M_PI*((doy-80)/365.0f); float v=0.5f*(cosf(x)+1.0f); return v;
}

// -------- HTTPS GET (SWPC) ----------
bool httpsGET(const char* host,const char* path,String& body,uint32_t maxLen=8192,uint32_t timeoutMs=8000){
  TLSClient cli; cli.setInsecure(); if(!cli.connect(host,443)) return false;
  cli.print(String("GET ")+path+" HTTP/1.1\r\nHost: "+host+"\r\nUser-Agent: ESP-Funkwetter\r\nConnection: close\r\n\r\n");
  String resp; resp.reserve(maxLen); unsigned long t0=millis();
  while((cli.connected()||cli.available()) && resp.length()<(int)maxLen){
    if(cli.available()) resp+=(char)cli.read();
    if(millis()-t0>timeoutMs) break; delay(1);
  }
  cli.stop(); int hdr=resp.indexOf("\r\n\r\n"); if(hdr<0) return false; body=resp.substring(hdr+4); return true;
}

// -------- Plain HTTP GET ------------
bool httpGET(const String& hostOrIp,const char* path,String& body,uint32_t maxLen=4096,uint32_t timeoutMs=5000){
  IPAddress ip; if(!WiFi.hostByName(hostOrIp.c_str(),ip)) return false;
  WiFiClient cli; if(!cli.connect(ip,80)) return false;
  String req=String("GET ")+path+" HTTP/1.1\r\nHost: "+hostOrIp+"\r\nConnection: close\r\n\r\n";
  cli.print(req);
  String resp; resp.reserve(maxLen); unsigned long t0=millis();
  while((cli.connected()||cli.available()) && resp.length()<(int)maxLen){
    if(cli.available()) resp+=(char)cli.read();
    if(millis()-t0>timeoutMs) break; delay(1);
  }
  cli.stop(); int hdr=resp.indexOf("\r\n\r\n"); if(hdr<0) return false; body=resp.substring(hdr+4); return true;
}

// -------- Flare (GOES) ---------------
bool parseFlareClass(const String& body,char &cls,float &mag){
  for(int i=0;i+2<body.length();++i){ char c=body[i];
    if((c=='A'||c=='B'||c=='C'||c=='M'||c=='X') && isDigit(body[i+1])){
      int j=i+1; while(j<body.length()&&(isDigit(body[j])||body[j]=='.')) j++;
      cls=c; mag=body.substring(i+1,j).toFloat(); return true; } }
  return false;
}
int penaltyFor(char cls,float mag){ if(cls=='C')return 1; if(cls=='M')return (mag>=5.0f)?4:3; if(cls=='X')return (mag>=2.0f)?6:5; return 0; }
String extractPeakTime(const String& body){
  int p=body.indexOf("peak"); if(p<0) p=body.indexOf("max"); if(p<0) return "";
  int q=body.indexOf('"',p); if(q<0) return ""; int s=body.indexOf('"',q+1); if(s<0) return ""; int e=body.indexOf('"',s+1); if(e<0) return ""; return body.substring(s+1,e);
}
void fetchAndApplyFlarePenalty(){
  if(millis()-FL_fetched_ms<5UL*60UL*1000UL) return; FL_fetched_ms=millis();
  FL_class=""; FL_peak=""; FL_penalty=0; if(WiFi.status()!=WL_CONNECTED) return;
  String body; if(!httpsGET("services.swpc.noaa.gov","/json/goes/primary/xray-flares-latest.json",body)) return;
  char cls='?'; float mag=0;
  if(parseFlareClass(body,cls,mag)){
    FL_class=String(cls)+String(mag,1); FL_peak=extractPeakTime(body); FL_penalty=penaltyFor(cls,mag);
    if(g_flare_dayOnly){ updateDaylight(); if(!g_is_daylight) return; }
    if(H_score10>0){
      int pen=scaleDeltaFrom10(FL_penalty);
      int minFloor=(g_score_max==10?1:0);
      H_score10=max(minFloor,H_score10-pen);
      H_label10=labelForHF10(scoreForLabels10(H_score10));
      SOL_applied_delta10=-pen;
    }
  }
}

// -------- Solar (F10.7/Kp) -----------
bool parseF107(const String& body,float& out){
  int p=body.lastIndexOf("observed_flux"); if(p<0) p=body.lastIndexOf("\"flux\""); if(p<0) return false;
  int c=body.indexOf(':',p); if(c<0) return false; int i=c+1; while(i<(int)body.length()&&(body[i]==' '||body[i]=='\t')) i++;
  int j=i; while(j<(int)body.length()&&(isDigit(body[j])||body[j]=='.')) j++; if(j==i) return false;
  out=body.substring(i,j).toFloat(); return (out>0);
}
bool parseKpLatest(const String& body,float& kp,String& ts){
  int lb=body.lastIndexOf('['); if(lb<0) return false; int rb=body.indexOf(']',lb); if(rb<0) return false;
  String row=body.substring(lb+1,rb); int comma=row.lastIndexOf(','); if(comma<0) return false;
  String kpstr=row.substring(comma+1); kpstr.trim(); kp=kpstr.toFloat();
  String tstr=row.substring(0,comma); tstr.trim(); if(tstr.startsWith("\"")) tstr.remove(0,1); if(tstr.endsWith("\"")) tstr.remove(tstr.length()-1);
  ts=tstr; return (kp>0);
}
void computeSolarScore(){
  float S_f=isnan(SOL_f107)?NAN:clamp01((SOL_f107-70.0f)/130.0f);
  float S_k=isnan(SOL_kp)?NAN:clamp01(1.0f-(SOL_kp-1.0f)/6.0f);
  const float wF=0.7f,wK=0.3f; float S=0,W=0;
  if(!isnan(S_f)){S+=wF*S_f;W+=wF;} if(!isnan(S_k)){S+=wK*S_k;W+=wK;}
  S=(W>0)?(S/W):0.0f; SOL_score=scoreFromSScaled(S); SOL_label=labelForSolar(scoreForLabels10(SOL_score));
}
void fetchSolarActivityAndMaybeAdjust10m(){
  if(millis()-SOL_fetched_ms<10UL*60UL*1000UL) return; SOL_fetched_ms=millis();
  int deltaShown=0; if(WiFi.status()!=WL_CONNECTED){ SOL_applied_delta10=0; return; }
  String b1; if(httpsGET("services.swpc.noaa.gov","/json/solar-cycle/f10-7cm-flux.json",b1,8192)){ float f=NAN; if(parseF107(b1,f)) SOL_f107=f; }
  String b2,ts; if(httpsGET("services.swpc.noaa.gov","/products/noaa-planetary-k-index.json",b2,8192)){ float k=NAN; if(parseKpLatest(b2,k,ts)) SOL_kp=k; }
  computeSolarScore();
  if(g_solar_adjust10 && H_score10>0){
    int sol_s10=scoreForLabels10(SOL_score); int d10=0;
    if(sol_s10>=9) d10=+2; else if(sol_s10>=7) d10=+1; else if(sol_s10<=3) d10=-1;
    if(!isnan(SOL_kp) && SOL_kp>5.0f) d10-=1;
    int dScaled=scaleDeltaFrom10(d10);
    if(dScaled!=0){ int minFloor=(g_score_max==10?1:0); H_score10=constrain(H_score10+dScaled,minFloor,g_score_max); H_label10=labelForHF10(scoreForLabels10(H_score10)); deltaShown=dScaled; }
  }
  SOL_applied_delta10=deltaShown;
}

// -------- Störungen (%) --------------
void computeDisturbanceProbability(){
  float rk=isnan(SOL_kp)?0.15f:clamp01((SOL_kp-2.0f)/5.0f); // Kp 2->0, 7->1
  float rf=0.0f;
  if(FL_penalty>0){
    float pen=clamp01(FL_penalty/6.0f);
    rf = g_flare_dayOnly && !g_is_daylight ? 0.0f : (g_is_daylight ? pen : 0.3f*pen);
  }
  float risk=clamp01(0.7f*rk+0.3f*rf);
  DIST_prob_pct=(int)round(risk*100.0f);
  if(risk>=0.7f) DIST_label="hoch"; else if(risk>=0.35f) DIST_label="mittel"; else DIST_label="niedrig";
}

// -------- 10 m Bewertung -------------
void computeHF10m(){
  FLAG_HF10_FALLBACK_USED=false;
  bool haveSpots=!isnan(H_SPOTS15), haveDx=!isnan(H_DXKM), haveSnr=!isnan(H_SNRDB);
  if(haveSpots||haveDx||haveSnr){
    float S_spots=haveSpots?clamp01(H_SPOTS15/30.0f):NAN;
    float S_dx   =haveDx?clamp01((H_DXKM-800.0f)/2200.0f):NAN;
    float S_snr  =haveSnr?clamp01((H_SNRDB+20.0f)/25.0f):NAN;
    float w1=0.30f,w2=0.50f,w3=0.20f,S=0,W=0;
    if(!isnan(S_spots)){S+=w1*S_spots;W+=w1;}
    if(!isnan(S_dx))   {S+=w2*S_dx;   W+=w2;}
    if(!isnan(S_snr))  {S+=w3*S_snr;  W+=w3;}
    S=(W>0)?(S/W):0.0f; H_score10=scoreFromSScaled(S); H_label10=labelForHF10(scoreForLabels10(H_score10)); return;
  }
  float Sf=isnan(SOL_f107)?NAN:clamp01((SOL_f107-80.0f)/90.0f);
  float Sk=isnan(SOL_kp)?NAN:clamp01(1.0f-(SOL_kp-1.0f)/5.0f);
  float St=dayTimeFactor(), Ss=seasonEquinoxFactor();
  float S=0,W=0; if(!isnan(Sf)){S+=0.40f*Sf;W+=0.40f;} if(!isnan(Sk)){S+=0.20f*Sk;W+=0.20f;} S+=0.25f*St;W+=0.25f; S+=0.15f*Ss;W+=0.15f;
  S=(W>0)?(S/W):0.0f; H_score10=scoreFromSScaled(S); H_label10=labelForHF10(scoreForLabels10(H_score10)); FLAG_HF10_FALLBACK_USED=true;
}

// -------- HTTP helpers ----------------
void sendResponse(WiFiClient &c,int code,const char* status,const char* type,const String& body){
  c.setNoDelay(true);
  c.println(String("HTTP/1.1 ")+code+" "+status);
  c.print("Content-Type: "); c.println(type);
  c.print("Content-Length: "); c.println(body.length());
  c.println("Connection: close"); c.println(); c.print(body);
  c.flush(); delay(15); c.stop();
}
String pathOnly(const String& req){ int a=req.indexOf(' '); if(a<0) return "/"; int b=req.indexOf(' ',a+1); if(b<0) return "/"; return req.substring(a+1,b); }
String getQuery(const String& path){ int q=path.indexOf('?'); return (q<0)?String(""):path.substring(q+1); }
String getParam(const String& query,const String& key){ String k=key+"="; int p=query.indexOf(k); if(p<0) return ""; int s=p+k.length(); int e=query.indexOf('&',s); String v=(e<0)?query.substring(s):query.substring(s,e); v.replace('+',' '); return v; }
float toFloatOrNaN(const String& s){ if(!s.length()) return NAN; char* endp=nullptr; float v=strtof(s.c_str(),&endp); if(endp==s.c_str()) return NAN; return v; }

// -------- Web Seiten ------------------
String checkboxRow(const char* label,const char* name,bool checked,const char* hint){
  String s="<tr><td>"+String(label)+"</td><td><input type='checkbox' name='"+String(name)+"' value='1'"; if(checked) s+=" checked"; s+="></td><td>"+String(hint)+"</td></tr>"; return s; }
String inputRow(const char* label,const char* name,float v,const char* unit){
  String s="<tr><td>"+String(label)+"</td><td><input name='"+String(name)+"' value='"; if(!isnan(v)) s+=String(v,2); s+="'></td><td>"+String(unit)+"</td></tr>"; return s; }
String inputRowInt(const char* label,const char* name, int v,const char* unit){
  String s="<tr><td>"+String(label)+"</td><td><input name='"+String(name)+"' value='"+String(v)+"'></td><td>"+String(unit)+"</td></tr>"; return s; }
String legendTropo(){ return "<details><summary>Was bedeutet „Tropo m&ouml;glich“?</summary><div style='padding:8px 0'>dN/dz &lt; −157&nbsp;N/km =&nbsp;Ducting; −79..−157 =&nbsp;erh&ouml;hte Reichweite.</div></details>"; }
String legend10m(){ return "<details><summary>Wie wird 10&nbsp;m bewertet?</summary><div style='padding:8px 0'>On-Air-Proxy; sonst Fallback aus F10.7/Kp/Tag/Saison.</div></details>"; }

String pageLocal(const String& fullPath){
  String q=getQuery(fullPath);
  if(q.length()){
    // Eingaben
    L_P1=toFloatOrNaN(getParam(q,"P1")); L_T1=toFloatOrNaN(getParam(q,"T1")); L_RH1=toFloatOrNaN(getParam(q,"RH1")); L_z1=toFloatOrNaN(getParam(q,"z1"));
    L_P2=toFloatOrNaN(getParam(q,"P2")); L_T2=toFloatOrNaN(getParam(q,"T2")); L_RH2=toFloatOrNaN(getParam(q,"RH2")); L_z2=toFloatOrNaN(getParam(q,"z2"));
    H_SPOTS15=toFloatOrNaN(getParam(q,"SPOTS15")); H_DXKM=toFloatOrNaN(getParam(q,"DXKM")); H_SNRDB=toFloatOrNaN(getParam(q,"SNRDB"));
    // Schalter
    g_flare_enabled =(getParam(q,"FLARE")=="1");
    g_flare_dayOnly =(getParam(q,"DAYONLY")=="1");
    g_solar_enabled =(getParam(q,"SOLAR")=="1");
    g_solar_adjust10=(getParam(q,"SOLAR_ADJ10")=="1");
    g_splash_enabled=(getParam(q,"SPLASH")=="1");
    g_vhf_heuristic_enable=(getParam(q,"VHF_HEUR")=="1");
    // Heute-Override
    g_today_override_vhf=(getParam(q,"OV_VHF")=="1"); String ovs=getParam(q,"OV_VHF_SCORE"); if(ovs.length()){ int s=ovs.toInt(); g_today_score_vhf10=constrain(s,1,10); }
    // Skala
    String sc=getParam(q,"SCALE"); if(sc=="5") g_score_max=5; else if(sc=="10") g_score_max=10;
    // OLED Kontrast
    String cstr=getParam(q,"CONTRAST"); if(cstr.length()){ int c=cstr.toInt(); OLED_CONTRAST=constrain(c,0,255); u8g2.setContrast(OLED_CONTRAST); }
    // Remote
    g_remote_enabled=(getParam(q,"REMOTE")=="1"); String rh=getParam(q,"REMOTE_HOST"); if(rh.length()) g_remote_host=rh;
    // Overlay
    g_overlay_enabled=(getParam(q,"OVERLAY")=="1");
    String per=getParam(q,"OVERLAY_PERIOD_S"); if(per.length()){ int ps=per.toInt(); ps=constrain(ps,3,600); OVERLAY_PERIOD_MS=(uint32_t)ps*1000UL; }
    String dur=getParam(q,"OVERLAY_DUR_MS"); if(dur.length()){ int dm=dur.toInt(); dm=constrain(dm,500,15000); OVERLAY_DURATION_MS=(uint32_t)dm; }
    if(OVERLAY_DURATION_MS>OVERLAY_PERIOD_MS-250) OVERLAY_DURATION_MS=OVERLAY_PERIOD_MS-250;

    saveState();

    // Overlay-Scheduler sofort neu setzen
    if(g_overlay_enabled){
      OLED_OVERLAY=false;
      OLED_OVERLAY_NEXT = millis() + OVERLAY_PERIOD_MS;
    } else {
      OLED_OVERLAY=false;
      OLED_OVERLAY_NEXT = 0;
    }
  }

  // Rechnen/Aktualisieren
  computeLocalTroposphere();
  computeHF10m();
  updateDaylight();
  computeVHFHeuristicIfSingleHeight();
  applyTodayOverrideVHF();
  if(g_flare_enabled) fetchAndApplyFlarePenalty();
  if(g_solar_enabled) fetchSolarActivityAndMaybeAdjust10m();
  computeDisturbanceProbability();

  String h=F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
             "<title>Funkwetter – Lokal</title>"
             "<style>body{font-family:system-ui,Arial,sans-serif;margin:16px}a{color:#06c;text-decoration:none}nav a{margin-right:12px}"
             "table{border-collapse:collapse;width:100%;max-width:1000px}td,th{border:1px solid #ccc;padding:6px 8px}th{background:#f5f5f5}"
             ".ampel{margin:16px 0;padding:12px;border-radius:8px;background:#f8f9fb}input{width:100%;max-width:160px;padding:4px}"
             "button{padding:8px 12px;margin:4px}</style></head><body>");
  h+="<nav><a href='/local'>Lokal</a> <a href='/json'>JSON</a> <a href='/reset'>Reset</a> | mDNS: <b>http://"+String(HOSTNAME)+".local/</b></nav>";
  h+="<h1>Lokale Berechnung</h1>";
  h+="<p><b>Zeit:</b> "+g_time_str+" &nbsp; | &nbsp; <b>Aufgang:</b> "+g_rise_str+" &nbsp; <b>Untergang:</b> "+g_set_str+
     " &nbsp; | &nbsp; <b>Tag/Nacht:</b> "+String(g_is_daylight? "Tag":"Nacht")+
     " &nbsp; | &nbsp; <b>Skala:</b> "+String(g_score_max==10? "Fein (1–10)":"Grob (0–5)")+"</p>";

  h+="<form method='GET' action='/local'><table>";
  h+="<tr><th colspan='3'>H&ouml;he 1 (2 m/70 cm)</th></tr>";
  h+=inputRow("Druck P1","P1",L_P1,"hPa"); h+=inputRow("Temp T1","T1",L_T1,"°C"); h+=inputRow("Feuchte RH1","RH1",L_RH1,"%"); h+=inputRow("H&ouml;he z1","z1",L_z1,"m");
  h+="<tr><th colspan='3'>H&ouml;he 2</th></tr>";
  h+=inputRow("Druck P2","P2",L_P2,"hPa"); h+=inputRow("Temp T2","T2",L_T2,"°C"); h+=inputRow("Feuchte RH2","RH2",L_RH2,"%"); h+=inputRow("H&ouml;he z2","z2",L_z2,"m");

  h+="<tr><th colspan='3'>On-Air 10&nbsp;m (optional)</th></tr>";
  h+=inputRow("Spots/15&nbsp;min","SPOTS15",H_SPOTS15,"Anzahl"); h+=inputRow("max. Distanz","DXKM",H_DXKM,"km"); h+=inputRow("Median-SNR","SNRDB",H_SNRDB,"dB");

  h+="<tr><th colspan='3'>Heuristiken</th></tr>";
  h+=checkboxRow("VHF-Heuristik bei nur 1 H&ouml;he","VHF_HEUR",g_vhf_heuristic_enable,"Nutzt Druck/Nacht/RH als Proxy");

  h+="<tr><th colspan='3'>Heute-Override (VHF)</th></tr>";
  h+="<tr><td>Aktiv</td><td><input type='checkbox' name='OV_VHF' value='1'"+String(g_today_override_vhf?" checked":"")+"></td><td>Wenn heute klar besser (Tropo etc.).</td></tr>";
  h+="<tr><td>Stufe (1–10)</td><td><input name='OV_VHF_SCORE' value='"+String(g_today_score_vhf10)+"'></td><td>z.&nbsp;B. 8</td></tr>";

  h+="<tr><th colspan='3'>Solarflare</th></tr>";
  h+=checkboxRow("Flares ber&uuml;cksichtigen","FLARE",g_flare_enabled,"GOES X-Ray – 10 m D&auml;mpfung v. a. tags");

  h+="<tr><th colspan='3'>Sonnenaktivit&auml;t (API)</th></tr>";
  h+=checkboxRow("F10.7 &amp; Kp ber&uuml;cksichtigen","SOLAR",g_solar_enabled,"SWPC – 10 min Throttle");
  h+=checkboxRow("10 m mit Solar justieren","SOLAR_ADJ10",g_solar_adjust10,"+2/+1/0/−1; Kp>5: −1");

  h+="<tr><th colspan='3'>Punkte-Skala</th></tr>";
  h+="<tr><td>Anzeige</td><td colspan='2'>"
     "<label><input type='radio' name='SCALE' value='10' "+String(g_score_max==10?"checked":"")+"> Fein (1–10)</label>&nbsp;&nbsp;"
     "<label><input type='radio' name='SCALE' value='5'  "+String(g_score_max==5 ?"checked":"")+"> Grob (0–5)</label>"
     "</td></tr>";

  h+="<tr><th colspan='3'>Startup</th></tr>";
  h+=checkboxRow("Splash-Animation (~8s)","SPLASH",g_splash_enabled,"Beim Start anzeigen");

  h+="<tr><th colspan='3'>OLED</th></tr>";
  h+="<tr><td>Kontrast</td><td><input type='range' min='0' max='255' name='CONTRAST' value='"+String(OLED_CONTRAST)+"'></td><td>"+String(OLED_CONTRAST)+"</td></tr>";

  // Overlay-Settings
  h+="<tr><th colspan='3'>Overlay / RAW-Sensordaten</th></tr>";
  h+=checkboxRow("Overlay aktivieren","OVERLAY",g_overlay_enabled,"Zeigt alle X s f&uuml;r Y ms den Sensor-Screen");
  h+=inputRowInt("Intervall","OVERLAY_PERIOD_S",(int)(OVERLAY_PERIOD_MS/1000UL),"Sekunden (3–600)");
  h+=inputRowInt("Dauer","OVERLAY_DUR_MS",(int)OVERLAY_DURATION_MS,"ms (500–15000)");

  h+="<tr><th colspan='3'>Remote Sensor (ESP BMP280+DHT22)</th></tr>";
  h+=checkboxRow("Remote aktivieren","REMOTE",g_remote_enabled,"Zieht P/T/RH/z von http://HOST/sensors.json");
  h+="<tr><td>Host/IP</td><td><input name='REMOTE_HOST' value='"+g_remote_host+"'></td><td>z.&nbsp;B. funk-remote.local oder 192.168.1.75</td></tr>";

  h+="</table><p><button type='submit'>Berechnen / Anwenden</button></p></form>";

  // Ergebnisse
  h+="<div class='ampel'><h3>Band-Bewertung</h3><ul>";
  auto fmtScore=[&](int s){ return String(s)+"/"+String(g_score_max); };

  if(!isnan(L_dNdz) || FLAG_VHF_HEUR_USED){
    String src=FLAG_VHF_HEUR_USED?" (Heuristik)":"";
    h+="<li><b>2 m:</b> Stufe "+fmtScore(L_score2m)+" – "+L_score2m_lbl+src+"</li>";
    h+="<li><b>70 cm:</b> Stufe "+fmtScore(L_score70)+" – "+L_score70_lbl+src+"</li>";
  } else {
    h+="<li><b>2 m:</b> unbekannt</li><li><b>70 cm:</b> unbekannt</li>";
  }

  h+="<li><b>10 m:</b> Stufe "+fmtScore(H_score10)+" – "+H_label10+(FLAG_HF10_FALLBACK_USED?" (Solar/Heuristik)":"")+"</li>";

  if(g_solar_enabled && SOL_score>0){
    h+="<li><b>Sonnenaktivit&auml;t:</b> Stufe "+fmtScore(SOL_score)+" – "+SOL_label+
       " (F10.7="+f2(SOL_f107,0)+" sfu, Kp="+f2(SOL_kp,1)+")";
    if(SOL_applied_delta10!=0) h+=" &nbsp; d10m: "+String(SOL_applied_delta10);
    h+="</li>";
  }
  h+="<li><b>Atmosph&auml;rische St&ouml;rungen:</b> "+String(DIST_prob_pct)+"% – "+DIST_label+"</li>";

  h+="<p><b>Overlay:</b> "+String(g_overlay_enabled ? "aktiv" : "aus")+ " &nbsp; ("+String(OVERLAY_PERIOD_MS/1000)+"s / "+String(OVERLAY_DURATION_MS)+"ms)</p>";
  h+="<p><b>Remote:</b> "+String(g_remote_enabled ? (REMOTE_OK ? "OK" : "Fehler ("+REMOTE_ERR+")") : "aus")+"</p>";
  h+="</ul></div>";

  if(!isnan(L_dNdz)){
    auto f1=[&](float v,int d){ return isnan(v)?String("—"):String(v,d); };
    h+="<h3>Berechnete Gr&ouml;&szlig;en</h3><table>";
    h+="<tr><th>Gr&ouml;&szlig;e</th><th>Wert</th><th>Einheit</th></tr>";
    h+="<tr><td>N1</td><td>"+f1(L_N1,1)+"</td><td>N-units</td></tr>";
    h+="<tr><td>N2</td><td>"+f1(L_N2,1)+"</td><td>N-units</td></tr>";
    h+="<tr><td>dN/dz</td><td>"+f1(L_dNdz,0)+"</td><td>N/km</td></tr>";
    h+="<tr><td>k-Faktor</td><td>"+f1(L_k,2)+"</td><td>—</td></tr>";
    h+="</table>";
  }
  h+=legendTropo(); h+=legend10m();
  h+="</body></html>";
  return h;
}

String jsonOut(){
  computeLocalTroposphere(); computeHF10m(); updateDaylight();
  computeVHFHeuristicIfSingleHeight(); applyTodayOverrideVHF();
  if(g_flare_enabled) fetchAndApplyFlarePenalty();
  if(g_solar_enabled) fetchSolarActivityAndMaybeAdjust10m();
  computeDisturbanceProbability();

  String s="{"; auto num=[&](const char* k,float v,int d){ s+="\""; s+=k; s+="\":"; if(isnan(v)) s+="null"; else s+=String(v,d); s+=","; };
  s+="\"score_max\":"+String(g_score_max)+",";
  s+="\"flags\":{\"vhf_heur\":"+(FLAG_VHF_HEUR_USED?String("true"):String("false"))+",\"hf10_fallback\":"+(FLAG_HF10_FALLBACK_USED?String("true"):String("false"))+"},";
  s+="\"scores\":{\"2m\":"+String(L_score2m)+",\"70cm\":"+String(L_score70)+",\"10m\":"+String(H_score10)+",\"solar\":"+String(SOL_score)+"},";
  s+="\"labels\":{\"2m\":\""+L_score2m_lbl+"\",\"70cm\":\""+L_score70_lbl+"\",\"10m\":\""+H_label10+"\",\"solar\":\""+SOL_label+"\"},";
  s+="\"flare\":{\"enabled\":"+(g_flare_enabled?String("true"):String("false"))+",\"dayonly\":"+(g_flare_dayOnly?String("true"):String("false"))+
     ",\"class\":\""+FL_class+"\",\"peak\":\""+FL_peak+"\",\"penalty10\":"+String(FL_penalty)+"},";
  s+="\"solar\":{\"enabled\":"+(g_solar_enabled?String("true"):String("false"))+",\"adjust10\":"+(g_solar_adjust10?String("true"):String("false"))+
     ",\"f107\":"+(isnan(SOL_f107)?String("null"):String(SOL_f107,0))+",\"kp\":"+(isnan(SOL_kp)?String("null"):String(SOL_kp,1))+
     ",\"score\":"+String(SOL_score)+",\"delta_scaled\":"+String(SOL_applied_delta10)+"},";
  s+="\"disturbance\":{\"prob_pct\":"+String(DIST_prob_pct)+",\"label\":\""+DIST_label+"\"},";
  s+="\"overlay\":{\"enabled\":"+(g_overlay_enabled?String("true"):String("false"))+
     ",\"period_s\":"+String(OVERLAY_PERIOD_MS/1000)+",\"duration_ms\":"+String(OVERLAY_DURATION_MS)+"},";
  s+="\"remote\":{\"enabled\":"+(g_remote_enabled?String("true"):String("false"))+",\"ok\":"+(REMOTE_OK?String("true"):String("false"))+
     ",\"host\":\""+g_remote_host+"\",\"last_err\":\""+REMOTE_ERR+"\"},";
  s+="\"time\":{\"now\":\""+g_time_str+"\",\"sunrise\":\""+g_rise_str+"\",\"sunset\":\""+g_set_str+"\",\"daylight\":"+(g_is_daylight?String("true"):String("false"))+"},";
  num("N1",L_N1,1); num("N2",L_N2,1); num("dN_dz",L_dNdz,0); num("k",L_k,2);
  num("spots15",H_SPOTS15,0); num("dx_km",H_DXKM,0); num("snr_db",H_SNRDB,1);
  num("P1",L_P1,1); num("T1",L_T1,1); num("RH1",L_RH1,0); num("z1",L_z1,0);
  num("P2",L_P2,1); num("T2",L_T2,1); num("RH2",L_RH2,0); num("z2",L_z2,0);
  if (s[s.length()-1]==',') s.remove(s.length()-1); s+="}"; return s;
}

// -------- Remote JSON parse ----------
bool parseRemoteJSON(const String& b,float& P,float& T,float& RH,float& Z){
  auto findNum=[&](const char* key,float& out)->bool{
    int k=b.indexOf(String("\"")+key+"\""); if(k<0) return false; int c=b.indexOf(':',k); if(c<0) return false;
    int i=c+1; while(i<(int)b.length()&&(b[i]==' '||b[i]=='\t')) i++; int j=i;
    while(j<(int)b.length()&&(isDigit(b[j])||b[j]=='.'||b[j]=='-')) j++; if(j==i) return false; out=b.substring(i,j).toFloat(); return true; };
  bool ok=false; float p=NAN,t=NAN,rh=NAN,z=NAN;
  if(findNum("P_hPa",p)) ok=true; if(findNum("T_C",t)) ok=true; if(findNum("RH_pct",rh)) ok=true; if(findNum("z_m",z)) ok=true;
  P=p; T=t; RH=rh; Z=z; return ok;
}
bool fetchRemoteSensorOnce(String& err){
  err=""; if(!g_remote_enabled||g_remote_host.length()==0){ err="disabled"; return false; }
  if(WiFi.status()!=WL_CONNECTED){ err="wifi"; return false; }
  String body; if(!httpGET(g_remote_host,"/sensors.json",body)){ err="http"; return false; }
  float p=NAN,t=NAN,rh=NAN,z=NAN; if(!parseRemoteJSON(body,p,t,rh,z)){ err="parse"; return false; }
  R_P_hPa=p; R_T_C=t; R_RH_pct=rh; R_z_m=z;
  if(!isnan(R_P_hPa)) L_P1=R_P_hPa; if(!isnan(R_T_C)) L_T1=R_T_C; if(!isnan(R_RH_pct)) L_RH1=R_RH_pct; if(!isnan(R_z_m)) L_z1=R_z_m;
  return true;
}
void pollRemoteSensor(){
  if(!g_remote_enabled) { REMOTE_OK=false; return; }
  if(millis()-REMOTE_LAST_MS<60000) return;
  REMOTE_LAST_MS=millis();
  String err;
  for(int i=0;i<2;i++){
    if(fetchRemoteSensorOnce(err)){ REMOTE_OK=true; REMOTE_FAILS=0; REMOTE_ERR=""; computeLocalTroposphere(); computeVHFHeuristicIfSingleHeight(); return; }
    delay(200);
  }
  REMOTE_OK=false; REMOTE_FAILS++; REMOTE_ERR=err;
}

// -------- OLED helpers ----------------
void approach(float &cur,float target,float alpha=0.25f){ cur += (target-cur)*alpha; if(fabsf(target-cur)<0.05f) cur=target; }
void drawBarBasic(int x,int y,int w,int h,float val01){ u8g2.drawFrame(x,y,w,h); int fill=(int)round((w-2)*val01); fill=constrain(fill,0,w-2); if(fill>0) u8g2.drawBox(x+1,y+1,fill,h-2); }
void drawBarSolarAnim(int x,int y,int w,int h,float val01,int scoreDisp){
  // Grundbalken
  u8g2.drawFrame(x,y,w,h);
  int fill = (int)round((w-2)*clamp01(val01));
  fill = constrain(fill, 0, w-2);
  if(fill <= 0){
    return;
  }
  // Füllung (fix, ohne XOR)
  u8g2.setDrawColor(1);
  u8g2.drawBox(x+1, y+1, fill, h-2);

  // Sanfte Shine-Animation: 1px „Gap“ + 2px helles Band, ohne XOR
  if(fill > 6 && scoreDisp > 0){
    // Takt auf ~40 ms drosseln (ca. 25 fps)
    if(millis() - SOL_ANIM_MS > 40){
      SOL_ANIM_MS = millis();
      SOL_ANIM_PHASE++;
    }
    int span = fill - 4;
    if(span < 1) span = 1;
    int pos = SOL_ANIM_PHASE % span;

    // 1px „Schatten“-Lücke
    u8g2.setDrawColor(0);
    u8g2.drawBox(x+1+pos, y+1, 1, h-2);

    // 2px „Glanz“-Streifen
    u8g2.setDrawColor(1);
    u8g2.drawBox(x+2+pos, y+1, 2, h-2);
  }

  // Sicherheit
  u8g2.setDrawColor(1);
}
void drawWifiIcon(int x,int y,bool ok){ if(!ok){ u8g2.drawStr(x-6,y,"x"); return; } u8g2.drawPixel(x,y); u8g2.drawCircle(x,y,2); u8g2.drawCircle(x,y,4); }
void drawSunMoon(int x,int y,bool day){ if(day){ u8g2.drawCircle(x,y,4); u8g2.drawDisc(x,y,2); } else { u8g2.drawDisc(x,y,4); u8g2.setDrawColor(0); u8g2.drawDisc(x+2,y-1,4); u8g2.setDrawColor(1); } }
void drawFlareIcon(int x,int y,bool active){ if(!active) return; u8g2.drawTriangle(x,y-4,x-3,y+2,x+3,y+2); u8g2.drawLine(x,y+2,x,y+5); }

// -------- Ticker (ASCII safe) ----------
String buildTicker(){
  auto fmtScore=[&](int s){ return String(s)+"/"+String(g_score_max); };
  auto fmtF = [&](float v, int d){ return isnan(v) ? String("--") : String(v, d); };
  auto fmtI = [&](float v){ return isnan(v) ? String("--") : String((int)round(v)); };

  String s;

  // 1) Band-Scores
  s += "2m "+fmtScore(L_score2m)+" ("+L_score2m_lbl+")  ";
  s += "70cm "+fmtScore(L_score70)+" ("+L_score70_lbl+")  ";
  s += "10m "+fmtScore(H_score10)+" ("+H_label10+")";

  // 2) Solar + Störungen
  s += "  |  Solar: "+fmtScore(SOL_score)+" ("+SOL_label+") [F10.7 "+fmtF(SOL_f107,0)+" sfu, Kp "+fmtF(SOL_kp,1)+"]";
  if (SOL_applied_delta10 != 0){
    int d = SOL_applied_delta10;
    s += "  10m-adj " + String((d>0?"+":"")) + String(d);
  }
  if (FL_class.length()) s += "  |  Flare "+FL_class;
  s += "  |  Stoerungen: " + String(DIST_prob_pct) + "%";

  // 3) Messwerte Höhe 1 / Höhe 2
  bool haveH1 = !(isnan(L_P1)&&isnan(L_T1)&&isnan(L_RH1)&&isnan(L_z1));
  bool haveH2 = !(isnan(L_P2)&&isnan(L_T2)&&isnan(L_RH2)&&isnan(L_z2));
  if (haveH1 || haveH2){
    s += "  |  H1: P " + fmtF(L_P1,1) + " hPa, T " + fmtF(L_T1,1) + " C, rF " + fmtI(L_RH1) + " %, z " + fmtI(L_z1) + " m";
    if (haveH2){
      s += "  |  H2: P " + fmtF(L_P2,1) + " hPa, T " + fmtF(L_T2,1) + " C, rF " + fmtI(L_RH2) + " %, z " + fmtI(L_z2) + " m";
    }
  }

  // 4) Berechnete Größen
  if (!isnan(L_dNdz) || !isnan(L_k) || !isnan(L_N1) || !isnan(L_N2)){
    s += "  |  N1 " + fmtF(L_N1,1) + ", N2 " + fmtF(L_N2,1) + ", dN/dz " + fmtF(L_dNdz,0) + " N/km, k " + fmtF(L_k,2);
  } else if (FLAG_VHF_HEUR_USED){
    s += "  |  VHF Heuristik aktiv";
  }

  // 5) 10m On-Air Proxy
  if (!isnan(H_SPOTS15) || !isnan(H_DXKM) || !isnan(H_SNRDB)){
    s += "  |  10m proxy: spots/15 " + fmtI(H_SPOTS15) + ", dx " + fmtI(H_DXKM) + " km, snr " + fmtF(H_SNRDB,1) + " dB";
  } else if (FLAG_HF10_FALLBACK_USED){
    s += "  |  10m via Solar/Time/Season";
  }

  // 6) Remote-Status
  s += "  |  Rem:"; s += g_remote_enabled ? (REMOTE_OK?"OK":"ERR") : "OFF";
  if (g_remote_enabled){
    s += " (" + g_remote_host + ")";
    if (REMOTE_OK){
      s += " P " + fmtF(R_P_hPa,1) + " hPa, T " + fmtF(R_T_C,1) + " C, rF " + fmtI(R_RH_pct) + " %, z " + fmtI(R_z_m) + " m";
    } else if (REMOTE_ERR.length()){
      s += " ["+REMOTE_ERR+"]";
    }
  }

  return ascii_de(s);
}

// ---- Gemeinsamer Ticker-Update & -Draw ----
void updateTicker(){
  if(millis() - OLED_TICKER_MS > 2000){
    OLED_TICKER_MS = millis();
    OLED_TICKER = buildTicker();
    int w = u8g2.getStrWidth(OLED_TICKER.c_str());
    if (OLED_TICKER_X < -w) OLED_TICKER_X = 128;
  }
  static uint8_t tickDiv = 0;
  if(++tickDiv >= 2){
    OLED_TICKER_X -= 1;
    tickDiv = 0;
  }
}
void drawTickerAtBottom(){
  u8g2.setFont(u8g2_font_6x10_tf);
  updateTicker();
  u8g2.drawStr(OLED_TICKER_X, 64, OLED_TICKER.c_str());
}

// ---- Overlay: Rohdaten/Infos (Ticker unten aktiv) ----
void oledRenderOverlay(){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0,10,"RAW / Sensors");
  // Uhrzeit rechts
  String t=(g_time_str.length()>=16)?g_time_str.substring(11,16):String("--:--");
  int t_w=u8g2.getStrWidth(t.c_str()); 
  u8g2.drawStr(128-t_w,10,t.c_str());

  // Inhalt so layouten, dass die letzte Zeile bei y<=54 bleibt (unten 10px für Ticker)
  u8g2.setFont(u8g2_font_5x8_tf);
  int y=18;
  auto line=[&](String s){
    if (y <= 54) { u8g2.drawStr(0,y, ascii_de(s).c_str()); }
    y += 9;
  };

  // H1 / H2
  line("H1: P "+f2(L_P1,1)+" hPa, T "+f2(L_T1,1)+" C, rF "+f2(L_RH1,0)+" %, z "+f2(L_z1,0)+" m");
  line("H2: P "+f2(L_P2,1)+" hPa, T "+f2(L_T2,1)+" C, rF "+f2(L_RH2,0)+" %, z "+f2(L_z2,0)+" m");

  // berechnete Größen
  line("N1 "+f2(L_N1,1)+", N2 "+f2(L_N2,1)+", dN/dz "+f2(L_dNdz,0)+" N/km, k "+f2(L_k,2));

  // Solar & Disturbances
  line("Solar F10.7 "+f2(SOL_f107,0)+" sfu, Kp "+f2(SOL_kp,1)+", Stoer "+String(DIST_prob_pct)+"%");

  // 10m Proxy
  line("10m Proxy: spots15 "+f2(H_SPOTS15,0)+", dx "+f2(H_DXKM,0)+" km, snr "+f2(H_SNRDB,1)+" dB");

  // Remote
  {
    String rs="Remote: "; rs += g_remote_enabled ? (REMOTE_OK?"OK ":"ERR ") : "OFF ";
    rs += "host="+g_remote_host;
    if(g_remote_enabled && REMOTE_OK){
      rs += " | P "+f2(R_P_hPa,1)+", T "+f2(R_T_C,1)+", rF "+f2(R_RH_pct,0)+", z "+f2(R_z_m,0);
    } else if (g_remote_enabled && !REMOTE_OK && REMOTE_ERR.length()){
      rs += " ["+REMOTE_ERR+"]";
    }
    line(rs);
  }

  // --- Ticker unten auch im Overlay ---
  drawTickerAtBottom();

  u8g2.sendBuffer();
}

// -------- OLED Render (Normal) --------
void oledRender(){
  computeVHFHeuristicIfSingleHeight(); applyTodayOverrideVHF();

  // Overlay aktiv?
  if (OLED_OVERLAY){
    oledRenderOverlay();
    return;
  }

  float t2m=(L_score2m>0)?L_score2m:0, t70=(L_score70>0)?L_score70:0, t10=(H_score10>0)?H_score10:0, tSolar=(SOL_score>0)?SOL_score:0;
  if(millis()-lastDispMs>100){ lastDispMs=millis(); approach(disp2m,t2m); approach(disp70,t70); approach(disp10,t10); approach(dispSolar,tSolar); }

  const int X=18, W=78, H=5, Y0=23, DY=9;
  u8g2.clearBuffer();

  // Header
  u8g2.setFont(u8g2_font_6x10_tf);
  const char* title="Funkwetter"; u8g2.drawStr(0,10,title);
  int title_w=u8g2.getStrWidth(title), y_center=8;
  drawWifiIcon(title_w+6,y_center,WiFi.status()==WL_CONNECTED);
  drawSunMoon(title_w+22,y_center,g_is_daylight);
  String t=(g_time_str.length()>=16)?g_time_str.substring(11,16):String("--:--"); int t_w=u8g2.getStrWidth(t.c_str()); u8g2.drawStr(128-t_w,10,t.c_str());

  // Balken
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0,Y0,"2m");  drawBarBasic(X,Y0-6,W,H,disp2m/(float)g_score_max);
  u8g2.drawStr(0,Y0+DY,"70"); drawBarBasic(X,Y0+DY-6,W,H,disp70/(float)g_score_max);
  u8g2.drawStr(0,Y0+2*DY,"10"); drawBarBasic(X,Y0+2*DY-6,W,H,disp10/(float)g_score_max);
  u8g2.drawStr(0,Y0+3*DY,"Sol"); drawBarSolarAnim(X,Y0+3*DY-6,W,H,dispSolar/(float)g_score_max,(int)round(tSolar));

  auto drawVal=[&](int x,int y,int s){ char b[12]; snprintf(b,sizeof(b),"%2d/%d",s,g_score_max); u8g2.drawStr(x,y,b); };
  drawVal(X+W+4,Y0,(int)round(t2m)); drawVal(X+W+4,Y0+DY,(int)round(t70)); drawVal(X+W+4,Y0+2*DY,(int)round(t10)); drawVal(X+W+4,Y0+3*DY,(int)round(tSolar));

  // Ticker
  drawTickerAtBottom();

  bool flareActive=(g_flare_enabled && (FL_penalty>0 || SOL_applied_delta10<0) && (millis()/400)%2==0); drawFlareIcon(124,20,flareActive);
  u8g2.sendBuffer();
}

// -------- Splash ----------------------
void runStartupAnimation(uint32_t duration_ms=8000){
  struct StarL{ float x,y,v; uint8_t tw; };
  const int N=48; StarL s[N];
  auto initOne=[&](StarL &a){ a.x=random(0,128); a.y=random(2,62); a.v=0.4f+(random(0,100)/100.0f)*1.1f; a.tw=random(0,8); };
  auto resetRight=[&](StarL &a){ a.x=128+random(0,16); a.y=random(2,62); a.v=0.4f+(random(0,100)/100.0f)*1.1f; a.tw=random(0,8); };
  auto drawStar=[&](int x,int y,uint8_t tw){ if(x<1||x>126||y<1||y>62) return; switch(tw&0x03){case 0:u8g2.drawPixel(x,y);break;case 1:u8g2.drawPixel(x,y);u8g2.drawPixel(x-1,y);u8g2.drawPixel(x+1,y);u8g2.drawPixel(x,y-1);u8g2.drawPixel(x,y+1);break;default:u8g2.drawPixel(x,y);u8g2.drawPixel(x+1,y);break;} };
  auto prog=[&](int x,int y,int w,int h,float f){ f=clamp01(f); u8g2.drawFrame(x,y,w,h); int fill=(int)round((w-2)*f); if(fill>0) u8g2.drawBox(x+1,y+1,fill,h-2); };
  for(int i=0;i<N;i++) initOne(s[i]);
  const char* steps[]={"WLAN checken...","Zeit sync (NTP)...","Init OLED/MDNS...","Server starten...","Berechne...","Bereit."}; const int NS=sizeof(steps)/sizeof(steps[0]);
  uint32_t t0=millis();
  while(millis()-t0<duration_ms){
    for(int i=0;i<N;i++){ s[i].x-=s[i].v; if(s[i].x<-2) resetRight(s[i]); s[i].tw++; }
    float frac=(millis()-t0)/(float)duration_ms; if(frac>1) frac=1; int idx=(int)floor(frac*NS); if(idx>=NS) idx=NS-1;
    u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tf);
    const char* title="Funkwetter"; u8g2.drawStr(0,10,title);
    int title_w=u8g2.getStrWidth(title); int y=8; drawWifiIcon(title_w+6,y,WiFi.status()==WL_CONNECTED); drawSunMoon(title_w+22,y,g_is_daylight);
    String tt=(g_time_str.length()>=16)?g_time_str.substring(11,16):String("--:--"); int tw=u8g2.getStrWidth(tt.c_str()); u8g2.drawStr(128-tw,10,tt.c_str());
    for(int i=0;i<N;i++) drawStar((int)s[i].x,(int)s[i].y,s[i].tw);
    u8g2.setFont(u8g2_font_7x13B_tf); const char* msg=steps[idx]; int mw=u8g2.getStrWidth(msg); u8g2.drawStr((128-mw)/2,36,msg);
    u8g2.setFont(u8g2_font_5x8_tf); String sub="Initialisiere Anzeige..."; int sw=u8g2.getStrWidth(sub.c_str()); u8g2.drawStr((128-sw)/2,48,sub.c_str());
    prog(8,54,112,6,frac); u8g2.sendBuffer(); delay(16);
  }
}

// -------- Setup / Loop ----------------
void setup(){
  Serial.begin(115200); delay(200);
  u8g2.begin(); u8g2.setPowerSave(0); u8g2.setContrast(OLED_CONTRAST);
  u8g2.clearBuffer(); u8g2.setFont(u8g2_font_7x13B_tf); u8g2.drawStr(0,12,"OLED OK..."); u8g2.drawFrame(0,16,128,16); u8g2.drawBox(2,18,124,12); u8g2.sendBuffer(); delay(600);

  loadState();
  wifiConnect();
  httpServer.begin();
  configTzTime(TZ_EU_BERLIN,"pool.ntp.org","time.cloudflare.com","time.nist.gov"); delay(200); updateDaylight();

  if(g_splash_enabled) runStartupAnimation(8000);
  g_bootstrap_done=false; g_bootstrap_at_ms=millis()+3000UL;

  // Overlay-Scheduler initialisieren
  if(g_overlay_enabled){
    OLED_OVERLAY=false;
    OLED_OVERLAY_NEXT = millis() + OVERLAY_PERIOD_MS;
  } else {
    OLED_OVERLAY=false; OLED_OVERLAY_NEXT=0;
  }

  // erster Remote-Pull
  if(g_remote_enabled){ REMOTE_LAST_MS=0; pollRemoteSensor(); }

  Serial.printf("HTTP: http://%s/  (mDNS: http://%s.local/)\n", WiFi.localIP().toString().c_str(), HOSTNAME);
}

void handleHttp(); // forward

void loop(){
  handleHttp();
  if(WiFi.status()!=WL_CONNECTED) wifiConnect();
  MDNS.update();

  static unsigned long tmr=0; if(millis()-tmr>15000){ tmr=millis(); updateDaylight(); }

  // Bootstrap: erste API-Fetches + Rechnen
  if(!g_bootstrap_done && millis()>=g_bootstrap_at_ms){
    if(g_solar_enabled){ SOL_fetched_ms=0; fetchSolarActivityAndMaybeAdjust10m(); }
    if(g_flare_enabled){ FL_fetched_ms=0; fetchAndApplyFlarePenalty(); }
    computeLocalTroposphere(); computeHF10m(); updateDaylight(); computeVHFHeuristicIfSingleHeight(); applyTodayOverrideVHF(); computeDisturbanceProbability(); oledRender();
    g_bootstrap_done=true;
  }

  // zyklische Fetches
  static unsigned long tFetch=0;
  if(millis()-tFetch>60000){ tFetch=millis();
    if(g_solar_enabled) fetchSolarActivityAndMaybeAdjust10m();
    if(g_flare_enabled) fetchAndApplyFlarePenalty();
    computeDisturbanceProbability();
  }

  // Remote zyklisch
  pollRemoteSensor();

  // Overlay-Zeitsteuerung
  unsigned long now = millis();
  if(g_overlay_enabled){
    if(!OLED_OVERLAY && (OLED_OVERLAY_NEXT!=0) && now >= OLED_OVERLAY_NEXT){
      OLED_OVERLAY = true;
      OLED_OVERLAY_END = now + OVERLAY_DURATION_MS;
      OLED_OVERLAY_NEXT = now + OVERLAY_PERIOD_MS;
    }
    if(OLED_OVERLAY && now >= OLED_OVERLAY_END){
      OLED_OVERLAY = false;
    }
  } else {
    OLED_OVERLAY = false;
  }

  oledRender();
}

// -------- WLAN / mDNS ----------
void wifiConnect(){
  WiFi.mode(WIFI_STA); WiFi.hostname(HOSTNAME);
  WiFi.begin(WIFI_SSID,WIFI_PASS);
  Serial.print("WLAN verbinden");
  int guard=0; while(WiFi.status()!=WL_CONNECTED && guard<120){ delay(500); Serial.print("."); guard++; }
  Serial.println(); Serial.print("IP: "); Serial.println(WiFi.localIP());
  if(!MDNS.begin(HOSTNAME)) Serial.println("mDNS start FEHLGESCHLAGEN");
  else { MDNS.addService("http","tcp",80); Serial.printf("mDNS: http://%s.local/\n", HOSTNAME); }
}

// -------- HTTP Handling --------------
void handleHttp(){
  WiFiClient c=httpServer.available(); if(!c) return; c.setTimeout(5000);
  String reqLine=c.readStringUntil('\n'); reqLine.trim();
  while(c.connected()){ String line=c.readStringUntil('\n'); if(line=="\r"||line.length()==0) break; }
  bool isRoot=reqLine.startsWith("GET / "); bool isLocal=reqLine.startsWith("GET /local"); bool isJSON=reqLine.startsWith("GET /json"); bool isReset=reqLine.startsWith("GET /reset ");
  if(isRoot){ c.setNoDelay(true); c.println("HTTP/1.1 303 See Other"); c.println("Location: /local"); c.println("Content-Length: 0"); c.println("Connection: close"); c.println(); c.flush(); delay(10); c.stop(); return; }
  if(isReset){
    L_P1=L_T1=L_RH1=L_z1=L_P2=L_T2=L_RH2=L_z2=NAN; L_N1=L_N2=L_dNdz=L_k=NAN; L_amp_2m=L_amp_70cm="unbekannt";
    L_score2m=L_score70=0; L_score2m_lbl=L_score70_lbl="unbekannt";
    H_SPOTS15=H_DXKM=H_SNRDB=NAN; H_score10=0; H_label10="unbekannt";
    g_flare_enabled=false; g_flare_dayOnly=true; FL_class=""; FL_peak=""; FL_penalty=0; FL_fetched_ms=0;
    g_solar_enabled=true;  g_solar_adjust10=true; SOL_fetched_ms=0; SOL_f107=NAN; SOL_kp=NAN; SOL_score=0; SOL_label="unbekannt"; SOL_applied_delta10=0;
    g_score_max=10; g_splash_enabled=true;
    g_vhf_heuristic_enable=true; FLAG_VHF_HEUR_USED=false; FLAG_HF10_FALLBACK_USED=false;
    g_today_override_vhf=false; g_today_score_vhf10=8;
    g_remote_enabled=false; g_remote_host="funk-remote.local"; REMOTE_OK=false; REMOTE_ERR=""; REMOTE_FAILS=0;
    // Overlay defaults
    g_overlay_enabled=true; OVERLAY_PERIOD_MS=10000; OVERLAY_DURATION_MS=2000; OLED_OVERLAY=false; OLED_OVERLAY_NEXT=millis()+OVERLAY_PERIOD_MS;
    EEPROM.begin(512); for(int i=0;i<512;i++) EEPROM.write(i,0xFF); EEPROM.commit();
    sendResponse(c,200,"OK","text/plain","reset ok"); return;
  }
  if(isJSON){ sendResponse(c,200,"OK","application/json; charset=utf-8", jsonOut()); return; }
  if(isLocal){ String fullPath=pathOnly(reqLine); sendResponse(c,200,"OK","text/html; charset=utf-8", pageLocal(fullPath)); return; }
  sendResponse(c,404,"Not Found","text/plain","404");
}
