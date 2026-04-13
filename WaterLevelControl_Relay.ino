#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

// ===== DEBUG (Virtual Terminal Proteus pin2=RX, pin3=TX) =====
SoftwareSerial dbg(2, 3);
#define DBG(s)  dbg.println(s)
#define DBGP(s) dbg.print(s)

// ===== PIN =====
#define PIN_TRIG 8
#define PIN_ECHO 7
#define PIN_PUMP 9

// ===== WIFI =====
#define WIFI_SSID "ELECTRONICSTREE"
#define WIFI_PASS "ADMIN"

// ===== RELAY SERVER (Render.com — HTTP, no HTTPS needed!) =====
// Ganti dengan URL Render.com kamu setelah deploy
#define RELAY_HOST "esp01-relay.onrender.com"   // ← GANTI INI
#define RELAY_PATH_D "/data"
#define RELAY_PATH_C "/command"
#define B44_INTERVAL  30000UL   // kirim data setiap 30 detik
#define CMD_INTERVAL  10000UL   // polling command setiap 10 detik

// ===== LCD =====
LiquidCrystal_I2C lcd(0x20, 20, 4);

// ===== KEYPAD =====
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {{'1','2','3','A'},{'4','5','6','B'},{'7','8','9','C'},{'*','0','#','D'}};
byte rp[ROWS] = {A0,A1,A2,A3};
byte cp[COLS] = {4,5,6,10};
Keypad kpd = Keypad(makeKeymap(keys), rp, cp, ROWS, COLS);

// ===== ENUMS =====
enum UnitMode:uint8_t{U_PCT,U_CM,U_M,U_MMH};
enum AMode:uint8_t{AM_MANUAL,AM_PID};
enum TMode:uint8_t{TM_PI,TM_PID};
enum Scr:uint8_t{S_SPLASH,S_STAT,S_MENU,S_MODE,S_PSUB,S_ARUN,S_MTUN,S_MPWM,S_PROT,S_CAL,S_CALV,S_CALU,S_CALD,S_UNIT,S_EDIT,S_ALARM,S_IP};
enum ET:uint8_t{E_NO,E_SP,E_KP,E_KI,E_KD,E_PMAX,E_RLIM,E_LRV,E_URV,E_MPWM,E_ALHI,E_ALLO};

// ===== ESP STATE MACHINE =====
enum ESPState:uint8_t{ES_AT,ES_RST,ES_CWMODE,ES_CWJAP,ES_CIPMUX,ES_READY,ES_SENDING};
ESPState espSt = ES_AT;
uint32_t espCmdMs = 0;
bool espReady=false, cmdPending=false, espSendingCmd=false, b44Connected=false;
#define ESP_TO 20000UL

// ===== VARS =====
UnitMode unitMode=U_PCT; AMode amode=AM_PID; TMode tmode=TM_PI;
Scr scr=S_SPLASH; ET etgt=E_NO; Scr eback=S_STAT;
uint8_t cur=0,mscr=0,unitScr=0;
uint8_t pidLastPWM=0,manPWM=0,pidPmax=240,pidRlim=25;
uint8_t tunNeed=8,tunXcount=0,sensorErrCount=0;
bool stog=false,pidOn=false,eeDirty=false;
bool tunActive=false,tunDone=false,tunCancel=false;
bool alarmHiAct=false,alarmLoAct=false;
float sLRV=50,sURV=5,sRAW=0;
float pidSP=50,pidKp=5,pidKi=0.2f,pidKd=0;
float pidInteg=0,pidPrevPV=0;
float alarmHi=90,alarmLo=10;
float tunBias=120,tunAmp=60,tunPVhi,tunPVlo,tunSign=0;
float emaVal=0,lastPV=0;
uint32_t tCtrl=0,tLCD=0,stogT=0,eeLastMs=0;
uint32_t tunLastMs=0,tunPrevMs=0,b44LastMs=0,cmdLastMs=0;
static char lb[21],ta[7],tb[7];
char nbuf[9],tunReason[9]="NONE";

// ===== FILTER =====
#define MED_N 3
float mbuf[MED_N]; uint8_t midx=0; bool mfill=false;
#define EMA_A 0.15f
#define Ts_MS 100UL
#define STOG_MS 600UL
#define EE_MIN 10000UL
#define SENSOR_ERR_MAX 5

// ===== EEPROM =====
#define EE_MAGIC 0xAB
struct EERec{uint8_t mag,unit,pmax,rlim;float sp,kp,ki,kd,lrv,urv,alHi,alLo;};
void eeMarkDirty(){eeDirty=true;}
void eeFlush(uint32_t now){
  if(!eeDirty||now-eeLastMs<EE_MIN)return;
  EERec d={EE_MAGIC,(uint8_t)unitMode,pidPmax,pidRlim,pidSP,pidKp,pidKi,pidKd,sLRV,sURV,alarmHi,alarmLo};
  EEPROM.put(0,d);eeLastMs=now;eeDirty=false;
}
void eeLoad(){
  EERec d;EEPROM.get(0,d);if(d.mag!=EE_MAGIC)return;
  unitMode=(UnitMode)d.unit;pidPmax=d.pmax;pidRlim=d.rlim;
  pidSP=constrain(d.sp,0,100);pidKp=constrain(d.kp,0,500);
  pidKi=constrain(d.ki,0,500);pidKd=constrain(d.kd,0,500);
  sLRV=constrain(d.lrv,0,200);sURV=constrain(d.urv,0,200);
  alarmHi=constrain(d.alHi,0,100);alarmLo=constrain(d.alLo,0,100);
}

// ===== HELPERS =====
static inline float cf(float x,float a,float b){return x<a?a:x>b?b:x;}
static inline int ci(int x,int a,int b){return x<a?a:x>b?b:x;}
void LP(uint8_t r,const char*s){lcd.setCursor(0,r);uint8_t i=0;while(i<20&&s[i])lcd.write(s[i++]);while(i++<20)lcd.write(' ');}
void LP_P(uint8_t r,const char*ps){lcd.setCursor(0,r);uint8_t i=0;char c;while(i<20&&(c=pgm_read_byte(ps+i++)))lcd.write(c);while(i++<20)lcd.write(' ');}

// ===== UNIT =====
const char ul0[]PROGMEM="% ";const char ul1[]PROGMEM="cm";
const char ul2[]PROGMEM="m ";const char ul3[]PROGMEM="mmH2O";
const char*const ulT[]PROGMEM={ul0,ul1,ul2,ul3};
const char*getUL(){static char b[6];strcpy_P(b,(const char*)pgm_read_ptr(&ulT[unitMode]));return b;}
void fmtVal(float v,char*buf){if(unitMode==U_M){dtostrf(v,4,2,buf);buf[4]=0;}else snprintf(buf,5,"%4d",(int)(v+0.5f));}
void stopPump(){analogWrite(PIN_PUMP,0);pidLastPWM=0;}

// ===== ESP =====
void espF(const __FlashStringHelper*s){DBGP(F("[TX] "));DBG(s);Serial.println(s);espCmdMs=millis();}
void espS(const char*s){DBGP(F("[TX] "));DBG(s);Serial.println(s);espCmdMs=millis();}
void espInit(){
  espReady=false;b44Connected=false;cmdPending=false;espSendingCmd=false;espSt=ES_AT;
  DBG(F("[INIT] Resetting ESP..."));delay(3000);espF(F("AT"));
}

void processLine(const char*line){
  if(!line[0])return;DBGP(F("[RX] "));DBG(line);
  switch(espSt){
    case ES_AT:
      if(strstr(line,"OK")){DBG(F("[ST] AT OK -> RST"));espF(F("AT+RST"));espSt=ES_RST;}break;
    case ES_RST:
      if(strstr(line,"ready")||strstr(line,"OK")){
        DBG(F("[ST] RST OK -> CWMODE"));delay(3000);espF(F("AT+CWMODE=1"));espSt=ES_CWMODE;}break;
    case ES_CWMODE:
      if(strstr(line,"OK")||strstr(line,"no change")){
        DBG(F("[ST] CWMODE OK -> CWJAP"));delay(500);
        char cmd[48];snprintf(cmd,sizeof(cmd),"AT+CWJAP=\"%s\",\"%s\"",WIFI_SSID,WIFI_PASS);
        espS(cmd);espSt=ES_CWJAP;}break;
    case ES_CWJAP:
      if(strstr(line,"GOT IP")){
        DBG(F("[ST] GOT IP -> CIPMUX"));delay(500);espF(F("AT+CIPMUX=0"));espSt=ES_CIPMUX;}
      if(strstr(line,"FAIL")){
        DBG(F("[ERR] CWJAP FAIL, retry..."));delay(2000);
        char cmd[48];snprintf(cmd,sizeof(cmd),"AT+CWJAP=\"%s\",\"%s\"",WIFI_SSID,WIFI_PASS);espS(cmd);}break;
    case ES_CIPMUX:
      if(strstr(line,"OK")){
        espReady=true;espSt=ES_READY;
        b44LastMs=millis()-B44_INTERVAL;cmdLastMs=millis();
        DBG(F("[ST] READY! WiFi connected."));if(scr==S_IP)lcd.clear();}break;
    case ES_SENDING:
      if(strstr(line,"+HTTPCLIENT:")){
        DBGP(F("[HTTP RESP] "));DBG(line);
        if(espSendingCmd&&cmdPending)parseCommandResponse(line);
        else b44Connected=true;}
      if(strstr(line,"OK")){
        DBG(F("[ST] SEND OK -> READY"));
        if(!espSendingCmd)b44Connected=true;
        espSendingCmd=false;espSt=ES_READY;}
      if(strstr(line,"ERROR")||strstr(line,"FAIL")){
        DBGP(F("[ERR] SEND FAIL: "));DBG(line);
        cmdPending=false;espSendingCmd=false;espReady=false;b44Connected=false;espInit();}break;
    case ES_READY:
      if(strstr(line,"WIFI DISCONNECT")){
        DBG(F("[ERR] WiFi disconnected!"));espReady=false;b44Connected=false;espSt=ES_CWJAP;
        char cmd[48];snprintf(cmd,sizeof(cmd),"AT+CWJAP=\"%s\",\"%s\"",WIFI_SSID,WIFI_PASS);espS(cmd);}break;
  }
}

void readESP(){
  static char buf[160];static uint8_t bi=0;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||bi>=159){buf[bi]=0;bi=0;if(buf[0])processLine(buf);}
    else if(c!='\r')buf[bi++]=c;
  }
  if(espSt!=ES_READY&&millis()-espCmdMs>ESP_TO){DBG(F("[ERR] Timeout!"));espInit();}
}

// ===== SEND DATA KE RELAY (bukan langsung ke Base44) =====
void sendToBase44(){
  if(!espReady||espSt!=ES_READY)return;
  DBG(F("[SEND] Sending to Relay..."));
  char pv[7],sp[7],kpS[8],kiS[8],kdS[8];
  dtostrf(lastPV,4,1,pv);dtostrf(pidSP,4,1,sp);
  dtostrf(pidKp,4,2,kpS);dtostrf(pidKi,4,3,kiS);dtostrf(pidKd,4,3,kdS);
  char*p=pv;while(*p==' ')p++;
  char*s=sp;while(*s==' ')s++;
  char*k1=kpS;while(*k1==' ')k1++;
  char*k2=kiS;while(*k2==' ')k2++;
  char*k3=kdS;while(*k3==' ')k3++;
  char url[256];
  // ── Kirim GET ke relay server Render.com ──
  snprintf(url,sizeof(url),
    "AT+HTTPCLIENT=2,0,\"http://%s%s"
    "?pv=%s&sp=%s&out=%d&mode=%c"
    "&run=%d&ah=%d&al=%d"
    "&kp=%s&ki=%s&kd=%s"
    "&pm=%d&rl=%d\",,,1",
    RELAY_HOST, RELAY_PATH_D,
    p,s,pidLastPWM,amode==AM_PID?'P':'M',
    (pidOn||amode==AM_MANUAL)?1:0,
    alarmHiAct?1:0,alarmLoAct?1:0,
    k1,k2,k3,pidPmax,pidRlim);
  espSendingCmd=false;espS(url);espSt=ES_SENDING;
}

// ===== POLL COMMAND DARI RELAY =====
void pollCommand(){
  if(!espReady||espSt!=ES_READY)return;
  DBG(F("[POLL] Polling command..."));
  char url[128];
  snprintf(url,sizeof(url),"AT+HTTPCLIENT=2,0,\"http://%s%s\",,,1",RELAY_HOST,RELAY_PATH_C);
  espSendingCmd=true;cmdPending=true;espS(url);espSt=ES_SENDING;
}

// ===== PARSE COMMAND RESPONSE =====
void parseCommandResponse(const char*line){
  const char*p;
  p=strstr(line,"\"cmd_sp\":");
  if(p&&strncmp(p+9,"null",4)!=0){float v=atof(p+9);if(v>=0&&v<=100){pidSP=v;eeMarkDirty();}}
  p=strstr(line,"\"cmd_mode\":\"");
  if(p){char m=*(p+12);if(m=='P'){amode=AM_PID;pidOn=true;pidInteg=0;}else if(m=='M'){amode=AM_MANUAL;pidOn=false;}}
  p=strstr(line,"\"cmd_pwm\":");
  if(p&&strncmp(p+10,"null",4)!=0){int v=atoi(p+10);if(v>=0&&v<=255)manPWM=(uint8_t)v;}
  p=strstr(line,"\"cmd_running\":");
  if(p&&strncmp(p+14,"null",4)!=0){int r=atoi(p+14);if(r==1)pidOn=true;else if(r==0){pidOn=false;stopPump();}}
  cmdPending=false;DBG(F("[CMD] Applied!"));
}

// ===== MENU =====
const char mi0[]PROGMEM="SET SP";const char mi1[]PROGMEM="MODE";const char mi2[]PROGMEM="SENSOR CALIB";
const char mi3[]PROGMEM="PROTECT";const char mi4[]PROGMEM="UNITS";const char mi5[]PROGMEM="ALARM";const char mi6[]PROGMEM="IP INFO";
const char*const mItems[]PROGMEM={mi0,mi1,mi2,mi3,mi4,mi5,mi6};

// ===== UNIT CONVERT =====
float toUnit(float p){float sp=sLRV-sURV;if(sp<0.01f)sp=0.01f;float cm=p*sp/100.0f;
  switch(unitMode){case U_PCT:return p;case U_CM:return cm;case U_M:return cm/100.0f;case U_MMH:return cm*10.0f;}return p;}

// ===== LCD SCREENS ===== (identik dengan kode asli)
void dSplash(){LP_P(0,PSTR("--------------------"));LP_P(1,PSTR(" WATER LEVEL CONTROL"));LP_P(2,PSTR("--------------------"));LP_P(3,stog?PSTR("  PRESS D TO START "):PSTR("                    "));}
void dStat(){
  char al[4]="   ";if(alarmHiAct)memcpy_P(al,PSTR("!HI"),3);else if(alarmLoAct)memcpy_P(al,PSTR("!LO"),3);al[3]=0;
  snprintf_P(lb,21,PSTR("OUT:%-3d %s %s"),pidLastPWM,amode==AM_MANUAL?" [M] ":"[PID]",al);LP(0,lb);
  LP_P(1,PSTR("--------------------"));
  char vb[5];fmtVal(toUnit(lastPV),vb);snprintf_P(lb,21,PSTR("PV  :%s %s"),vb,getUL());LP(2,lb);
  fmtVal(toUnit(pidSP),vb);snprintf_P(lb,21,PSTR("SP  :%s %s"),vb,getUL());LP(3,lb);}
void dMenu(){LP_P(0,PSTR("=== MENU ===        "));char ib[16];
  for(uint8_t r=0;r<3;r++){uint8_t i=mscr+r;if(i>=7){LP_P(r+1,PSTR(""));continue;}
  strcpy_P(ib,(const char*)pgm_read_ptr(&mItems[i]));snprintf_P(lb,21,PSTR("%s%-19s"),i==cur?">":"  ",ib);LP(r+1,lb);}}
void dMode(){LP_P(0,PSTR("MODE:               "));LP_P(1,cur==0?PSTR(">PID            "):PSTR(" PID            "));LP_P(2,cur==1?PSTR(">MANUAL         "):PSTR(" MANUAL         "));LP_P(3,PSTR("A=Back  B=Select    "));}
void dPSub(){LP_P(0,PSTR("PID MODE:           "));LP_P(1,cur==0?PSTR(">AUTO TUNE PI   "):PSTR(" AUTO TUNE PI   "));LP_P(2,cur==1?PSTR(">AUTO TUNE PID  "):PSTR(" AUTO TUNE PID  "));LP_P(3,cur==2?PSTR(">MANUAL TUNE    "):PSTR(" MANUAL TUNE    "));}
void dArun(){const char*m=(tmode==TM_PI)?"PI":"PID";
  if(tunActive){snprintf_P(lb,21,PSTR("AUTO %s RUN        "),m);LP(0,lb);dtostrf(lastPV,4,1,ta);dtostrf(pidSP,4,1,tb);snprintf_P(lb,21,PSTR("PV:%s SP:%s%%  "),ta,tb);LP(1,lb);snprintf_P(lb,21,PSTR("OUT:%-3d x:%-3d     "),pidLastPWM,tunXcount);LP(2,lb);LP_P(3,PSTR("A=Back  C=Cancel   "));}
  else if(tunCancel){snprintf_P(lb,21,PSTR("AUTO %s CANCEL     "),m);LP(0,lb);LP(1,tunReason);LP_P(2,PSTR(""));LP_P(3,PSTR("A=Back             "));}
  else{snprintf_P(lb,21,PSTR("AUTO %s DONE       "),m);LP(0,lb);dtostrf(pidKp,6,2,ta);snprintf_P(lb,21,PSTR("KP:%s             "),ta);LP(1,lb);dtostrf(pidKi,6,2,ta);dtostrf(pidKd,6,2,tb);snprintf_P(lb,21,PSTR("KI:%s KD:%s  "),ta,tb);LP(2,lb);LP_P(3,PSTR("A=Back             "));}}
void dMtun(){LP_P(0,PSTR("MANUAL TUNE  B=EDIT "));dtostrf(pidKp,6,2,ta);snprintf_P(lb,21,PSTR("%sKP %s      "),cur==0?">":"  ",ta);LP(1,lb);dtostrf(pidKi,6,2,ta);snprintf_P(lb,21,PSTR("%sKI %s      "),cur==1?">":"  ",ta);LP(2,lb);dtostrf(pidKd,6,2,ta);snprintf_P(lb,21,PSTR("%sKD %s      "),cur==2?">":"  ",ta);LP(3,lb);}
void dMpwm(){char pv[5],sp[5];fmtVal(toUnit(lastPV),pv);fmtVal(toUnit(pidSP),sp);LP_P(0,PSTR("MANUAL PWM CONTROL  "));snprintf_P(lb,21,PSTR("PV:%s SP:%s %s "),pv,sp,getUL());LP(1,lb);snprintf_P(lb,21,PSTR("PWM:%-3d  (0-255)  "),manPWM);LP(2,lb);LP_P(3,PSTR("B=EDIT 2/8=+/- C=Bk"));}
void dAlarm(){LP_P(0,PSTR("ALARM SETTING B=EDT "));snprintf_P(lb,21,PSTR("%sHI  %3d%%         "),cur==0?">":"  ",(int)alarmHi);LP(1,lb);snprintf_P(lb,21,PSTR("%sLO  %3d%%         "),cur==1?">":"  ",(int)alarmLo);LP(2,lb);LP_P(3,PSTR("A=Back  8/2=Select  "));}
void dIP(){LP_P(0,PSTR("ESP01 - RELAY MODE  "));
  if(espReady){LP_P(1,PSTR("WiFi : TERHUBUNG!   "));snprintf_P(lb,21,PSTR("SSID :%-14s"),WIFI_SSID);LP(2,lb);LP_P(3,b44Connected?PSTR("RELAY: TERHUBUNG!   "):PSTR("RELAY: MENGIRIM...  "));}
  else{switch(espSt){case ES_AT:case ES_RST:LP_P(1,PSTR("Init ESP01...       "));break;case ES_CWMODE:LP_P(1,PSTR("Set WiFi mode...    "));break;case ES_CWJAP:LP_P(1,PSTR("Menghubungkan WiFi.."));break;case ES_CIPMUX:LP_P(1,PSTR("Setup koneksi...    "));break;default:LP_P(1,PSTR("WiFi: Tunggu...     "));break;}
  snprintf_P(lb,21,PSTR("SSID :%-14s"),WIFI_SSID);LP(2,lb);LP_P(3,PSTR("RELAY: TIDAK TERHUBG"));}}
void dCal(){dtostrf(sRAW,5,1,ta);LP_P(0,PSTR("SENSOR CALIB        "));snprintf_P(lb,21,PSTR("RAW:%s cm         "),ta);LP(1,lb);LP_P(2,cur==0?PSTR(">SET LRV (0%)    "):PSTR(" SET LRV (0%)   "));LP_P(3,cur==1?PSTR(">SET URV (100%)  "):PSTR(" SET URV (100%) "));}
void dCalSub(bool isL){dtostrf(sRAW,5,1,ta);dtostrf(isL?sLRV:sURV,5,1,tb);LP_P(0,isL?PSTR("SET LRV (0%)        "):PSTR("SET URV (100%)      "));snprintf_P(lb,21,PSTR("RAW:%s cm         "),ta);LP(1,lb);snprintf_P(lb,21,PSTR("%s:%s cm         "),isL?"LRV":"URV",tb);LP(2,lb);LP_P(3,isL?PSTR("B=Set #=Next A=Bk   "):PSTR("B=Set #=Done A=Bk   "));}
void dCalD(){dtostrf(sLRV,5,1,ta);dtostrf(sURV,5,1,tb);LP_P(0,PSTR("CALIBRATION OK!     "));snprintf_P(lb,21,PSTR("LRV: %s cm       "),ta);LP(1,lb);snprintf_P(lb,21,PSTR("URV: %s cm       "),tb);LP(2,lb);LP_P(3,PSTR("A=Back to Menu      "));}
void dProt(){LP_P(0,PSTR("PROTECT  B=EDIT     "));snprintf_P(lb,21,PSTR("%sPMAX %-3d         "),cur==0?">":"  ",pidPmax);LP(1,lb);snprintf_P(lb,21,PSTR("%sRLIM %-3d         "),cur==1?">":"  ",pidRlim);LP(2,lb);LP_P(3,PSTR("A=Back  8/2=Scroll  "));}
void dUnit(){LP_P(0,PSTR("SELECT UNIT:        "));char ub[8];
  for(uint8_t r=0;r<3;r++){uint8_t i=unitScr+r;if(i>=4){LP_P(r+1,PSTR(""));continue;}
  strcpy_P(ub,(const char*)pgm_read_ptr(&ulT[i]));snprintf_P(lb,21,PSTR("%s%-18s"),i==cur?">":"  ",ub);LP(r+1,lb);}}
void dEdit(){LP_P(0,PSTR("EDIT VALUE          "));snprintf_P(lb,21,PSTR("Input: %-13s"),nbuf);LP(1,lb);LP_P(2,PSTR("D=decimal  *=del    "));LP_P(3,PSTR("A=Cancel  #=Enter   "));}
void drawUI(){static Scr ls=(Scr)255;if(scr!=ls){lcd.clear();ls=scr;}
  switch(scr){case S_SPLASH:dSplash();break;case S_STAT:dStat();break;case S_MENU:dMenu();break;case S_MODE:dMode();break;case S_PSUB:dPSub();break;case S_ARUN:dArun();break;case S_MTUN:dMtun();break;case S_MPWM:dMpwm();break;case S_PROT:dProt();break;case S_CAL:dCal();break;case S_CALV:dCalSub(true);break;case S_CALU:dCalSub(false);break;case S_CALD:dCalD();break;case S_UNIT:dUnit();break;case S_EDIT:dEdit();break;case S_ALARM:dAlarm();break;case S_IP:dIP();break;}}

// ===== SENSOR =====
float readDist(){digitalWrite(PIN_TRIG,LOW);delayMicroseconds(2);digitalWrite(PIN_TRIG,HIGH);delayMicroseconds(10);digitalWrite(PIN_TRIG,LOW);unsigned long d=pulseIn(PIN_ECHO,HIGH,30000);if(!d)return NAN;sRAW=(float)d*0.0343f/2.0f;return sRAW;}
float medN(float x){mbuf[midx++]=x;if(midx>=MED_N){midx=0;mfill=true;}uint8_t n=mfill?MED_N:midx;float t[MED_N];memcpy(t,mbuf,n*sizeof(float));for(uint8_t i=0;i<n-1;i++)for(uint8_t j=i+1;j<n;j++)if(t[j]<t[i]){float tmp=t[i];t[i]=t[j];t[j]=tmp;}return t[n/2];}
float readPV(){float d=readDist();if(isnan(d)){sensorErrCount++;return emaVal;}sensorErrCount=0;float span=sLRV-sURV;if(span<0.01f)span=0.01f;float p=cf((d-sLRV)*(-100.0f/span),0,100);float m=medN(p);if(emaVal==0)emaVal=m;else emaVal+=EMA_A*(m-emaVal);static float pvS=0;pvS+=0.1f*(emaVal-pvS);return pvS;}

// ===== PID =====
void setPWM(int pw){if(pw<12)pw=0;if(pw>0&&pw<60)pw=60;pw=ci(pw,0,pidPmax);int delta=pw-pidLastPWM;if(delta>(int)pidRlim)pw=pidLastPWM+pidRlim;if(delta<-(int)pidRlim)pw=pidLastPWM-pidRlim;pidLastPWM=(uint8_t)ci(pw,0,pidPmax);analogWrite(PIN_PUMP,pidLastPWM);}
void pidStep(float pv,float dt){float e=pidSP-pv;if(fabs(e)<0.5f)e=0;e=cf(e,-20,20);float P=pidKp*e;float D=-pidKp*pidKd*(pv-pidPrevPV)/dt;float us=cf(P+pidInteg+D,0,(float)pidPmax);if(!((us>=(float)pidPmax-0.001f&&e>0)||(us<=0&&e<0)))pidInteg=cf(pidInteg+pidKi*e*dt,-255,255);setPWM((int)cf(P+pidInteg+D,0,(float)pidPmax));}

// ===== AUTOTUNE =====
void cancelTune(const char*r){tunActive=false;tunCancel=true;tunDone=false;strncpy(tunReason,r,8);tunReason[8]=0;pidInteg=0;tunSign=0;}
void startTune(){tunCancel=false;tunDone=false;memcpy_P(tunReason,PSTR("NONE"),5);tunActive=true;pidOn=true;tunXcount=0;tunPVhi=-1e9f;tunPVlo=1e9f;tunPrevMs=0;tunLastMs=0;tunSign=0;pidInteg=0;}
void tuneStep(float pv){float e=pidSP-pv;if(pv>tunPVhi)tunPVhi=pv;if(pv<tunPVlo)tunPVlo=pv;setPWM((int)(tunBias+(e>=0?tunAmp:-tunAmp)));float s=(e>=0)?1.0f:-1.0f;if(tunSign==0)tunSign=s;if(s!=tunSign){tunXcount++;tunPrevMs=tunLastMs;tunLastMs=millis();tunSign=s;}
  if(tunXcount>=tunNeed&&tunPrevMs>0){float Tu=((float)(tunLastMs-tunPrevMs)*2.0f)/1000.0f;float a=(tunPVhi-tunPVlo)/2.0f;if(a<0.01f)a=0.01f;float Ku=(4.0f*tunAmp)/(3.14159f*a);
    if(tmode==TM_PI){float Ti=Tu/1.2f;pidKp=cf(0.45f*Ku,0.01f,200);pidKi=cf(Ti>0.001f?pidKp/Ti:0,0,50);pidKd=0;}
    else{float Ti=Tu/2.0f,Td=Tu/8.0f;pidKp=cf(0.6f*Ku,0.01f,200);pidKi=cf(Ti>0.001f?pidKp/Ti:0,0,50);pidKd=cf(pidKp*Td,0,50);}
    amode=AM_PID;pidOn=true;tunActive=false;tunDone=true;tunCancel=false;pidInteg=0;eeMarkDirty();}
  if(tunLastMs>0&&millis()-tunLastMs>15000UL)cancelTune("TIMEOUT");}
void emergencyStop(){pidOn=false;tunActive=false;tunCancel=false;tunDone=false;pidInteg=0;manPWM=0;tunSign=0;stopPump();lcd.noBacklight();amode=AM_MANUAL;while(true)delay(1000);}

// ===== KEYPAD =====
void startEdit(ET t,Scr b){etgt=t;nbuf[0]=0;eback=b;scr=S_EDIT;}
void commitEdit(){if(!nbuf[0])return;float fv=atof(nbuf);int iv=atoi(nbuf);switch(etgt){case E_SP:pidSP=cf(fv,0,100);break;case E_KP:pidKp=cf(fv,0,500);break;case E_KI:pidKi=cf(fv,0,500);break;case E_KD:pidKd=cf(fv,0,500);break;case E_PMAX:pidPmax=(uint8_t)ci(iv,0,255);break;case E_RLIM:pidRlim=(uint8_t)ci(iv,1,255);break;case E_LRV:sLRV=cf(fv,0,200);break;case E_URV:sURV=cf(fv,0,200);break;case E_MPWM:manPWM=(uint8_t)ci(iv,0,255);break;case E_ALHI:alarmHi=cf(fv,0,100);break;case E_ALLO:alarmLo=cf(fv,0,100);break;default:break;}etgt=E_NO;nbuf[0]=0;eeMarkDirty();}
void goBack(){if(scr==S_EDIT){scr=eback;return;}if(scr==S_MENU){scr=S_STAT;cur=0;mscr=0;return;}if(scr==S_MODE||scr==S_PROT||scr==S_CAL||scr==S_UNIT||scr==S_ALARM||scr==S_IP){scr=S_MENU;cur=0;mscr=0;return;}if(scr==S_PSUB){scr=S_MODE;cur=0;return;}if(scr==S_ARUN){if(tunActive)cancelTune("USER");scr=S_PSUB;cur=(tmode==TM_PI)?0:1;return;}if(scr==S_MTUN){scr=S_PSUB;cur=2;return;}if(scr==S_MPWM){scr=S_STAT;return;}if(scr==S_CALV||scr==S_CALU||scr==S_CALD){scr=S_CAL;return;}scr=S_STAT;cur=0;mscr=0;}
void handleKey(char k){if(k=='D'){if(scr==S_SPLASH){lcd.backlight();amode=AM_PID;pidOn=true;pidInteg=0;scr=S_STAT;cur=0;mscr=0;return;}if(scr==S_EDIT){uint8_t l=strlen(nbuf);if(!strchr(nbuf,'.')&&l<8){nbuf[l]='.';nbuf[l+1]=0;}return;}emergencyStop();return;}if(k=='A'){goBack();return;}if(k=='C'){if(tunActive){cancelTune("USER");return;}if(scr==S_MPWM){scr=S_STAT;return;}return;}switch(scr){case S_STAT:if(k=='#'){scr=S_MENU;cur=0;mscr=0;}break;case S_MENU:if(k=='8'){cur=(cur+1)%7;if(cur<mscr)mscr=cur;if(cur>=mscr+3)mscr=cur-2;}else if(k=='2'){cur=(cur+6)%7;if(cur<mscr)mscr=cur;if(cur>=mscr+3)mscr=cur-2;}else if(k=='B'||k=='#'){switch(cur){case 0:startEdit(E_SP,S_MENU);break;case 1:scr=S_MODE;cur=0;break;case 2:scr=S_CAL;cur=0;break;case 3:scr=S_PROT;cur=0;break;case 4:scr=S_UNIT;cur=(uint8_t)unitMode;break;case 5:scr=S_ALARM;cur=0;break;case 6:scr=S_IP;break;}}break;case S_MODE:if(k=='8'||k=='2')cur=(cur+1)%2;else if(k=='B'){if(cur==0){scr=S_PSUB;cur=0;amode=AM_PID;}else{amode=AM_MANUAL;manPWM=0;pidOn=false;scr=S_MPWM;cur=0;}}break;case S_PSUB:if(k=='8'||k=='2')cur=(cur+1)%3;else if(k=='B'){amode=AM_PID;if(cur==0){tmode=TM_PI;startTune();scr=S_ARUN;}else if(cur==1){tmode=TM_PID;startTune();scr=S_ARUN;}else{pidOn=true;scr=S_MTUN;cur=0;}}break;case S_MTUN:if(k=='8'||k=='2')cur=(cur+1)%3;else if(k=='B'){if(cur==0)startEdit(E_KP,S_MTUN);else if(cur==1)startEdit(E_KI,S_MTUN);else startEdit(E_KD,S_MTUN);}break;case S_MPWM:if(k=='8')manPWM=(uint8_t)ci(manPWM+5,0,255);else if(k=='2')manPWM=(uint8_t)ci(manPWM-5,0,255);else if(k=='B')startEdit(E_MPWM,S_MPWM);break;case S_CAL:if(k=='8'||k=='2')cur=(cur+1)%2;else if(k=='B'){readDist();if(cur==0){sLRV=sRAW;scr=S_CALV;}else{sURV=sRAW;scr=S_CALU;}}break;case S_CALV:if(k=='B'){readDist();sLRV=sRAW;}else if(k=='#')scr=S_CALU;break;case S_CALU:if(k=='B'){readDist();sURV=sRAW;}else if(k=='#'){if(sLRV>sURV+5){scr=S_CALD;eeMarkDirty();}}break;case S_PROT:if(k=='8'||k=='2')cur=(cur+1)%2;else if(k=='B'){if(cur==0)startEdit(E_PMAX,S_PROT);else startEdit(E_RLIM,S_PROT);}break;case S_ALARM:if(k=='8'||k=='2')cur=(cur+1)%2;else if(k=='B'){if(cur==0)startEdit(E_ALHI,S_ALARM);else startEdit(E_ALLO,S_ALARM);}break;case S_UNIT:if(k=='8'){cur=(cur+1)%4;if(cur<unitScr)unitScr=cur;if(cur>=unitScr+3)unitScr=cur-2;}else if(k=='2'){cur=(uint8_t)((cur+3)%4);if(cur<unitScr)unitScr=cur;if(cur>=unitScr+3)unitScr=cur-2;}else if(k=='B'||k=='#'){unitMode=(UnitMode)cur;eeMarkDirty();scr=S_MENU;cur=0;mscr=0;}break;case S_EDIT:{uint8_t l=strlen(nbuf);if(k=='*'){if(l>0)nbuf[l-1]=0;}else if(k=='#'){commitEdit();goBack();}else if(k>='0'&&k<='9'){if(l<8){nbuf[l]=k;nbuf[l+1]=0;}}break;}default:break;}}

// ===== SETUP =====
void setup(){
  pinMode(PIN_TRIG,OUTPUT);pinMode(PIN_ECHO,INPUT);pinMode(PIN_PUMP,OUTPUT);
  Serial.begin(9600);dbg.begin(9600);lcd.init();lcd.backlight();
  DBG(F("=== WATER LEVEL v3 + RELAY ==="));
  DBG(F("Relay: Render.com HTTP Relay"));
  memset(mbuf,0,sizeof(mbuf));eeLoad();stopPump();
  amode=AM_PID;pidOn=false;manPWM=0;stog=true;
  stogT=millis();tCtrl=millis();
  b44LastMs=millis();cmdLastMs=millis()+15000UL;
  espInit();
}

// ===== LOOP =====
void loop(){
  uint32_t now=millis();
  if(scr==S_SPLASH&&now-stogT>=STOG_MS){stog=!stog;stogT=now;}
  char k=kpd.getKey();if(k){handleKey(k);tLCD=now-300UL;}
  readESP();
  if(now-tCtrl>=Ts_MS){
    float dt=cf((float)(now-tCtrl)/1000.0f,0.02f,1.0f);tCtrl=now;
    if(scr!=S_SPLASH){
      float pv=readPV();
      if(sensorErrCount>=SENSOR_ERR_MAX){stopPump();pidInteg=0;}
      else{lastPV=pv;alarmHiAct=(pv>alarmHi);alarmLoAct=(pv<alarmLo);
        if(alarmHiAct||alarmLoAct){stopPump();pidInteg=0;}
        else if(amode==AM_MANUAL)setPWM(manPWM);
        else if(pidOn){if(tunActive)tuneStep(pv);else pidStep(pv,dt);}
        else stopPump();
        pidPrevPV=pv;}
    }else stopPump();
  }
  // Kirim data ke relay setiap B44_INTERVAL
  if(espReady&&espSt==ES_READY&&scr!=S_SPLASH&&now-b44LastMs>=B44_INTERVAL){b44LastMs=now;sendToBase44();}
  // Poll command dari relay setiap CMD_INTERVAL
  if(espReady&&espSt==ES_READY&&now-cmdLastMs>=CMD_INTERVAL){cmdLastMs=now;pollCommand();}
  eeFlush(now);
  if(now-tLCD>=300){tLCD=now;drawUI();}
}
