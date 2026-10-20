#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <string>
#include <deque>
#include <stdarg.h>
#include <iostream>
// ---- simulated world ----
static double now_us = 0, pos = 60000;          // carriage position in encoder counts (true)
static bool blocked = false;                    // motor stalls (steps do nothing)
static const double COUNTS_PER_STEP = 8.0, MIN_TRIG = 500, MAX_TRIG = 127500;
static std::deque<char> rx; static std::string txbuf;
static uint8_t wire_reg; static int wire_left; static uint8_t wire_bytes[2];
unsigned long millis(){ return (unsigned long)(now_us/1000); }
void delay(unsigned long ms){ now_us += ms*1000; } void delayMicroseconds(unsigned int u){ now_us += u; }
void pinMode(int,int){} void analogReadResolution(int){} int analogRead(int){ return 2048; }
int digitalRead(int p){ if(p==27) return pos<=MIN_TRIG; if(p==14) return pos>=MAX_TRIG; return 0; }
void SerialT::begin(int){} int SerialT::available(){ return rx.size(); } int SerialT::read(){ char c=rx.front(); rx.pop_front(); return c; }
static void emit(const std::string&s){ txbuf+=s; }
void SerialT::print(const char*s){ emit(s);} void SerialT::print(float f,int d){ char b[32]; snprintf(b,32,"%.*f",d,f); emit(b);}
void SerialT::println(const char*s){ emit(std::string(s)+"\n");} void SerialT::println(float f,int d){ print(f,d); emit("\n");}
int SerialT::printf(const char*f,...){ char b[256]; va_list a; va_start(a,f); int n=vsnprintf(b,256,f,a); va_end(a); emit(b); return n; }
SerialT Serial; WireT Wire;
void WireT::begin(int,int){} void WireT::setClock(uint32_t){} void WireT::beginTransmission(uint8_t){} void WireT::write(uint8_t r){ wire_reg=r; }
int WireT::endTransmission(bool){ return 0; }
int WireT::requestFrom(uint8_t,uint8_t n){ now_us += 300;                       // one I2C read costs 300 us
  if(wire_reg==0x0C){ long raw=((long)pos)%4096; if(raw<0) raw+=4096; wire_bytes[0]=raw>>8; wire_bytes[1]=raw&0xFF; }
  else { wire_bytes[0]=0x20; } wire_left=n; return n; }
int WireT::available(){ return wire_left; } int WireT::read(){ int i=(wire_reg==0x0C)?(2-wire_left):0; wire_left--; return wire_bytes[i]; }
static float spd=0; static double last_step_us=0;
AccelStepper::AccelStepper(int,int,int){} void AccelStepper::setSpeed(float s){ spd=s; } void AccelStepper::setMaxSpeed(float){} void AccelStepper::setMinPulseWidth(unsigned int){}
bool AccelStepper::runSpeed(){ if(spd==0) return false; double iv=1e6/fabs(spd);
  if(now_us-last_step_us>=iv){ last_step_us=now_us; if(!blocked) pos += (spd>0?1:-1)*COUNTS_PER_STEP; return true;} return false; }
// ---- test driver (protocol of rail_esp32.ino) ----
void setup(); void loop();
static std::string drain(){ std::string s=txbuf; txbuf.clear(); return s; }
static void send(const char*cmd){ for(const char*p=cmd;*p;++p) rx.push_back(*p); rx.push_back('\n'); }
static void spin(int n){ for(int i=0;i<n;i++){ loop(); now_us+=200; } }
static std::string run_until(const char* cmd, const char* expect, double max_s){
  send(cmd); std::string acc; double t0=now_us;
  while(now_us-t0<max_s*1e6){ loop(); now_us+=200; acc+=drain(); if(acc.find(expect)!=std::string::npos) return acc; }
  return acc+"<<TIMEOUT>>";
}
static int fails=0;
static bool has(const std::string&o,const char*x){ return o.find(x)!=std::string::npos; }
static void check(bool ok,const char*what,const std::string&out=""){ std::cout<<(ok?"PASS ":"FAIL ")<<what<<(ok?"":("  -> "+out))<<"\n"; if(!ok) fails++; }
static double mm(){ return (pos-MIN_TRIG)/(4096/8.0); }
int main(){
  setup(); std::string o=drain();
  check(has(o,"UNHOMED"),"boot: announces UNHOMED, no homing",o);
  double p0=pos; spin(20000); check(pos==p0,"boot: rail does not move on its own (4 s idle)");
  o=run_until("STATUS?","STATE",1); check(has(o,"STATE UNHOMED"),"STATUS? says UNHOMED",o);
  o=run_until("MOVE 100","ERR",1); check(has(o,"ERR NOT_HOMED"),"MOVE refused before HOME",o);
  // STOP must abort homing
  send("HOME"); spin(20000); check(pos!=p0,"HOME starts moving"); o=run_until("STOP","STOPPED",1);
  double ps=pos; spin(20000); check(has(o,"STOPPED")&&fabs(pos-ps)<30,"STOP aborts homing and the motor stops",o);
  o=run_until("STATUS?","STATE",1); check(has(o,"STATE UNHOMED"),"after aborted homing: UNHOMED",o);
  o=run_until("HOME","HOMING COMPLETE",200); check(has(o,"HOMING COMPLETE"),"HOME completes",o.substr(0,300)); std::cout<<"   "<<o.substr(o.find("Measured"),40);
  o=run_until("STATUS?","STATE",1); check(has(o,"STATE READY"),"STATUS? says READY",o);
  o=run_until("MOVE 125","DONE",60); check(has(o,"ACK")&&has(o,"DONE")&&fabs(mm()-125)<0.5,"MOVE 125: ACK then DONE, within 0.5 mm",o); std::cout<<"   true pos "<<mm()<<" mm\n";
  o=run_until("MOVEPCT 50","DONE",60); check(has(o,"DONE")&&fabs(mm()-124)<1.0,"MOVEPCT 50 -> half of measured travel",o); std::cout<<"   true pos "<<mm()<<" mm\n";
  o=run_until("MOVEPCT 0","DONE",60); check(has(o,"DONE")&&mm()>2.5&&mm()<3.7,"MOVEPCT 0 stops 3 mm inside the base switch",o); std::cout<<"   true pos "<<mm()<<" mm\n";
  o=run_until("MOVEPCT 100","DONE",60); check(has(o,"DONE")&&mm()<246&&mm()>243,"MOVEPCT 100 stops 3 mm inside the end switch, no trip",o); std::cout<<"   true pos "<<mm()<<" mm\n";
  send("MOVEPCT 0"); spin(3000); drain(); o=run_until("STOP","STOPPED",1); ps=pos; spin(5000);
  check(has(o,"STOPPED")&&fabs(pos-ps)<30,"STOP during a move: stops and holds",o); o=drain(); check(!has(o,"DONE"),"no DONE after a cancelled move");
  blocked=true; o=run_until("MOVE 100","FAULT",10); check(has(o,"FAULT: Stall"),"stall -> FAULT",o.substr(0,150));
  blocked=false; o=run_until("MOVE 100","ERR",1); check(has(o,"ERR"),"after FAULT, MOVE refused",o);
  o=run_until("HOME","HOMING COMPLETE",200); check(has(o,"HOMING COMPLETE"),"HOME recovers from FAULT",o.substr(0,200));
  o=run_until("MOVE 100","DONE",60); check(has(o,"DONE"),"normal moves work again",o);
  pos=MAX_TRIG+10; spin(100); o=drain(); check(has(o,"FAULT: MAX limit"),"switch trip during motion -> FAULT",o);
  std::cout<<(fails?"\nFAILED\n":"\nALL PASSED\n"); return fails;
}
