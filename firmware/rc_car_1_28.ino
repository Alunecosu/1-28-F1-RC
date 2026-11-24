
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ===================== SETTINGS (change these) =====================
const char* WIFI_PASSWORD = "12345678";

const int PIN_STEER = 4;
const int PIN_ESC   = 5;

// Steering:
const int  STEER_CENTER_DEG = 90;         // servo center (use this to trim the wheels straight)
const int  STEER_MAX_DEG    = 30;         // max degrees left AND right from center
const bool STEER_INVERT     = false;      // set true if the car steers the wrong way

// Throttle limits in percent (100% = full ESC signal)
const int MAX_FORWARD_PCT = 100;           
const int MAX_REVERSE_PCT = 50;
const int BRAKE_PCT       = 60;           // brake strength
const int DEADZONE_PCT    = 5;            // small throttle values are treated as zero

// Timing
const unsigned long FAILSAFE_MS    = 300;   // no data for this long -> throttle 0, steering 0
const unsigned long BRAKE_HOLD_MS  = 1000;  // after this long, brake signal stops so the ESC does not start reversing
const unsigned long REVERSE_ARM_MS = 250;   // ESC must see neutral this long before it accepts reverse
// ===================================================================

const int ESC_NEUTRAL = 1500;

WebServer http(80);
WebSocketsServer ws(81);
Servo steerServo, escServo;

// Latest command from the phone
int  cmdSteer = 0, cmdThrottle = 0;
bool cmdBrake = false;
bool havePacket = false;
unsigned long lastPacketMs = 0;


bool killed = true;

// ESC logic state
bool brakeActive = false, reversing = false;
unsigned long brakeStartMs = 0, neutralSinceMs = 0;
int lastEscUs = ESC_NEUTRAL;

// ---------------------- Controller web page ----------------------
const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<title>RC car</title>
<style>
:root{--bg:#d9dfe2;--panel:#f3f5f6;--line:#c3cbcf;--ink:#161b1e;--muted:#5d6a70;
--steer:#2b3a42;--go:#1b5fd1;--brake:#f2a900;--kill:#d7263d;--ok:#1f8a4c;}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}
html,body{margin:0;height:100%;overflow:hidden;background:var(--bg);color:var(--ink);
font-family:"Bahnschrift","DIN Alternate","Roboto Condensed","Arial Narrow",system-ui,sans-serif;
user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;touch-action:none;overscroll-behavior:none;}
#app{height:100%;display:grid;gap:10px;padding:10px;
padding-left:max(10px,env(safe-area-inset-left));padding-right:max(10px,env(safe-area-inset-right));
grid-template-columns:1fr minmax(96px,20%) minmax(120px,22%);
grid-template-rows:auto 1fr;grid-template-areas:"bar bar bar" "steer brake thr";}
#bar{grid-area:bar;display:flex;align-items:center;justify-content:space-between;gap:12px;}
#status{font-size:19px;font-weight:600;display:flex;align-items:center;gap:10px;}
#status::before{content:"";width:13px;height:13px;border-radius:50%;background:var(--muted);}
#status[data-state=ok]::before{background:var(--ok);}
#status[data-state=wait]::before{background:var(--brake);}
#status[data-state=bad]::before{background:var(--kill);}
#kill{min-height:52px;padding:0 36px;border:0;border-radius:14px;background:var(--kill);color:#fff;
font:inherit;font-size:22px;font-weight:700;touch-action:none;}
body.killed #kill{background:var(--ink);}
.pad{position:relative;background:var(--panel);border:2px solid var(--line);border-radius:22px;touch-action:none;}
#steer{grid-area:steer;}
#thr{grid-area:thr;}
.track{position:absolute;}
#steer .track{left:34px;right:34px;top:0;bottom:0;}
#thr .track{top:34px;bottom:34px;left:0;right:0;}
#steer .track::before{content:"";position:absolute;left:0;right:0;top:50%;height:4px;margin-top:-2px;border-radius:2px;background:var(--line);}
#steer .track::after{content:"";position:absolute;left:50%;top:50%;width:3px;height:28px;margin:-14px 0 0 -1.5px;background:var(--line);}
#thr .track::before{content:"";position:absolute;top:0;bottom:0;left:50%;width:4px;margin-left:-2px;border-radius:2px;background:var(--line);}
#thr .track::after{content:"";position:absolute;top:50%;left:50%;height:3px;width:28px;margin:-1.5px 0 0 -14px;background:var(--line);}
.lbl{position:absolute;font-size:16px;color:var(--muted);}
#steer .a{left:14px;bottom:10px;} #steer .b{right:14px;bottom:10px;}
#thr .a{left:12px;top:8px;} #thr .b{left:12px;bottom:8px;}
.knob{position:absolute;width:64px;height:64px;margin:-32px 0 0 -32px;border-radius:50%;z-index:2;}
#sk{top:50%;left:50%;background:var(--steer);}
#tk{top:50%;left:50%;background:var(--go);}
#brake{grid-area:brake;border:0;border-radius:22px;background:var(--brake);color:var(--ink);
font:inherit;font-size:28px;font-weight:700;touch-action:none;}
#brake.on{background:#a87100;color:#fff;}
body.killed .pad,body.killed #brake{opacity:.45;}
#rotate{display:none;position:fixed;inset:0;z-index:10;background:var(--bg);align-items:center;justify-content:center;
text-align:center;font-size:24px;font-weight:600;padding:30px;}
@media (orientation:portrait){#rotate{display:flex;}}
</style>
</head>
<body>
<div id="app">
  <div id="bar">
    <div id="status" data-state="wait">Connecting...</div>
    <button id="kill">Re-arm</button>
  </div>
  <div id="steer" class="pad">
    <div class="track" id="st"><div class="knob" id="sk"></div></div>
    <span class="lbl a">Left</span><span class="lbl b">Right</span>
  </div>
  <button id="brake">Brake</button>
  <div id="thr" class="pad">
    <div class="track" id="tt"><div class="knob" id="tk"></div></div>
    <span class="lbl a">Forward</span><span class="lbl b">Reverse</span>
  </div>
</div>
<div id="rotate">Turn your phone sideways to drive</div>
<script>
(function(){
var $=function(id){return document.getElementById(id);};
var steer=0,thr=0,brake=0,killed=true,link='wait',ws=null;
var statusEl=$('status'),killBtn=$('kill');

function render(){
  var t,c;
  if(link==='open'){ if(killed){t='Stopped. Tap Re-arm to drive.';c='bad';} else {t='Ready';c='ok';} }
  else if(link==='wait'){t='Connecting...';c='wait';}
  else {t='Connection lost. Retrying...';c='bad';}
  statusEl.textContent=t; statusEl.setAttribute('data-state',c);
  killBtn.textContent=killed?'Re-arm':'Kill';
  document.body.classList.toggle('killed',killed);
}

function connect(){
  link='wait'; render();
  ws=new WebSocket('ws://'+(location.hostname||'192.168.4.1')+':81/');
  ws.onopen=function(){link='open';render();};
  ws.onclose=function(){link='lost';render();setTimeout(connect,700);};
  ws.onerror=function(){try{ws.close();}catch(e){}};
  ws.onmessage=function(e){
    if(e.data==='S,1')killed=true; else if(e.data==='S,0')killed=false;
    render();
  };
}

// Send the control state 20 times per second. The car stops if these stop arriving.
setInterval(function(){
  if(ws&&ws.readyState===1) ws.send('C,'+steer+','+thr+','+brake);
},50);

// Kill / re-arm button
killBtn.addEventListener('pointerdown',function(e){
  e.preventDefault();
  if(!ws||ws.readyState!==1)return;
  if(killed){ ws.send('R'); }
  else { killed=true; render(); ws.send('K'); }
});

// Touch pad (axis 'x' for steering, 'y' for throttle). Springs back to the center on release.
function pad(outer,track,knob,axis,onChange){
  var pid=null;
  function set(v){
    if(axis==='x') knob.style.left=((v+1)/2*100)+'%';
    else knob.style.top=((1-v)/2*100)+'%';
    onChange(Math.round(v*100));
  }
  function read(e){
    var r=track.getBoundingClientRect(),v;
    if(axis==='x') v=((e.clientX-r.left)/r.width)*2-1;
    else v=1-((e.clientY-r.top)/r.height)*2;
    set(Math.max(-1,Math.min(1,v)));
  }
  outer.addEventListener('pointerdown',function(e){e.preventDefault();pid=e.pointerId;outer.setPointerCapture(pid);read(e);});
  outer.addEventListener('pointermove',function(e){if(e.pointerId===pid)read(e);});
  function end(e){if(e.pointerId===pid){pid=null;set(0);}}
  outer.addEventListener('pointerup',end);
  outer.addEventListener('pointercancel',end);
  outer.addEventListener('lostpointercapture',end);
  set(0);
}
pad($('steer'),$('st'),$('sk'),'x',function(v){steer=v;});
pad($('thr'),$('tt'),$('tk'),'y',function(v){thr=v;});

// Brake button (hold to brake)
var b=$('brake'),bp=null;
b.addEventListener('pointerdown',function(e){e.preventDefault();bp=e.pointerId;b.setPointerCapture(bp);brake=1;b.classList.add('on');});
function bend(e){if(e.pointerId===bp){bp=null;brake=0;b.classList.remove('on');}}
b.addEventListener('pointerup',bend);
b.addEventListener('pointercancel',bend);
b.addEventListener('lostpointercapture',bend);

document.addEventListener('contextmenu',function(e){e.preventDefault();});
render();
connect();
})();
</script>
</body>
</html>
)HTML";

// ---------------------- Car control ----------------------
int steerAngle(int s) {
  if (STEER_INVERT) s = -s;
  return STEER_CENTER_DEG + (s * STEER_MAX_DEG) / 100;   // same angle both sides
}

// Returns the ESC pulse in microseconds (1000..2000, 1500 = neutral)
int escPulse(int throttle, bool brake, unsigned long now) {
  int us = ESC_NEUTRAL;
  bool isReverse = false;

  if (brake) {
    if (!brakeActive) { brakeActive = true; brakeStartMs = now; }
    if (now - brakeStartMs < BRAKE_HOLD_MS) us = ESC_NEUTRAL - BRAKE_PCT * 5;
  } else {
    brakeActive = false;
    if (throttle > DEADZONE_PCT) {
      us = ESC_NEUTRAL + constrain(throttle, 0, MAX_FORWARD_PCT) * 5;
    } else if (throttle < -DEADZONE_PCT) {
      // ESC only accepts reverse after it has been in neutral for a moment
      bool canReverse = reversing || (lastEscUs == ESC_NEUTRAL && (now - neutralSinceMs) >= REVERSE_ARM_MS);
      if (canReverse) {
        us = ESC_NEUTRAL - constrain(-throttle, 0, MAX_REVERSE_PCT) * 5;
        isReverse = true;
      }
    }
  }

  if (us == ESC_NEUTRAL && lastEscUs != ESC_NEUTRAL) neutralSinceMs = now;
  reversing = isReverse;
  lastEscUs = us;
  return us;
}

void updateOutputs() {
  unsigned long now = millis();
  bool linkOk = havePacket && (now - lastPacketMs < FAILSAFE_MS);

  int s = 0, t = 0;
  bool b = false;
  if (linkOk && !killed) {          // failsafe or kill -> throttle 0, steering 0
    s = cmdSteer; t = cmdThrottle; b = cmdBrake;
  }
  steerServo.write(steerAngle(s));
  escServo.writeMicroseconds(escPulse(t, b, now));
}

void handleMessage(const char* m) {
  if (m[0] == 'C') {                                   // control packet: C,steer,throttle,brake
    int s, t, b;
    if (sscanf(m, "C,%d,%d,%d", &s, &t, &b) == 3) {
      cmdSteer = constrain(s, -100, 100);
      cmdThrottle = constrain(t, -100, 100);
      cmdBrake = (b != 0);
      lastPacketMs = millis();
      havePacket = true;
    }
  } else if (m[0] == 'K') {                            // kill
    killed = true;
    updateOutputs();                                   // stop right now
    ws.broadcastTXT("S,1");
  } else if (m[0] == 'R') {                            // re-arm (only if throttle is at zero and no brake)
    if (abs(cmdThrottle) <= DEADZONE_PCT && !cmdBrake) killed = false;
    ws.broadcastTXT(killed ? "S,1" : "S,0");
  }
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      ws.sendTXT(num, killed ? "S,1" : "S,0");
      break;
    case WStype_DISCONNECTED:
      havePacket = false;                              // failsafe right away
      break;
    case WStype_TEXT:
      handleMessage((const char*)payload);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Outputs first: steering center, ESC neutral
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  steerServo.setPeriodHertz(50);
  escServo.setPeriodHertz(50);
  steerServo.attach(PIN_STEER, 500, 2400);
  escServo.attach(PIN_ESC, 1000, 2000);
  steerServo.write(STEER_CENTER_DEG);
  escServo.writeMicroseconds(ESC_NEUTRAL);

  // Own WiFi network. Name has a short unique ending so many cars can run together.
  char ssid[24];
  snprintf(ssid, sizeof(ssid), "RC-1-28-%04X", (unsigned)((ESP.getEfuseMac() >> 32) & 0xFFFF));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, WIFI_PASSWORD, 1, 0, 2);           // channel 1, visible, max 2 phones
  WiFi.setSleep(false);

  http.on("/", []() { http.send_P(200, "text/html", INDEX_HTML); });
  http.onNotFound([]() {
    http.sendHeader("Location", "/", true);
    http.send(302, "text/plain", "");
  });
  http.begin();

  ws.begin();
  ws.onEvent(onWsEvent);

  Serial.println();
  Serial.print("WiFi name: "); Serial.println(ssid);
  Serial.print("Password:  "); Serial.println(WIFI_PASSWORD);
  Serial.print("Open:      http://"); Serial.println(WiFi.softAPIP());
}

void loop() {
  http.handleClient();
  ws.loop();

  static unsigned long lastTick = 0;
  if (millis() - lastTick >= 20) {                     // 50 Hz output update
    lastTick = millis();
    updateOutputs();
  }
}
