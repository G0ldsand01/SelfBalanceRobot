#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>

// =================== Wi-Fi ===================
const char *SSID = "GG_TEP";
const char *PASS = "lolaflorez&";
WebServer server(80);

// =================== Moteurs ===================
#define L_FWD 25
#define L_REV 26
#define R_FWD 32
#define R_REV 14

const int PWM_BITS = 10;
const int PWM_FREQ = 1000;
const int MAX_PWM = (1 << PWM_BITS) - 1;
const int MIN_PWM = 25;

// =================== MPU & PID ===================
MPU6050 mpu;
double setpoint = 0, input, output;
double Kp = 80.0, Ki = 1.5, Kd = 1.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

float angle = 0, accAngle = 0, gyroRate = 0;
float alpha = 0.98;
unsigned long lastLoopMicros = 0;
const unsigned long LOOP_PERIOD_US = 3000; // 3 ms → ~333 Hz

bool enabled = false;

// =================== HTML ===================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="fr"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>CMK Robot PID</title>
<style>
body{margin:0;background:#0a0b0d;color:#fff;font-family:Segoe UI,Arial,sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh}
.card{background:#15171a;padding:24px;border-radius:20px;box-shadow:0 8px 30px rgba(0,0,0,.6);width:90%;max-width:440px;text-align:center}
h1{margin-top:0}
label{display:block;margin:10px 0 4px;font-weight:600}
input[type=range]{width:100%}
button{width:100%;padding:10px;border:none;border-radius:8px;background:#2eff8e;color:#000;font-weight:600;font-size:1.1rem;cursor:pointer}
.data{margin-top:10px;text-align:left;font-size:0.95rem;line-height:1.4rem;background:#1a1b20;border-radius:10px;padding:10px}
span.val{color:#2eff8e}
</style></head><body>
<div class="card">
<h1>CMK Robot</h1>
<div id="angle" style="font-size:1.3rem;margin:10px 0;">Angle: --</div>
<button id="toggle">Activer</button>
<label>Kp: <span id="KpVal">--</span></label><input id="Kp" type="range" min="0" max="1000" step="0.5">
<label>Ki: <span id="KiVal">--</span></label><input id="Ki" type="range" min="0" max="10" step="0.05">
<label>Kd: <span id="KdVal">--</span></label><input id="Kd" type="range" min="0" max="10" step="0.05">
<label>Alpha: <span id="AlVal">--</span></label><input id="Al" type="range" min="0.80" max="0.995" step="0.001">

<div class="data">
<b>Données en temps réel</b><br>
Angle: <span id="a" class="val">--</span>°<br>
PWM: <span id="p" class="val">--</span><br>
Acc: <span id="acc" class="val">--</span><br>
Gyro: <span id="g" class="val">--</span><br>
État: <span id="st" class="val">--</span>
</div>
</div>

<script>
const vars=['Kp','Ki','Kd','Al'];
async function getPID(){
 const r=await fetch('/pid');const j=await r.json();
 vars.forEach(v=>{document.getElementById(v).value=j[v];document.getElementById(v+'Val').textContent=j[v].toFixed(2);});
}
vars.forEach(v=>{
 const el=document.getElementById(v);
 el.addEventListener('input',()=>{fetch(`/pid?${v}=${el.value}`);document.getElementById(v+'Val').textContent=el.value;});
});
document.getElementById('toggle').onclick=async()=>{
 const r=await fetch('/toggle');const j=await r.json();
 document.getElementById('toggle').textContent=j.enabled?'Désactiver':'Activer';
};
setInterval(async()=>{
 try{
  const r=await fetch('/status');const j=await r.json();
  document.getElementById('angle').textContent='Angle: '+j.angle.toFixed(1)+'°';
  document.getElementById('a').textContent=j.angle.toFixed(2);
  document.getElementById('p').textContent=j.pwm;
  document.getElementById('acc').textContent=j.acc.toFixed(1);
  document.getElementById('g').textContent=j.gyro.toFixed(1);
  document.getElementById('st').textContent=j.enabled?'Actif':'Désactivé';
 }catch(e){}
},250);
getPID();
</script></body></html>
)rawliteral";

// =================== Fonctions moteur ===================
void drive(int pwmL, int pwmR) {
  int dL = abs(pwmL), dR = abs(pwmR);
  if (dL < MIN_PWM) dL = 0;
  if (dR < MIN_PWM) dR = 0;

  if (pwmL > 0) { ledcWrite(0, dL); ledcWrite(1, 0); }
  else if (pwmL < 0) { ledcWrite(0, 0); ledcWrite(1, dL); }
  else { ledcWrite(0, 0); ledcWrite(1, 0); }

  if (pwmR > 0) { ledcWrite(2, dR); ledcWrite(3, 0); }
  else if (pwmR < 0) { ledcWrite(2, 0); ledcWrite(3, dR); }
  else { ledcWrite(2, 0); ledcWrite(3, 0); }
}

// =================== Web ===================
void handleRoot(){ server.send(200,"text/html",INDEX_HTML); }
void handleToggle(){
  enabled = !enabled;
  input = output = angle = 0;
  String json = "{\"enabled\":" + String(enabled ? "true" : "false") + "}";
  server.send(200,"application/json",json);
}
void handlePID(){
  if(server.hasArg("Kp")) Kp = server.arg("Kp").toFloat();
  if(server.hasArg("Ki")) Ki = server.arg("Ki").toFloat();
  if(server.hasArg("Kd")) Kd = server.arg("Kd").toFloat();
  if(server.hasArg("Al")) alpha = server.arg("Al").toFloat();
  pid.SetTunings(Kp, Ki, Kd);
  String json="{\"Kp\":"+String(Kp)+",\"Ki\":"+String(Ki)+",\"Kd\":"+String(Kd)+",\"Al\":"+String(alpha)+"}";
  server.send(200,"application/json",json);
}
void handleStatus(){
  String json="{\"angle\":"+String(angle,2)+",\"pwm\":"+String((int)output)+",\"acc\":"+String(accAngle,1)+",\"gyro\":"+String(gyroRate,1)+",\"enabled\":"+(enabled?"true":"false")+"}";
  server.send(200,"application/json",json);
}

// =================== Setup ===================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Initialisation MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU non détecté !");
    while (1);
  }

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(5); // 5 ms → 200 Hz (PID interne)
  pid.SetOutputLimits(-255, 255);

  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcSetup(1, PWM_FREQ, PWM_BITS);
  ledcSetup(2, PWM_FREQ, PWM_BITS);
  ledcSetup(3, PWM_FREQ, PWM_BITS);
  ledcAttachPin(L_FWD, 0);
  ledcAttachPin(L_REV, 1);
  ledcAttachPin(R_FWD, 2);
  ledcAttachPin(R_REV, 3);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.printf("Connexion Wi-Fi à %s...\n", SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.printf("\n✅ Connecté ! IP: %s\n", WiFi.localIP().toString().c_str());

  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.on("/pid", handlePID);
  server.on("/status", handleStatus);
  server.begin();

  lastLoopMicros = micros();
  Serial.println("Robot prêt à 333 Hz PID loop.");
}

// =================== Loop ===================
void loop() {
  server.handleClient();

  // synchroniser à 3 ms pour stabilité PID
  if ((micros() - lastLoopMicros) < LOOP_PERIOD_US) return;
  unsigned long now = micros();
  float dt = (now - lastLoopMicros) / 1e6;
  lastLoopMicros = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accAngle = atan2(-ax, az) * 180.0 / PI;
  gyroRate = gy / 131.0;
  angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accAngle;
  input = angle;

  pid.Compute();

  if (!enabled || fabs(angle) > 45) {
    drive(0, 0);
    return;
  }

  int pwm = constrain((int)output, -255, 255);
  drive(pwm, pwm);
}
