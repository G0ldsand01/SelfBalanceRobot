// =======================================================================
// == Self-Balancing Robot (ESP32 + MPU6050) - Kd-dominant, Stable + Web UI
// =======================================================================
#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Wire.h>
#include "mpu6050_simple.h"
#include "LED_ESP32.h"

/* =================== Wi-Fi =================== */
const char *STA_SSID  = "GG_TEP";
const char *STA_PASS  = "lolaflorez&";
const char *MDNS_NAME = "robotbalance";

/* =================== Pins =================== */
#define PIN_L_FWD 25
#define PIN_L_REV 26
#define PIN_R_FWD 32
#define PIN_R_REV 14
#define PIN_LED   2

// Encodeurs (A/B)
#define ENCODER_L_A 34
#define ENCODER_L_B 18
#define ENCODER_R_A 35
#define ENCODER_R_B 19

/* =================== Objets =================== */
MPU6050Simple mpu;
WebServer     server(80);
LED_ESP32     statusLed(PIN_LED, 2);

/* =================== Orientation / Direction =================== */
// Si le robot corrige dans le mauvais sens: mettre -1
const int MOTOR_DIR = +1;   // +1 ou -1 selon câblage/sens

/* =================== Filtre & PID =================== */
static float angle_deg = 0.0f;
static bool  enabled   = false;
float alpha            = 0.97f;    // complémentaire (poids gyro)

// Gains PID (Kd-dominant)
float setpoint = 0.0f;             // sera recalibré via /recalibrate
float Kp = 22.0f;
float Ki = 0.00f;                   // laisser 0.0, activer plus tard si besoin
float Kd = 0.16f;
float Kgyro = 3.0f;                 // feed-forward gyro

// Limites / zones
const float DEADZONE_ANGLE = 0.5f;  // petite zone morte
int   MIN_PWM = 25;
const int PWM_RES_BITS = 10;
const int MAX_PWM      = (1 << PWM_RES_BITS) - 1; // 1023

// Commandes “manuel” via UI
float target_angle_offset = 0.0f;   // marche avant/arrière
float turn_speed          = 0.0f;   // rotation (différentiel)

/* =================== Auto-idle =================== */
// Coupe les moteurs si angle et gyro quasi nuls un certain temps
const float IDLE_ANGLE_DEG = 0.6f;
const float IDLE_GYRO_DPS  = 2.0f;
const uint32_t IDLE_DWELL_MS = 700; // rester stable X ms
static uint32_t idleTimerStart = 0;
static bool motorsIdle = false;

/* =================== PWM =================== */
const int PWM_FREQ = 330;
const int CH_L_FWD = 0, CH_L_REV = 1;
const int CH_R_FWD = 2, CH_R_REV = 3;

/* =================== Encodeurs =================== */
volatile long encoderCountLeft  = 0;
volatile long encoderCountRight = 0;
volatile float rpmLeft = 0.0f, rpmRight = 0.0f;

portMUX_TYPE encMuxL = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encMuxR = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleEncoderLeftA() {
  bool A = digitalRead(ENCODER_L_A);
  bool B = digitalRead(ENCODER_L_B);
  portENTER_CRITICAL_ISR(&encMuxL);
  if (A == B) encoderCountLeft++; else encoderCountLeft--;
  portEXIT_CRITICAL_ISR(&encMuxL);
}
void IRAM_ATTR handleEncoderLeftB() {
  bool A = digitalRead(ENCODER_L_A);
  bool B = digitalRead(ENCODER_L_B);
  portENTER_CRITICAL_ISR(&encMuxL);
  if (A != B) encoderCountLeft++; else encoderCountLeft--;
  portEXIT_CRITICAL_ISR(&encMuxL);
}
void IRAM_ATTR handleEncoderRightA() {
  bool A = digitalRead(ENCODER_R_A);
  bool B = digitalRead(ENCODER_R_B);
  portENTER_CRITICAL_ISR(&encMuxR);
  if (A == B) encoderCountRight++; else encoderCountRight--;
  portEXIT_CRITICAL_ISR(&encMuxR);
}
void IRAM_ATTR handleEncoderRightB() {
  bool A = digitalRead(ENCODER_R_A);
  bool B = digitalRead(ENCODER_R_B);
  portENTER_CRITICAL_ISR(&encMuxR);
  if (A != B) encoderCountRight++; else encoderCountRight--;
  portEXIT_CRITICAL_ISR(&encMuxR);
}

/* =================== Gyro Bias =================== */
static float gyroBiasY = 0.0f;
static bool  biasReady = false;

void computeGyroBias(uint16_t samples = 800) {
  // Mesure au repos (tenir le robot immobile)
  double sum = 0.0;
  for (uint16_t i = 0; i < samples; ++i) {
    mpu.update();
    sum += mpu.getGyroY();
    delayMicroseconds(1000); // ~1 kHz
  }
  gyroBiasY = (float)(sum / samples);
  biasReady = true;
}

/* =================== Moteurs =================== */
inline void driveMotor(int fwd_ch, int rev_ch, int speed) {
  int s = MOTOR_DIR * speed;
  int duty = abs(s);
  if (duty > MAX_PWM) duty = MAX_PWM;

  if (s > 0) {
    ledcWrite(fwd_ch, duty);
    ledcWrite(rev_ch, 0);
  } else if (s < 0) {
    ledcWrite(fwd_ch, 0);
    ledcWrite(rev_ch, duty);
  } else {
    ledcWrite(fwd_ch, 0);
    ledcWrite(rev_ch, 0);
  }
}

/* =================== Web UI =================== */
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="fr"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>Contrôle du Robot</title>
<style>
:root{--accent:#2eff8e;--accent-red:#ff4b4b;--accent-warn:#ffc400;--text:#f1f1f1;
--bg:radial-gradient(circle at 30% 20%, #141420 0%, #090912 80%);--glass:rgba(255,255,255,0.06);
--border:rgba(255,255,255,0.12);--blur:18px}
*{box-sizing:border-box}body{margin:0;height:100vh;background:var(--bg);font-family:Segoe UI,sans-serif;color:var(--text);display:flex;align-items:center;justify-content:center}
.card{display:flex;gap:40px;background:var(--glass);border:1px solid var(--border);backdrop-filter:blur(var(--blur));
border-radius:20px;padding:30px 40px;box-shadow:0 6px 30px rgba(0,0,0,.45);width:900px;max-width:96vw}
.left{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:flex-start;padding-top:10px}
.right{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center}
h1{margin:6px 0 12px;font-size:28px;text-shadow:0 0 8px rgba(255,255,255,.06)}
#status-text{font-weight:700;padding:8px 14px;border-radius:12px}
.enabled{background:var(--accent);color:#000;box-shadow:0 0 16px var(--accent)}
.disabled{background:var(--accent-red);color:#fff;box-shadow:0 0 12px var(--accent-red)}
#toggle-btn,#recal-btn{margin:6px 0;padding:10px 22px;border-radius:10px;border:none;background:linear-gradient(135deg,#333,#444);
color:#fff;font-size:1rem;cursor:pointer;width:250px}
#recal-btn{background:linear-gradient(135deg,#a66a00,#734900)}#recal-btn:active{background:var(--accent-warn);color:#000}
#visualizer{margin-top:12px;height:80px;width:180px;background:rgba(255,255,255,.03);border-radius:10px;border:1px solid var(--border);display:flex;align-items:flex-end;justify-content:center}
#robot{width:30px;height:70px;background:var(--accent);border-radius:4px;border:2px solid #fff;transform-origin:bottom center;transition:transform .09s linear;box-shadow:0 0 16px var(--accent)}
.stats{margin-top:14px;text-align:center;font-size:.95rem}
.controls{display:grid;grid-template-columns:repeat(3,90px);gap:12px;justify-content:center;align-items:center}
.ctrl-btn{width:90px;height:90px;border-radius:12px;border:none;background:linear-gradient(145deg,#222,#333);color:#fff;font-size:1.6rem;cursor:pointer;transition:all .08s;box-shadow:0 6px 20px rgba(0,0,0,.45)}
.ctrl-btn:active{transform:scale(.96);background:var(--accent);color:#000;box-shadow:0 0 22px var(--accent)}
.small{font-size:.9rem;color:#ddd;margin-top:6px}
</style></head><body>
<div class="card">
  <div class="left">
    <h1>Contrôle du Robot</h1>
    <p>Statut: <span id="status-text" class="disabled">Désactivé</span></p>
    <button id="toggle-btn">Activer/Désactiver Stabilisation</button>
    <button id="recal-btn">Recalibrer Setpoint (Tenir droit!)</button>
    <div id="visualizer"><div id="robot"></div></div>
    <p id="tilt" class="small">--</p>
    <div class="stats">
      <p>Angle: <span id="angle">--</span>°</p>
      <p>Encodeur Gauche: <span id="encL">--</span> (<span id="vL">--</span> rpm)</p>
      <p>Encodeur Droit: <span id="encR">--</span> (<span id="vR">--</span> rpm)</p>
    </div>
  </div>
  <div class="right">
    <div class="controls">
      <div></div><button class="ctrl-btn" id="btn-up">↑</button><div></div>
      <button class="ctrl-btn" id="btn-up-left">←</button>
      <button class="ctrl-btn" id="btn-stop">⨉</button>
      <button class="ctrl-btn" id="btn-up-right">→</button>
      <div></div><button class="ctrl-btn" id="btn-down">↓</button><div></div>
    </div>
  </div>
</div>
<script>
const statusText=document.getElementById('status-text');
const robotEl=document.getElementById('robot');
const tiltEl=document.getElementById('tilt');
function updateUIEnabled(e){ if(e){ statusText.textContent='Activé'; statusText.className='enabled'; } else { statusText.textContent='Désactivé'; statusText.className='disabled'; } }
async function toggle(){ try{ const r=await fetch('/toggle'); const j=await r.json(); updateUIEnabled(j.enabled);}catch(e){} }
async function recalibrate(){
  if(!confirm('Tenir le robot parfaitement droit, puis OK.')) return;
  try{ const r=await fetch('/recalibrate'); const j=await r.json();
    if(j.success){ alert('Recalibrage ok. Nouveau setpoint: '+(+j.newSetpoint).toFixed(2)+'°'); }
    else { alert('Erreur recalibrage.'); }
  }catch(e){ alert('Erreur de connexion.'); }
}
document.getElementById('toggle-btn').addEventListener('click', toggle);
document.getElementById('recal-btn').addEventListener('click', recalibrate);

function bind(btnId, dir){
  const el=document.getElementById(btnId);
  if(!el) return;
  el.addEventListener('mousedown', ()=>fetch(`/move?dir=${dir}`));
  el.addEventListener('touchstart',(ev)=>{ev.preventDefault(); fetch(`/move?dir=${dir}`);},{passive:false});
  ['mouseup','mouseleave','touchend','touchcancel'].forEach(evt=>el.addEventListener(evt,()=>fetch('/move?dir=stop')));
}
bind('btn-up','forward'); bind('btn-down','backward'); bind('btn-stop','stop');
bind('btn-up-left','left'); bind('btn-up-right','right');

setInterval(async ()=>{
  try{
    const r=await fetch('/status'); const j=await r.json();
    updateUIEnabled(j.enabled);
    document.getElementById('angle').textContent=(+j.angle).toFixed(1);
    document.getElementById('encL').textContent=j.encL;
    document.getElementById('encR').textContent=j.encR;
    document.getElementById('vL').textContent=(+j.vL).toFixed(1);
    document.getElementById('vR').textContent=(+j.vR).toFixed(1);
    if(j.enabled){
      const a=Math.max(-45,Math.min(45,j.angle));
      robotEl.style.transform=`rotate(${a}deg)`;
      const an=(+j.angle).toFixed(1);
      tiltEl.textContent=(j.angle>2?`Penché avant (${an}°)`: j.angle<-2?`Penché arrière (${an}°)`:`Stable (${an}°)`);
    } else {
      robotEl.style.transform='rotate(0deg)';
      tiltEl.textContent='--';
    }
  }catch(e){ tiltEl.textContent='Erreur'; updateUIEnabled(false); }
},200);
</script></body></html>
)rawliteral";

/* =================== États PID =================== */
static float pe = 0.0f;
static float prev_pwm = 0.0f;
static float integral = 0.0f;

/* =================== Web Endpoints =================== */
void handleRoot(){ server.send(200, "text/html", INDEX_HTML); }

void handleToggle(){
  enabled = !enabled;
  motorsIdle = false;
  if (!enabled){
    target_angle_offset = 0.0f;
    turn_speed = 0.0f;
  } else {
    pe = 0.0f; prev_pwm = 0.0f; integral = 0.0f;
    idleTimerStart = millis();
  }
  String response = "{\"enabled\":" + String(enabled ? "true" : "false") + "}";
  server.send(200, "application/json", response);
}

void handleMove(){
  if (!server.hasArg("dir")) { server.send(400, "text/plain", "Missing direction"); return; }
  String dir = server.arg("dir");
  motorsIdle = false; // sortir de l'idle dès qu'un ordre arrive
  if      (dir == "forward")  { target_angle_offset =  +6.0f; turn_speed = 0.0f; }
  else if (dir == "backward") { target_angle_offset =  -6.0f; turn_speed = 0.0f; }
  else if (dir == "left")     { turn_speed = -380.0f; target_angle_offset = 0.0f; }
  else if (dir == "right")    { turn_speed = +380.0f; target_angle_offset = 0.0f; }
  else if (dir == "stop")     { target_angle_offset = 0.0f;  turn_speed = 0.0f; }
  server.send(200, "text/plain", "OK");
}

void handleStatus(){
  long l, r;
  portENTER_CRITICAL(&encMuxL); l = encoderCountLeft;  portEXIT_CRITICAL(&encMuxL);
  portENTER_CRITICAL(&encMuxR); r = encoderCountRight; portEXIT_CRITICAL(&encMuxR);

  String response = "{";
  response += "\"enabled\":" + String(enabled ? "true" : "false") + ",";
  response += "\"angle\":"   + String(angle_deg, 2) + ",";
  response += "\"encL\":"    + String(l) + ",";
  response += "\"encR\":"    + String(r) + ",";
  response += "\"vL\":"      + String(rpmLeft, 2) + ",";
  response += "\"vR\":"      + String(rpmRight, 2);
  response += "}";
  server.send(200, "application/json", response);
}

void handleRecalibrate(){
  // recalibrage setpoint + recalcul biais gyro
  setpoint = angle_deg;
  computeGyroBias(600);
  target_angle_offset = 0.0f;
  turn_speed = 0.0f;
  pe = 0.0f; prev_pwm = 0.0f; integral = 0.0f;
  motorsIdle = false;
  Serial.printf("--- RECALIBRAGE --- Setpoint: %.2f / GyroBiasY: %.3f dps\n", setpoint, gyroBiasY);
  String response = "{\"success\":true,\"newSetpoint\":" + String(setpoint, 2) + "}";
  server.send(200, "application/json", response);
}

/* =================== Setup =================== */
void setup(){
  Serial.begin(115200);

  // PWM moteurs
  ledcSetup(CH_L_FWD, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_L_REV, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_R_FWD, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(CH_R_REV, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_L_FWD, CH_L_FWD);
  ledcAttachPin(PIN_L_REV, CH_L_REV);
  ledcAttachPin(PIN_R_FWD, CH_R_FWD);
  ledcAttachPin(PIN_R_REV, CH_R_REV);

  // I2C + MPU6050
  Wire.begin();
  Wire.setClock(400000L);
  if (!mpu.begin(Wire)) {
    Serial.println("Erreur: MPU6050 non trouvé !");
    while (1) delay(100);
  }
  delay(120);
  mpu.calibrate(400);
  computeGyroBias(800); // mesure au repos

  // Encodeurs
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), handleEncoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), handleEncoderLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), handleEncoderRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_B), handleEncoderRightB, CHANGE);

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\nIP: "); Serial.println(WiFi.localIP());
  if (MDNS.begin(MDNS_NAME)) MDNS.addService("http","tcp",80);

  // Serveur
  server.on("/",           HTTP_GET, handleRoot);
  server.on("/toggle",     HTTP_GET, handleToggle);
  server.on("/move",       HTTP_GET, handleMove);
  server.on("/status",     HTTP_GET, handleStatus);
  server.on("/recalibrate",HTTP_GET, handleRecalibrate);
  server.begin();

  // Démarrage: setpoint initial = angle actuel (robot tenu droit)
  mpu.update();
  angle_deg = 0.0f;
  setpoint  = 0.0f;
  idleTimerStart = millis();

  Serial.println("\n--- Robot prêt (PID Kd-dominant) ---");
}

/* =================== Loop =================== */
void loop(){
  server.handleClient();

  // ===== Encodeurs -> RPM =====
  static uint32_t lastEncoderCalc = 0;
  const uint32_t encIntervalMs = 200;
  static long prevL = 0, prevR = 0;

  if (millis() - lastEncoderCalc >= encIntervalMs){
    long curL, curR;
    portENTER_CRITICAL(&encMuxL); curL = encoderCountLeft;  portEXIT_CRITICAL(&encMuxL);
    portENTER_CRITICAL(&encMuxR); curR = encoderCountRight; portEXIT_CRITICAL(&encMuxR);

    long dL = curL - prevL;
    long dR = curR - prevR;
    prevL = curL; prevR = curR;

    const float TICKS_PER_REV = 400.0f; // adapter à tes encodeurs
    rpmLeft  = (dL / TICKS_PER_REV) * (60000.0f / (float)encIntervalMs);
    rpmRight = (dR / TICKS_PER_REV) * (60000.0f / (float)encIntervalMs);

    lastEncoderCalc = millis();
  }

  // ===== Boucle PID (1 kHz) =====
  static uint32_t lastRun = 0;
  const uint32_t interval_us = 1000;
  uint32_t now = micros();
  if (now - lastRun >= interval_us){
    lastRun = now;

    mpu.update();

    // Gyro lissé + bias
    static float gyroFiltered = 0.0f;
    float rawGyro = mpu.getGyroY();             // dps (adapter l’axe si nécessaire)
    if (biasReady) rawGyro -= gyroBiasY;
    gyroFiltered = 0.6f * gyroFiltered + 0.4f * rawGyro;
    float gyroRate = gyroFiltered;

    // Angle acc (pitch) — adapter au besoin si orientation différente
    float accAngle = atan2f(mpu.getAccY(), mpu.getAccZ()) * 180.0f / PI;

    // dt sécurisé
    static uint32_t last_dt_us = 0;
    uint32_t t = micros();
    float dt = (last_dt_us == 0) ? 0.001f : (t - last_dt_us) / 1e6f;
    if (dt <= 0.0f || dt > 0.02f) dt = 0.001f;
    last_dt_us = t;

    // Filtre complémentaire
    angle_deg = alpha * (angle_deg + gyroRate * dt) + (1.0f - alpha) * accAngle;

    // Auto-idle: s’arrête quand parfaitement stable pendant X ms
    if (enabled && fabsf(angle_deg) < IDLE_ANGLE_DEG && fabsf(gyroRate) < IDLE_GYRO_DPS &&
        fabsf(target_angle_offset) < 0.01f && fabsf(turn_speed) < 1.0f) {
      if (!motorsIdle) {
        if (millis() - idleTimerStart > IDLE_DWELL_MS) motorsIdle = true;
      }
    } else {
      motorsIdle = false;
      idleTimerStart = millis();
    }

    // PID
    float effective_setpoint = setpoint + target_angle_offset;
    float e = effective_setpoint - angle_deg;

    if (fabsf(e) < DEADZONE_ANGLE) e = 0.0f;

    // Intégrale + anti-windup
    integral += e * dt;
    if (Ki > 0.0f){
      const float I_LIM = 100.0f;
      if (integral >  I_LIM) integral =  I_LIM;
      if (integral < -I_LIM) integral = -I_LIM;
    }

    float d = (e - pe) / dt;
    pe = e;

    float u = Kp * e + Ki * integral + Kd * d + Kgyro * gyroRate;

    // Lissage PWM (EMA)
    const float a_old = 0.30f;           // poids ancien
    const float a_new = 1.0f - a_old;    // poids nouveau
    float smoothed_pwm = a_old * prev_pwm + a_new * u;
    prev_pwm = smoothed_pwm;

    int base_pwm = (int)roundf(constrain(smoothed_pwm, -MAX_PWM, MAX_PWM));

    // Backlash jump + seuil min
    const int BACKLASH_JUMP = 40;
    if (abs(base_pwm) < MIN_PWM) {
      base_pwm = 0;
    } else if (abs(base_pwm) < BACKLASH_JUMP) {
      base_pwm = (base_pwm > 0) ? BACKLASH_JUMP : -BACKLASH_JUMP;
    }

    // Différentiel de rotation
    int pwm_L = base_pwm + (int)turn_speed;
    int pwm_R = base_pwm - (int)turn_speed;

    // Failsafe chute
    if (!enabled || fabsf(angle_deg) > 45.0f){
      pwm_L = 0; pwm_R = 0;
      pe = 0.0f; prev_pwm = 0.0f; integral = 0.0f;
      motorsIdle = false;
      idleTimerStart = millis();
    }

    // Auto-idle moteurs coupés (mais prêt à réagir)
    if (enabled && motorsIdle){
      pwm_L = 0; pwm_R = 0;
    }

    driveMotor(CH_L_FWD, CH_L_REV, pwm_L);
    driveMotor(CH_R_FWD, CH_R_REV, pwm_R);
  }
}
