#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);
WebServer server(80);

const char* AP_SSID = "RobotDog";
const char* AP_PASS = "12345678";   // 至少 8 碼

// ---------- 設定 ----------
constexpr uint16_t SERVO_MIN_US = 600;
constexpr uint16_t SERVO_MAX_US = 2300;
constexpr uint16_t HYST_US      = 5;

const uint8_t legs[4][3] = {
  {0, 1, 2},
  {4, 5, 6},
  {8, 9, 10},
  {12, 13, 14},
};
const float HOME[4][3] = {
  {95, 160, 160},
  {90, 20, 20},
  {90, 20, 22},
  {95, 160, 160}
};

float    curAngle[16];
uint16_t lastUs[16] = {0};

// ---------- 步態參數（A 左前、B 右前、C 左後、D 右後）----------
constexpr float S = 30;
const float   CENTER[4]   = {95, 90, 90, 95};
const int8_t  DIR[4]      = {-1, +1, -1, +1};
const float   STAND[4][2] = {{130,110},{45,65},{55,75},{130,130}};
const float   LIFT[4]     = {-40, +40, +40, -40};
const uint8_t ORDER[4]    = {0, 3, 1, 2};                  // A D B C
constexpr uint16_t READY_MS = 800;
constexpr uint16_t LIFT_MS  = 200;
constexpr uint16_t SWING_MS = 400;

// ---------- 轉向參數 ----------
constexpr float    TURN_S        = 20;
constexpr uint8_t  TURN_CYCLES   = 2;
constexpr uint16_t TURN_BODY_MS  = 1200;
constexpr uint16_t TURN_LIFT_MS  = 300;
constexpr uint16_t TURN_SWING_MS = 600;

// ---------- 底層輸出（含遲滯）----------
void setServoEx(uint8_t ch, float angle, bool force) {
  angle = constrain(angle, 0.0f, 180.0f);
  curAngle[ch] = angle;

  uint16_t us = SERVO_MIN_US + (uint16_t)(angle * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f + 0.5f);

  if (!force && lastUs[ch] != 0 && abs((int)us - (int)lastUs[ch]) < HYST_US) return;

  lastUs[ch] = us;
  pwm.writeMicroseconds(ch, us);
}

// ---------- 腳緩動 ----------
constexpr uint32_t UPDATE_MS = 20;

struct Pose  { float a[3]; uint16_t ms; };
struct Seq   { const Pose* p; uint8_t n; };
struct Phase { Seq s[4]; };

void setPose(Pose& p, uint8_t l, float x, float tibiaOff, uint16_t ms);   // 手動原型

constexpr uint8_t PHASES        = 5;
constexpr uint8_t TURN_PHASES   = 10;
constexpr uint8_t SETTLE_PHASES = 4;
Pose  gaitPose[PHASES][4][3];
Phase gaitPhase[PHASES];
Pose  turnPose[TURN_PHASES][4][3];
Phase turnPhase[TURN_PHASES];
Pose  standPose[4][1];
Phase standPhase[1];
Pose  settlePose[SETTLE_PHASES][2];
Phase settlePhase[SETTLE_PHASES];

float hipAng(uint8_t l, float x) { return CENTER[l] + DIR[l] * x; }

void setPose(Pose& p, uint8_t l, float x, float tibiaOff, uint16_t ms) {
  p.a[0] = hipAng(l, x);
  p.a[1] = STAND[l][0];
  p.a[2] = STAND[l][1] + tibiaOff;
  p.ms = ms;
}

// ---- 走路 ----
void buildGait() {
  float x[4];
  x[ORDER[0]] = -S;
  x[ORDER[1]] = -S / 3;
  x[ORDER[2]] =  S / 3;
  x[ORDER[3]] =  S;

  // 階段 1：從任何已知位置（HOME 或站姿）一步到「站姿 + 錯開髖位置」
  for (uint8_t l = 0; l < 4; l++) {
    setPose(gaitPose[0][l][0], l, x[l], 0, READY_MS);
    gaitPhase[0].s[l] = {gaitPose[0][l], 1};
  }

  for (uint8_t k = 0; k < 4; k++) {
    uint8_t s = ORDER[k];
    for (uint8_t l = 0; l < 4; l++)
      setPose(gaitPose[k + 1][l][0], l, x[l], (l == s) ? LIFT[l] : 0, LIFT_MS);

    for (uint8_t l = 0; l < 4; l++) {
      if (l == s) x[l] = S;
      else        x[l] -= 2 * S / 3;
    }

    for (uint8_t l = 0; l < 4; l++) {
      Pose* q = gaitPose[k + 1][l];
      setPose(q[1], l, x[l], (l == s) ? LIFT[l] : 0, SWING_MS);
      setPose(q[2], l, x[l], 0, LIFT_MS);
      gaitPhase[k + 1].s[l] = {q, 3};
    }
  }
}

// ---- 站姿 ----
constexpr uint16_t STAND_MS = 1200;
void buildStand() {
  for (uint8_t l = 0; l < 4; l++) {
    setPose(standPose[l][0], l, 0, 0, STAND_MS);
    standPhase[0].s[l] = {standPose[l], 1};
  }
}

// ---- 收腳：依序抬腿、髖回中立、放下 ----
constexpr uint16_t SETTLE_LIFT_MS = 500;
constexpr uint16_t SETTLE_DOWN_MS = 300;
void buildSettle() {
  for (uint8_t k = 0; k < SETTLE_PHASES; k++) {
    uint8_t s = ORDER[k];
    for (uint8_t l = 0; l < 4; l++) settlePhase[k].s[l] = {nullptr, 0};
    setPose(settlePose[k][0], s, 0, LIFT[s], SETTLE_LIFT_MS);
    setPose(settlePose[k][1], s, 0, 0,       SETTLE_DOWN_MS);
    settlePhase[k].s[s] = {settlePose[k], 2};
  }
}

// ---- 轉向 ----
void buildTurn(int8_t dir) {
  float xs[4], xe[4];
  for (uint8_t l = 0; l < 4; l++) {
    int8_t side = (l == 0 || l == 2) ? 1 : -1;
    xs[l] = dir * side * TURN_S;
    xe[l] = -xs[l];
  }

  for (uint8_t p = 0; p < TURN_PHASES; p++)
    for (uint8_t l = 0; l < 4; l++) turnPhase[p].s[l] = {nullptr, 0};

  for (uint8_t l = 0; l < 4; l++) {                       // 階段 1
    setPose(turnPose[0][l][0], l, xe[l], 0, TURN_BODY_MS);
    turnPhase[0].s[l] = {turnPose[0][l], 1};
  }
  for (uint8_t k = 0; k < 4; k++) {                       // 階段 2~5
    uint8_t s = ORDER[k];
    Pose* q = turnPose[k + 1][s];
    setPose(q[0], s, xe[s], LIFT[s], TURN_LIFT_MS);
    setPose(q[1], s, xs[s], LIFT[s], TURN_SWING_MS);
    setPose(q[2], s, xs[s], 0,       TURN_LIFT_MS);
    turnPhase[k + 1].s[s] = {q, 3};
  }
  for (uint8_t l = 0; l < 4; l++) {                       // 階段 6
    setPose(turnPose[5][l][0], l, xe[l], 0, TURN_BODY_MS);
    turnPhase[5].s[l] = {turnPose[5][l], 1};
  }
  for (uint8_t k = 0; k < 4; k++) {                       // 階段 7~10
    uint8_t s = ORDER[k];
    Pose* q = turnPose[6 + k][s];
    setPose(q[0], s, xe[s], LIFT[s], TURN_LIFT_MS);
    setPose(q[1], s, 0,     LIFT[s], TURN_SWING_MS);
    setPose(q[2], s, 0,     0,       TURN_LIFT_MS);
    turnPhase[6 + k].s[s] = {q, 3};
  }
}

struct LegMotion {
  const Pose* seq = nullptr;
  uint8_t  len = 0, idx = 0, leg = 0;
  uint32_t t0 = 0;
  float    from[3] = {0, 0, 0};
  bool     active = false;

  void beginStep() {
    t0 = millis();
    for (int i = 0; i < 3; i++) from[i] = curAngle[legs[leg][i]];
  }

  void start(const Pose* s, uint8_t n) {
    for (int i = 0; i < 3; i++) {
      if (isnan(curAngle[legs[leg][i]])) {
        Serial.println("位置未知，請先執行 r (reload)");
        return;
      }
    }
    seq = s; len = n; idx = 0; active = true;
    beginStep();
  }

  void update() {
    if (!active) return;

    const Pose& p = seq[idx];
    float u = p.ms ? (millis() - t0) / (float)p.ms : 1.0f;
    if (u > 1.0f) u = 1.0f;
    float e = u * u * (3.0f - 2.0f * u);
    bool done = (u >= 1.0f);

    for (int i = 0; i < 3; i++)
      setServoEx(legs[leg][i], from[i] + (p.a[i] - from[i]) * e, done);

    if (done) {
      if (++idx >= len) active = false;
      else beginStep();
    }
  }
};

LegMotion motionLeg[4];

const Phase* curTable = gaitPhase;
uint8_t phase       = 0;
bool    looping     = false;
bool    turning     = false;
uint8_t turnLeft    = 0;
int8_t  pendingTurn = 0;

void resetState() {
  phase = 0;
  looping = false;
  turning = false;
  pendingTurn = 0;
  curTable = gaitPhase;
  for (int i = 0; i < 4; i++) motionLeg[i].active = false;
}

bool busyStandOrSettle() {
  return phase != 0 && (curTable == standPhase || curTable == settlePhase);
}

// ---------- 指令 ----------
void stopAll() {
  Serial.println("clearing");
  resetState();
  for (int ch = 0; ch < 16; ch++) {
    pwm.setPWM(ch, 0, 4096);
    curAngle[ch] = NAN;
    lastUs[ch] = 0;
  }
  Serial.println("cleared");
}

void reload() {
  Serial.println("reloading");
  resetState();
  for (int leg = 0; leg < 3; leg += 2) {
    for (int j = 0; j < 3; j++) {
      setServoEx(legs[leg][j],     HOME[leg][j],     true);
      setServoEx(legs[leg + 1][j], HOME[leg + 1][j], true);
      delay(j < 2 ? 500 : 2000);
    }
  }
  Serial.println("reload done");
}

// ---------- 階段控制 ----------
void startPhase(uint8_t p) {
  phase = p;
  Serial.print(turning ? "轉向階段 " : "階段 "); Serial.println(p);
  const Phase& ph = curTable[p - 1];
  for (uint8_t i = 0; i < 4; i++)
    if (ph.s[i].p) motionLeg[i].start(ph.s[i].p, ph.s[i].n);
  for (uint8_t i = 0; i < 4; i++)
    if (ph.s[i].p && !motionLeg[i].active) {   // 位置未知等失敗
      resetState();
      return;
    }
}

void beginStand() {
  Serial.println("站立");
  looping = false; turning = false; pendingTurn = 0;
  curTable = standPhase;
  startPhase(1);
}

void beginTurn(int8_t dir) {
  Serial.println(dir > 0 ? "開始右轉" : "開始左轉");
  buildTurn(dir);
  curTable = turnPhase;
  turning = true;
  turnLeft = TURN_CYCLES;
  startPhase(1);
}

void updatePhase() {
  if (phase == 0) return;
  for (uint8_t i = 0; i < 4; i++)
    if (motionLeg[i].active) return;   // 本階段還有腳在動

  // 收腳 → 站立
  if (curTable == settlePhase) {
    if (phase < SETTLE_PHASES) startPhase(phase + 1);
    else beginStand();
    return;
  }

  // 站立完成：回到可接任何指令的閒置狀態
  if (curTable == standPhase) {
    Serial.println("站立完成");
    resetState();
    return;
  }

  // 轉向
  if (turning) {
    if (phase == 6) {
      if (turnLeft > 1) { turnLeft--; startPhase(2); }
      else startPhase(7);
    } else if (phase < TURN_PHASES) {
      startPhase(phase + 1);
    } else {
      Serial.println("轉向完成");
      resetState();
    }
    return;
  }

  // 走路
  if (phase < PHASES) {
    startPhase(phase + 1);
  } else if (looping) {
    startPhase(2);
  } else {
    Serial.println("走路已停止");
    phase = 0;
    if (pendingTurn) {
      int8_t d = pendingTurn;
      pendingTurn = 0;
      beginTurn(d);
    }
  }
}

// e：立刻中斷，依序收腳後站立
void softStop() {
  if (busyStandOrSettle()) { Serial.println("收腳 / 站立中，請稍候"); return; }
  if (phase == 0) { Serial.println("已經停止"); return; }
  Serial.println("中斷動作，依序收腳後站立");
  for (int i = 0; i < 4; i++) motionLeg[i].active = false;
  looping = false; turning = false; pendingTurn = 0;
  curTable = settlePhase;
  startPhase(1);
}

// t：閒置時直接站立；走路 / 轉向中則先收腳再站立
void stand() {
  if (busyStandOrSettle()) { Serial.println("收腳 / 站立中，請稍候"); return; }
  if (phase != 0) { softStop(); return; }
  beginStand();
}

void startWalkLoop() {
  if (busyStandOrSettle()) { Serial.println("收腳 / 站立中，請稍候"); return; }
  if (turning)             { Serial.println("轉向中，請稍候"); return; }
  looping = true;
  pendingTurn = 0;
  curTable = gaitPhase;
  startPhase(1);
}

// dir：+1 右轉，-1 左轉
void turnCommand(int8_t dir) {
  if (busyStandOrSettle()) { Serial.println("收腳 / 站立中，請稍候"); return; }
  if (turning || pendingTurn) { Serial.println("轉向中，請稍候"); return; }
  if (phase != 0) {
    looping = false;
    pendingTurn = dir;
    Serial.println("先停止走路，走完這圈後轉向");
  } else {
    beginTurn(dir);
  }
}

void action3() {
  Serial.println("動作 3：尚未設定");
}

// ---------- 指令入口 ----------
void handleCommand(char c) {
  switch (c) {
    case 's': stopAll();        break;
    case 'r': reload();         break;
    case 'w': startWalkLoop();  break;
    case 'e': softStop();       break;
    case 'a': turnCommand(-1);  break;
    case 'd': turnCommand(+1);  break;
    case 't': stand();          break;
    case 'c': action3();        break;
    default:  Serial.println("指令：s 斷電 / r 歸位 / t 站立 / w 走 / e 收腳站立 / a 左轉 / d 右轉"); break;
  }
}

// ---------- 手機網頁 ----------
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:sans-serif;text-align:center;margin:0;padding:16px}
button{width:90%;height:70px;margin:8px 0;font-size:24px;border:0;border-radius:12px;color:#fff}
.half{width:44%;margin:8px 1%}
.r{background:#2980b9}.w{background:#27ae60}.e{background:#e67e22}.s{background:#c0392b}.t{background:#8e44ad}
#st{margin-top:12px;color:#555}
</style></head><body>
<h2>Robot Control</h2>
<button class="r" onclick="cmd('r')">歸位 (r)</button>
<button class="r" onclick="cmd('t')">站立 (t)</button>
<button class="w" onclick="cmd('w')">開始走 (w)</button>
<div>
<button class="t half" onclick="cmd('a')">左轉 (a)</button><button class="t half" onclick="cmd('d')">右轉 (d)</button>
</div>
<button class="e" onclick="cmd('e')">收腳站立 (e)</button>
<button class="s" onclick="cmd('s')">立刻斷電 (s)</button>
<div id="st"></div>
<script>
function cmd(c){
  document.getElementById('st').innerText='送出中...';
  fetch('/cmd?c='+c).then(r=>r.text()).then(t=>{document.getElementById('st').innerText=t})
  .catch(()=>{document.getElementById('st').innerText='連線失敗'});
}
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html; charset=utf-8", PAGE); }

void handleCmd() {
  if (!server.hasArg("c") || server.arg("c").length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "missing c");
    return;
  }
  char c = server.arg("c")[0];
  handleCommand(c);
  server.send(200, "text/plain; charset=utf-8", String("OK: ") + c);
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  if (!pwm.begin()) {
    Serial.println("PCA9685 Not Found.");
    while (true) delay(1000);
  } else {
    Serial.println("PCA9685 Found.");
  }
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  for (uint8_t i = 0; i < 4; i++) motionLeg[i].leg = i;

  for (int ch = 0; ch < 16; ch++) {
    pwm.setPWM(ch, 0, 4096);
    curAngle[ch] = NAN;
    lastUs[ch] = 0;
  }
  Serial.println("Stop all output");

  buildGait();
  buildStand();
  buildSettle();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();
}

void loop() {
  server.handleClient();

  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') handleCommand(c);
  }

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= UPDATE_MS) {
    last = now;
    for (uint8_t i = 0; i < 4; i++) motionLeg[i].update();
    updatePhase();
  }
}