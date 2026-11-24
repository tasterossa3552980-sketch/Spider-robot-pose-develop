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
constexpr uint16_t HYST_US      = 5;   // 遲滯門檻

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
uint16_t lastUs[16] = {0};   // 上次實際輸出的脈寬，0 = 尚未輸出

// ---------- 底層輸出（含遲滯）----------
void setServoEx(uint8_t ch, float angle, bool force) {
  angle = constrain(angle, 0.0f, 180.0f);
  curAngle[ch] = angle;

  uint16_t us = SERVO_MIN_US + (uint16_t)(angle * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f + 0.5f);

  if (!force && lastUs[ch] != 0 && abs((int)us - (int)lastUs[ch]) < HYST_US) return;

  lastUs[ch] = us;
  pwm.writeMicroseconds(ch, us);
}

void setServo(uint8_t ch, float angle) {
  setServoEx(ch, angle, false);
}

// ---------- 腳緩動 ----------
constexpr uint32_t UPDATE_MS = 20;   // 配合 50Hz

struct Pose { float a[3]; uint16_t ms; };

const Pose Stand_A[] = {
  {{ 95, 160, 160}, 400},
  {{ 95, 130, 110}, 400},
}

const Pose Stand_B[] = {
  {{90, 20, 20},, 400},
  {{90,  45,  65}, 400},
}

const Pose Stand_C[] = {
  {{90, 20, 22}, 400},
  {{90,  55,  75}, 400},
}

const Pose Stand_D[] = {
  {{95, 160, 160}, 400},
  {{ 95, 130, 130}, 400},
}

// ===== 第一組：從 reload 出發的第一輪 =====
const Pose WALK_A[] = {
  {{ 95, 160, 160}, 400},
  {{ 30, 130,  90}, 400},
  {{ 30, 130, 110}, 400},
};
const uint8_t WALK_A_LEN = sizeof(WALK_A) / sizeof(WALK_A[0]);

const Pose WALK_B[] = {
  {{ 90,  20,  20}, 400},
  {{150,  45,  90}, 400},
  {{150,  45,  65}, 400},
};
const uint8_t WALK_B_LEN = sizeof(WALK_B) / sizeof(WALK_B[0]);

const Pose WALK_C[] = {
  {{ 90,  20,  20}, 400},
  {{ 48,  55,  90}, 400},
  {{ 48,  55,  75}, 400},
};
const uint8_t WALK_C_LEN = sizeof(WALK_C) / sizeof(WALK_C[0]);

const Pose WALK_D[] = {
  {{ 95, 160, 160}, 400},
  {{135, 130,  90}, 400},
  {{135, 130, 130}, 400},
};
const uint8_t WALK_D_LEN = sizeof(WALK_D) / sizeof(WALK_D[0]);

// 只動髖：femur / tibia 維持 walk 結束時的值
const Pose HIP_A[] = { {{ 30, 130, 110}, 400}, {{ 95, 130, 110}, 400} };
const Pose HIP_B[] = { {{150,  45,  65}, 400}, {{ 90,  45,  65}, 400} };
const Pose HIP_C[] = { {{ 48,  55,  75}, 400}, {{ 95,  55,  75}, 400} };
const Pose HIP_D[] = { {{135, 130, 130}, 400}, {{ 90, 130, 130}, 400} };

const Pose* const HIPS[4]     = {HIP_A, HIP_B, HIP_C, HIP_D};
const uint8_t     HIP_LENS[4] = {2, 2, 2, 2};

const Pose* const WALKS[4]     = {WALK_A, WALK_B, WALK_C, WALK_D};
const uint8_t     WALK_LENS[4] = {WALK_A_LEN, WALK_B_LEN, WALK_C_LEN, WALK_D_LEN};
const uint8_t     ORDER[4]     = {0, 1, 2, 3};   // 執行順序，例如 A C B D 寫 {0,2,1,3}

// ===== 第二組：循環用（起點 = 上一輪結尾，終點回到同一位置）=====
const Pose LOOP_A[] = { {{ 95, 130,  90}, 400}, {{ 30, 130,  90}, 400}, {{ 30, 130, 110}, 400} };
const Pose LOOP_B[] = { {{ 90,  45,  90}, 400}, {{150,  45,  90}, 400}, {{150,  45,  65}, 400} };
const Pose LOOP_C[] = { {{ 95,  55,  90}, 400}, {{ 48,  55,  90}, 400}, {{ 48,  55,  75}, 400} };
const Pose LOOP_D[] = { {{ 90, 130,  90}, 400}, {{135, 130,  90}, 400}, {{135, 130, 130}, 400} };

const Pose LOOPHIP_A[] = { {{ 95, 130, 110}, 400} };
const Pose LOOPHIP_B[] = { {{ 90,  45,  65}, 400} };
const Pose LOOPHIP_C[] = { {{ 95,  55,  75}, 400} };
const Pose LOOPHIP_D[] = { {{ 90, 130, 130}, 400} };

const Pose* const LOOPS[4]        = {LOOP_A, LOOP_B, LOOP_C, LOOP_D};
const uint8_t     LOOP_LENS[4]    = {3, 3, 3, 3};
const Pose* const LOOPHIPS[4]     = {LOOPHIP_A, LOOPHIP_B, LOOPHIP_C, LOOPHIP_D};
const uint8_t     LOOPHIP_LENS[4] = {1, 1, 1, 1};

// 目前使用的數據組
const Pose* const* curWalks    = WALKS;
const uint8_t*     curWalkLens = WALK_LENS;
const Pose* const* curHips     = HIPS;
const uint8_t*     curHipLens  = HIP_LENS;

void useFirstSet() { curWalks = WALKS; curWalkLens = WALK_LENS; curHips = HIPS;     curHipLens = HIP_LENS; }
void useLoopSet()  { curWalks = LOOPS; curWalkLens = LOOP_LENS; curHips = LOOPHIPS; curHipLens = LOOPHIP_LENS; }

struct LegMotion {
  const Pose* seq = nullptr;
  uint8_t  len = 0, idx = 0, leg = 0;
  uint32_t t0 = 0;
  uint32_t startAt = 0;          // 延遲出發時間，0 = 立刻
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
    seq = s; len = n; idx = 0; active = true; startAt = 0;
    beginStep();
  }

  void startDelayed(const Pose* s, uint8_t n, uint32_t delayMs) {
    start(s, n);
    if (active) startAt = millis() + delayMs;
  }

  void update() {
    if (!active) return;
    if (startAt) {
      if ((int32_t)(millis() - startAt) < 0) return;   // 還沒到出發時間
      startAt = 0;
      beginStep();
    }

    const Pose& p = seq[idx];
    float u = p.ms ? (millis() - t0) / (float)p.ms : 1.0f;
    if (u > 1.0f) u = 1.0f;
    float e = u * u * (3.0f - 2.0f * u);   // smoothstep
    bool done = (u >= 1.0f);

    for (int i = 0; i < 3; i++)
      setServoEx(legs[leg][i], from[i] + (p.a[i] - from[i]) * e, done);

    if (done) {
      if (++idx >= len) {
        active = false;
        Serial.print("腳 "); Serial.print(leg); Serial.println(" 完成");
      } else {
        beginStep();
      }
    }
  }
};

LegMotion motionLeg[4];
uint8_t walkStage  = 0;       // 0 = 閒置，1~4 = 正在執行 ORDER 的第幾隻
bool    looping    = false;   // 是否循環
bool    hipRunning = false;   // hipAll 是否正在跑

// ---------- 指令 ----------
void stopAll() {
  Serial.println("clearing");
  walkStage = 0;
  looping = false;
  hipRunning = false;
  useFirstSet();
  for (int i = 0; i < 4; i++) motionLeg[i].active = false;
  for (int ch = 0; ch < 16; ch++) {
    pwm.setPWM(ch, 0, 4096);
    curAngle[ch] = NAN;
    lastUs[ch] = 0;
  }
  Serial.println("cleared");
}

void reload() {
  Serial.println("reloading");
  walkStage = 0;
  looping = false;
  hipRunning = false;
  useFirstSet();
  for (int i = 0; i < 4; i++) motionLeg[i].active = false;
  for (int leg = 0; leg < 3; leg += 2) {
    for (int j = 0; j < 3; j++) {
      setServoEx(legs[leg][j],     HOME[leg][j],     true);
      setServoEx(legs[leg + 1][j], HOME[leg + 1][j], true);
      delay(j < 2 ? 500 : 2000);
    }
  }
  Serial.println("reload done");
}

void walk() {
  Serial.println("walking...");
  uint8_t first = ORDER[0];
  Serial.print("腳 "); Serial.print(first); Serial.println(" 開始");
  motionLeg[first].start(curWalks[first], curWalkLens[first]);
  if (motionLeg[first].active) walkStage = 1;
}

void hipAll() {
  Serial.println("move");
  walkStage = 0;   // 取消依序流程
  for (uint8_t i = 0; i < 4; i++)
    motionLeg[i].start(curHips[i], curHipLens[i]);
  hipRunning = motionLeg[0].active;   // start 失敗就不進入
}

void updateWalk() {
  if (walkStage == 0) return;
  if (motionLeg[ORDER[walkStage - 1]].active) return;   // 這隻還在動，等它

  if (walkStage >= 4) {
    Serial.println("walk 完成");
    walkStage = 0;
    hipAll();
    return;
  }

  uint8_t next = ORDER[walkStage];
  Serial.print("腳 "); Serial.print(next); Serial.println(" 開始");
  motionLeg[next].start(curWalks[next], curWalkLens[next]);
  walkStage = motionLeg[next].active ? walkStage + 1 : 0;
}

// hipAll 全部停下後：若循環中，切到循環組再走下一輪
void updateLoop() {
  if (!hipRunning) return;
  for (uint8_t i = 0; i < 4; i++)
    if (motionLeg[i].active) return;   // 還有腳在動，等它

  hipRunning = false;
  if (looping) {
    Serial.println("下一輪（循環組）");
    useLoopSet();
    walk();
  } else {
    Serial.println("全部完成");
  }
}

void startWalkLoop() {
  looping = true;
  useFirstSet();   // 第一輪用 reload 出發的那組
  walk();
}

void stand() {

}

void action3() {
  Serial.println("動作 3：尚未設定");
}

// ---------- 指令入口（序列埠與網頁共用）----------
void handleCommand(char c) {
  switch (c) {
    case 's': stopAll();       break;   // 立刻斷電並停止循環
    case 'r': reload();        break;
    case 'w': startWalkLoop(); break;   // 開始循環
    case 'e': looping = false; Serial.println("本輪結束後停止"); break;
    case 'c': action3();       break;
    default:  Serial.println("指令：s 斷電 / r 歸位 / w 循環走 / e 本輪後停止 / c"); break;
  }
}

// ---------- 手機網頁 ----------
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head>
<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:sans-serif;text-align:center;margin:0;padding:16px}
button{width:90%;height:70px;margin:8px 0;font-size:24px;border:0;border-radius:12px;color:#fff}
.r{background:#2980b9}.w{background:#27ae60}.e{background:#e67e22}.s{background:#c0392b}
#st{margin-top:12px;color:#555}
</style></head><body>
<h2>Robot Control</h2>
<button class="r" onclick="cmd('r')">歸位 (r)</button>
<button class="w" onclick="cmd('w')">開始走 (w)</button>
<button class="e" onclick="cmd('e')">本輪後停止 (e)</button>
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

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());   // 通常是 192.168.4.1

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();
}

void loop() {
  server.handleClient();   // 處理手機請求

  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') handleCommand(c);
  }

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= UPDATE_MS) {
    last = now;
    for (uint8_t i = 0; i < 4; i++) motionLeg[i].update();
    updateWalk();
    updateLoop();
  }
}