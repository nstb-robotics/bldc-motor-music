// BLDC Motor Music - LED visualizer
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Teensy 4.0 driving a WS2812 strip (pin 14, WS2812Serial library).
// Receives notes and spectrum frames from the motor boards:
//   Motor 1 TX (pin 1) -> RX1 (pin 0), Motor 2 TX (pin 1) -> RX2 (pin 7), 115200 baud.
// Each motor has a 12-LED "comb" (one LED per pitch class, colour = octave).
// pos1 / pos2 set where each comb starts on the strip.

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <WS2812Serial.h>

const int NUM_LED = 75;
const int LED_PIN = 14;
const int LED_ORDER = WS2812_GRB;
const int COMB = 12;
const int NMOT = 2;
const int BOARD_LED = 13;

const char* FW_VER = "led 1.0";
char crash_txt[1536];
uint16_t crash_len = 0;

class CrashBuf : public Print {
public:
  size_t write(uint8_t c) override { if (crash_len < sizeof(crash_txt) - 1) crash_txt[crash_len++] = c; return 1; }
};

byte drawingMemory[NUM_LED * 3];
DMAMEM byte displayMemory[NUM_LED * 12];
WS2812Serial leds(NUM_LED, displayMemory, drawingMemory, LED_PIN, LED_ORDER);

const uint32_t SET_MAGIC = 0x4E534C33;
const int EE_MAGIC = 0;
const int EE_DATA  = 4;

int   pos_user[NMOT] = {61, 34};
float bright = 0.50f;
float bgain  = 1.0f;
float fade   = 0.15f;
int   clo = 36;
int   chi = 84;
float hdec = 2.0f;
const float GAMMA = 2.2f;
bool  mon = false;
int   test_mode = 0;

float nlev[NMOT][128];
float nvel[NMOT][128];
float nt[NMOT][128];
uint8_t nheld[NMOT][128];
float dlev[NMOT][COMB];
float dtau[NMOT][COMB];

struct Rx { uint8_t st, d1, n; uint8_t sb[49]; };
const int SPEC_LEN = 25;
const int JOIN_LEN = 49;
const int M_HIGH = 0, M_LOW = 1;
uint32_t join_ms = 0, join_count = 0;
bool join_seen = false;
bool joinFresh() { return join_seen && (millis() - join_ms < 1000u); }
float sp_env[NMOT][COMB];
uint8_t sp_note[NMOT][COMB];
uint8_t sp_mode[NMOT];
uint32_t sp_ms[NMOT];
uint32_t sp_count[NMOT];
bool sp_seen[NMOT];
float sdec = 0.15f;
float pk_pos[NMOT];
float pk_t[NMOT];
float pk_b[NMOT];
float pfade = 0.3f;
float mhue0 = 120.0f, mhue1 = 0.0f, mcurve = 1.0f;
const uint32_t MT_MAGIC = 0x4E534C4D;
const int EE_MT_MAGIC = 80, EE_MT_DATA = 84;
struct MSet { float hue0, hue1, curve, phold, pfade; };
float phold = 0.6f;
const uint32_t SP_MAGIC = 0x4E534C53;
const int EE_SP_MAGIC = 64, EE_SP_DATA = 68;
const uint32_t FD_MAGIC = 0x4E534C46;
const int EE_FD_MAGIC = 104, EE_FD_DATA = 108;
Rx rx[NMOT];
uint32_t rx_count[NMOT];
uint32_t blink_ms = 0;

void hueToRgb(float h, float &r, float &g, float &b) {
  h = fmodf(h, 360.0f); if (h < 0) h += 360.0f;
  float x = h / 60.0f;
  int i = (int)x;
  float f = x - i;
  float q = 1.0f - f;
  switch (i) {
    case 0: r = 1; g = f; b = 0; break;
    case 1: r = q; g = 1; b = 0; break;
    case 2: r = 0; g = 1; b = f; break;
    case 3: r = 0; g = q; b = 1; break;
    case 4: r = f; g = 0; b = 1; break;
    default: r = 1; g = 0; b = q; break;
  }
}

void noteColor(int note, float &r, float &g, float &b) {
  float x = (float)(note - clo) / (float)(chi - clo);
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;
  hueToRgb(270.0f * (1.0f - x), r, g, b);
}

void drumHit(int m, uint8_t note, uint8_t vel) {
  int i = note % 12;
  float v = 0.5f + 0.5f * (vel / 127.0f);
  float tau = 0.03f;
  switch (note) {
    case 46: case 49: case 52: case 55: case 57: tau = 0.09f; break;
  }
  if (v > dlev[m][i]) { dlev[m][i] = v; dtau[m][i] = tau; }
}

void allOffMotor(int m) {
  for (int n = 0; n < 128; n++) nheld[m][n] = 0;
}

void handleMsg(int m, uint8_t st, uint8_t d1, uint8_t d2) {
  uint8_t type = st & 0xF0, ch = (st & 0x0F) + 1;
  if (mon) {
    Serial.print("m"); Serial.print(m + 1);
    Serial.print(" "); Serial.print(type, HEX);
    Serial.print(" ch"); Serial.print(ch);
    Serial.print(" "); Serial.print(d1);
    Serial.print(" "); Serial.println(d2);
  }
  if (type == 0x90 && d2 > 0) {
    if (ch == 10) { drumHit(m, d1, d2); return; }
    float v = 0.3f + 0.7f * (d2 / 127.0f);
    nvel[m][d1] = v;
    nt[m][d1] = 0.0f;
    nlev[m][d1] = v;
    nheld[m][d1] = 1;
  } else if (type == 0x80 || (type == 0x90 && d2 == 0)) {
    if (ch != 10) nheld[m][d1] = 0;
  } else if (type == 0xB0 && (d1 == 120 || d1 == 123)) {
    allOffMotor(m);
  }
}

void handleSpec(int m, const uint8_t *b) {
  if (joinFresh()) return;
  sp_mode[m] = b[0];
  for (int k = 0; k < COMB; k++) {
    float v = b[1 + k] / 127.0f;
    if (v > sp_env[m][k]) sp_env[m][k] = v;
    sp_note[m][k] = b[13 + k];
  }
  sp_ms[m] = millis();
  sp_seen[m] = true;
  sp_count[m]++;
}

void handleJoin(const uint8_t *b) {
  for (int h = 0; h < 2; h++) {
    int m = h ? M_HIGH : M_LOW;
    sp_mode[m] = b[0];
    for (int k = 0; k < COMB; k++) {
      float v = b[1 + h * COMB + k] / 127.0f;
      if (v > sp_env[m][k]) sp_env[m][k] = v;
      sp_note[m][k] = b[25 + h * COMB + k];
    }
    sp_ms[m] = millis();
    sp_seen[m] = true;
    sp_count[m]++;
  }
  join_ms = millis();
  join_seen = true;
  join_count++;
}

void pollRx(int m, HardwareSerial &port) {
  while (port.available()) {
    uint8_t b = port.read();
    rx_count[m]++;
    blink_ms = millis();
    Rx &r = rx[m];
    if (b == 0xFF) { allOffMotor(m); r.st = 0; r.n = 0; if (mon) { Serial.print("m"); Serial.print(m + 1); Serial.println(" all off"); } continue; }
    if (b & 0x80) { r.st = b; r.n = 0; continue; }
    if (!r.st) continue;
    if (r.st == 0xF4) {
      r.sb[r.n++] = b;
      if (r.n >= SPEC_LEN) { handleSpec(m, r.sb); r.st = 0; r.n = 0; }
      continue;
    }
    if (r.st == 0xF5) {
      r.sb[r.n++] = b;
      if (r.n >= JOIN_LEN) { handleJoin(r.sb); r.st = 0; r.n = 0; }
      continue;
    }
    if (r.n == 0) { r.d1 = b; r.n = 1; }
    else { handleMsg(m, r.st, r.d1, b); r.n = 0; }
  }
}

void update(float dt) {
  float kr = expf(-dt / (fade / 3.0f));
  for (int m = 0; m < NMOT; m++) {
    for (int n = 0; n < 128; n++) {
      float l = nlev[m][n];
      if (l <= 0.0f) continue;
      if (nheld[m][n]) {
        float t = (nt[m][n] += dt);
        l = nvel[m][n] * (0.5f + 0.2f * expf(-t / 0.8f) + 0.3f * expf(-t / 0.04f));
        if (hdec > 0.0f) { l *= expf(-t / hdec); if (l < 0.02f) l = 0.0f; }
      } else {
        l *= kr;
        if (l < 0.02f) l = 0.0f;
      }
      nlev[m][n] = l;
    }
    float ks = expf(-dt / sdec);
    for (int i = 0; i < COMB; i++) sp_env[m][i] *= ks;
    float pp = sp_env[m][1] * COMB - 0.5f;
    if (pp >= 0.5f && (pp > pk_pos[m] || pk_b[m] <= 0.0f)) { pk_pos[m] = pp; pk_t[m] = 0.0f; pk_b[m] = 1.0f; }
    else {
      pk_t[m] += dt;
      if (pk_t[m] > phold && pk_b[m] > 0.0f) { pk_b[m] *= expf(-dt / pfade); if (pk_b[m] < 0.05f) { pk_b[m] = 0.0f; pk_pos[m] = 0.0f; } }
    }
    for (int i = 0; i < COMB; i++) {
      if (dlev[m][i] <= 0.0f) continue;
      dlev[m][i] *= expf(-dt / dtau[m][i]);
      if (dlev[m][i] < 0.03f) dlev[m][i] = 0.0f;
    }
  }
}

uint32_t toColor(float r, float g, float b) {
  float mx = r; if (g > mx) mx = g; if (b > mx) mx = b;
  if (mx > 1.0f) { r /= mx; g /= mx; b /= mx; }
  int R = (int)(powf(r, GAMMA) * bright * 255.0f + 0.5f);
  int G = (int)(powf(g, GAMMA) * bright * 255.0f + 0.5f);
  int B = (int)(powf(b, GAMMA) * bright * 255.0f + 0.5f);
  return ((uint32_t)R << 16) | ((uint32_t)G << 8) | (uint32_t)B;
}

void render() {
  for (int i = 0; i < NUM_LED; i++) leds.setPixel(i, 0);
  for (int m = 0; m < NMOT; m++) {
    int base = pos_user[m] - 1;
    for (int pc = 0; pc < COMB; pc++) {
      int idx = base + pc;
      if (idx < 0 || idx >= NUM_LED) continue;
      float r = 0, g = 0, b = 0;
      bool spec_on = sp_seen[m] && (millis() - sp_ms[m] < 1000u);
      if (test_mode) {
        hueToRgb(pc * 30.0f, r, g, b);
        b *= bgain;
        leds.setPixel(idx, toColor(r, g, b));
        continue;
      }
      if (spec_on && sp_mode[m] == 3) {
        int other = (m + 1) % NMOT;
        bool left = pos_user[m] <= pos_user[other];
        int k = left ? pc : (COMB - 1 - pc);
        float lvl = sp_env[m][0] * COMB;
        float on = lvl - k;
        if (on > 1.0f) on = 1.0f;
        if (on < 0.0f) on = 0.0f;
        float hue = mhue0 + (mhue1 - mhue0) * powf(k / (float)(COMB - 1), mcurve);
        hueToRgb(hue, r, g, b);
        float e = on;
        int pk = (int)(pk_pos[m] + 0.5f);
        if (pk_b[m] > 0.0f && k == pk && on < 0.7f * pk_b[m]) e = 0.7f * pk_b[m];
        r *= e; g *= e; b *= e;
      } else if (spec_on) {
        if (sp_mode[m] == 2) noteColor(sp_note[m][pc], r, g, b);
        else if (joinFresh()) {
          int kk = (m == M_LOW) ? pc : COMB + pc;
          hueToRgb(270.0f * (1.0f - kk / (float)(2 * COMB - 1)), r, g, b);
        }
        else hueToRgb(270.0f * (1.0f - pc / (float)(COMB - 1)), r, g, b);
        b *= bgain;
        float cm = r; if (g > cm) cm = g; if (b > cm) cm = b;
        float e = sp_env[m][pc] / cm;
        r *= e; g *= e; b *= e;
      }
      {
        float nr = 0, ng = 0, nb = 0;
        float lmax = 0, wsum = 0;
        for (int n = pc; n < 128; n += 12) {
          float l = nlev[m][n];
          if (l <= 0.0f) continue;
          float cr, cg, cb;
          noteColor(n, cr, cg, cb);
          cb *= bgain;
          float cm = cr; if (cg > cm) cm = cg; if (cb > cm) cm = cb;
          float l2 = l * l, w = l2 * l2;
          nr += cr / cm * w; ng += cg / cm * w; nb += cb / cm * w;
          wsum += w;
          if (l > lmax) lmax = l;
        }
        if (wsum > 0.0f) {
          float cm = nr; if (ng > cm) cm = ng; if (nb > cm) cm = nb;
          float k = lmax / cm;
          nr *= k; ng *= k; nb *= k;
        }
        float w = dlev[m][pc];
        nr += w; ng += w; nb += w;
        float sm = r; if (g > sm) sm = g; if (b > sm) sm = b;
        float nm = nr; if (ng > nm) nm = ng; if (nb > nm) nm = nb;
        if (nm > sm) { r = nr; g = ng; b = nb; }
      }
      leds.setPixel(idx, toColor(r, g, b));
    }
  }
  leds.show();
}

struct Set { int pos[NMOT]; float bright, bgain; int clo, chi; float hdec; };

void save() {
  Set st;
  for (int i = 0; i < NMOT; i++) st.pos[i] = pos_user[i];
  st.bright = bright; st.bgain = bgain; st.clo = clo; st.chi = chi; st.hdec = hdec;
  EEPROM.put(EE_DATA, st);
  EEPROM.put(EE_MAGIC, SET_MAGIC);
  EEPROM.put(EE_SP_DATA, sdec);
  EEPROM.put(EE_SP_MAGIC, SP_MAGIC);
  MSet ms = {mhue0, mhue1, mcurve, phold, pfade};
  EEPROM.put(EE_MT_DATA, ms);
  EEPROM.put(EE_MT_MAGIC, MT_MAGIC);
  EEPROM.put(EE_FD_DATA, fade);
  EEPROM.put(EE_FD_MAGIC, FD_MAGIC);
  Serial.println("saved");
}

void load() {
  uint32_t ms; EEPROM.get(EE_SP_MAGIC, ms);
  if (ms == SP_MAGIC) { float v; EEPROM.get(EE_SP_DATA, v); if (v >= 0.03f && v <= 2.0f) sdec = v; }
  uint32_t mf; EEPROM.get(EE_FD_MAGIC, mf);
  if (mf == FD_MAGIC) { float v; EEPROM.get(EE_FD_DATA, v); if (v >= 0.03f && v <= 2.0f) fade = v; }
  uint32_t mm; EEPROM.get(EE_MT_MAGIC, mm);
  if (mm == MT_MAGIC) {
    MSet t; EEPROM.get(EE_MT_DATA, t);
    if (t.hue0 >= -360.0f && t.hue0 <= 720.0f) mhue0 = t.hue0;
    if (t.hue1 >= -360.0f && t.hue1 <= 720.0f) mhue1 = t.hue1;
    if (t.curve >= 0.2f && t.curve <= 5.0f) mcurve = t.curve;
    if (t.phold >= 0.0f && t.phold <= 3.0f) phold = t.phold;
    if (t.pfade >= 0.05f && t.pfade <= 2.0f) pfade = t.pfade;
  }
  uint32_t m; EEPROM.get(EE_MAGIC, m);
  if (m != SET_MAGIC) return;
  Set st; EEPROM.get(EE_DATA, st);
  for (int i = 0; i < NMOT; i++) pos_user[i] = constrain(st.pos[i], 1, NUM_LED - COMB + 1);
  bright = constrain(st.bright, 0.0f, 1.0f);
  bgain = constrain(st.bgain, 0.5f, 4.0f);
  hdec = constrain(st.hdec, 0.0f, 10.0f);
  {
    int lo = constrain(st.clo, 0, 126), hi = constrain(st.chi, 1, 127);
    if (hi > lo) { clo = lo; chi = hi; }
  }
  Serial.println("settings loaded");
}

void status() {
  Serial.print("fw="); Serial.print(FW_VER);
  Serial.print(" test="); Serial.print(test_mode);
  Serial.print(" mon="); Serial.print(mon);
  if (crash_len) Serial.print(" crash=1");
  Serial.println();
  Serial.print("  pos1="); Serial.print(pos_user[0]);
  Serial.print(" pos2="); Serial.print(pos_user[1]);
  Serial.print(" bright="); Serial.print(bright, 2);
  Serial.print(" bgain="); Serial.print(bgain, 2);
  Serial.print(" fade="); Serial.println(fade, 2);
  Serial.print("  clo="); Serial.print(clo);
  Serial.print(" chi="); Serial.print(chi);
  Serial.print(" hdec="); Serial.print(hdec, 2);
  Serial.print(" sdec="); Serial.print(sdec, 2);
  Serial.print(" phold="); Serial.print(phold, 2); Serial.print(" pfade="); Serial.print(pfade, 2);
  Serial.print(" mhue0="); Serial.print(mhue0, 0); Serial.print(" mhue1="); Serial.print(mhue1, 0); Serial.print(" mcurve="); Serial.println(mcurve, 2);
  Serial.print("  rx1="); Serial.print(rx_count[0]);
  Serial.print(" rx2="); Serial.println(rx_count[1]);
  for (int m = 0; m < NMOT; m++) {
    bool on = sp_seen[m] && (millis() - sp_ms[m] < 1000u);
    Serial.print("  m"); Serial.print(m + 1); Serial.print(on ? (sp_mode[m] == 3 ? " meter" : sp_mode[m] == 2 ? " spec:notes" : " spec:bands") : " midi");
    Serial.print(" frames="); Serial.println(sp_count[m]);
  }
  Serial.print("  join="); Serial.print(joinFresh() ? "on" : "off");
  Serial.print(" frames="); Serial.println(join_count);
}

void help() {
  Serial.println("--- led cmd: <name> <value>   (; for several) ---");
  Serial.println("pos1 <1-64>  pos2 <1-64>  (LED number of the C position, cable end = 1)");
  Serial.println("bright <0-1>  bgain <0.5-4>  fade <0.03-2 s>  clo <note>  chi <note>  hdec <0 hold, 0.1-10 s>");
  Serial.println("sdec <0.03-2 s spectrum/meter decay>  phold <0-3 s peak hold>  pfade <0.05-2 s peak fade>");
  Serial.println("mhue0 <deg outer>  mhue1 <deg center>  mcurve <0.2-5>");
  Serial.println("test 1/0  mon 1/0  save  st  crash  help  off");
}

void processCmd(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  int L = strlen(line);
  while (L > 0 && (line[L - 1] == ' ' || line[L - 1] == '\t')) line[--L] = 0;
  if (L == 0) return;
  char* sp = strchr(line, ' ');
  char* rest = nullptr;
  if (sp) { *sp = 0; rest = sp + 1; }
  bool hasVal = false; float val = 0;
  if (rest) { char* e; val = strtod(rest, &e); hasVal = (e != rest); }

  if (!strcmp(line, "st") || !strcmp(line, "status") || !strcmp(line, "s")) { status(); return; }
  if (!strcmp(line, "help") || !strcmp(line, "?")) { help(); return; }
  if (!strcmp(line, "crash")) { if (crash_len) Serial.print(crash_txt); else Serial.println("crash: no report from the last run"); return; }
  if (!strcmp(line, "save")) { save(); return; }
  if (!strcmp(line, "off")) { for (int m = 0; m < NMOT; m++) { allOffMotor(m); for (int n = 0; n < 128; n++) nlev[m][n] = 0; } Serial.println("off"); return; }
  if (!hasVal) { Serial.print("need value: "); Serial.println(line); return; }

  if      (!strcmp(line, "pos1"))   pos_user[0] = constrain((int)val, 1, NUM_LED - COMB + 1);
  else if (!strcmp(line, "pos2"))   pos_user[1] = constrain((int)val, 1, NUM_LED - COMB + 1);
  else if (!strcmp(line, "bright")) bright = constrain(val, 0.0f, 1.0f);
  else if (!strcmp(line, "bgain"))  bgain = constrain(val, 0.5f, 4.0f);
  else if (!strcmp(line, "fade"))   fade = constrain(val, 0.03f, 2.0f);
  else if (!strcmp(line, "clo")) { int v = constrain((int)val, 0, 126); if (v >= chi) { Serial.println("clo must be < chi"); return; } clo = v; }
  else if (!strcmp(line, "chi")) { int v = constrain((int)val, 1, 127); if (v <= clo) { Serial.println("chi must be > clo"); return; } chi = v; }
  else if (!strcmp(line, "hdec"))   hdec = (val <= 0.0f) ? 0.0f : constrain(val, 0.1f, 10.0f);
  else if (!strcmp(line, "mhue0"))  mhue0 = constrain(val, -360.0f, 720.0f);
  else if (!strcmp(line, "mhue1"))  mhue1 = constrain(val, -360.0f, 720.0f);
  else if (!strcmp(line, "mcurve")) mcurve = constrain(val, 0.2f, 5.0f);
  else if (!strcmp(line, "pfade"))  pfade = constrain(val, 0.05f, 2.0f);
  else if (!strcmp(line, "phold"))  phold = constrain(val, 0.0f, 3.0f);
  else if (!strcmp(line, "sdec"))   sdec = constrain(val, 0.03f, 2.0f);
  else if (!strcmp(line, "test"))   test_mode = (val != 0);
  else if (!strcmp(line, "mon"))    mon = (val != 0);
  else { Serial.print("? unknown: "); Serial.println(line); return; }
  status();
}

char cmdbuf[64];
uint8_t cmdlen = 0;
uint32_t last_us = 0;

void setup() {
  if (CrashReport) { CrashBuf cb; cb.print(CrashReport); crash_txt[crash_len] = 0; }
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  pinMode(BOARD_LED, OUTPUT);
  leds.begin();
  for (int i = 0; i < NUM_LED; i++) leds.setPixel(i, 0);
  leds.show();
  while (!Serial && millis() < 2000) {}
  if (crash_len) Serial.println("WARNING: the last run stopped with a crash (type crash to show the report)");
  load();
  help();
  status();
  last_us = micros();
}

void loop() {
  pollRx(0, Serial1);
  pollRx(1, Serial2);
  digitalWriteFast(BOARD_LED, (rx_count[0] | rx_count[1]) && millis() - blink_ms < 30u);

  uint32_t now = micros();
  if (now - last_us >= 5000u) {
    float dt = (now - last_us) * 1.0e-6f;
    last_us = now;
    update(dt);
    render();
  }

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r' || ch == ';') {
      if (cmdlen > 0) { cmdbuf[cmdlen] = 0; processCmd(cmdbuf); cmdlen = 0; }
    } else if (cmdlen < sizeof(cmdbuf) - 1) {
      cmdbuf[cmdlen++] = ch;
    }
  }
}
