// BLDC Motor Music - MIDI input and the 16-voice synth
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Sounds 1-7 (patch: epiano, fm-epiano, organ, guitar, saw, square, triangle),
// chosen per channel by program change; drums on channel 10.
// MIDI clock gives the tempo (smode 1 + tsync), note-ons give the note rate (smode 2).

#include "motor_music.h"

const float DECLICK_K = 0.98873f;   // exp(-1 / (DECLICK_S * FS)): 2 ms

float A4_HZ = 440.0f;
volatile float master = 0.02f;
float mvol_v = 72.5f;
static volatile float pr_atk_step[NALL], pr_dec_rate[NALL], pr_rel_rate[NALL], pr_mdec[NALL], pr_norm[NALL];
float dlev = 1.5f;
uint8_t ch_pre[17];
int  edit_p = 0;
bool pc_on = true;
volatile float vcurve = 1.0f;
float bend_range = 2.0f;
float lfo_hz = 5.5f;
float vdep   = 1.0f;
ChState chs[17];
bool  drum_on = true;
static uint32_t led_t = 0;
static bool led_lit = false;
bool  tsync = false;
float tbase = 120.0f;
float tgain = 2.0f;
volatile float sync_rpm = 0.0f;
volatile float cur_bpm = 0.0f;
volatile bool  sync_play = false;
static bool  sync_seen = false;
static uint32_t sync_last_ms = 0;
const int CLK_RING = 25;
const int TWIN = 12;                 // MIDI clocks used for the tempo estimate
static uint32_t clk_prev = 0;
static int   clk_n = 0;
static uint32_t clk_ts[CLK_RING];
static int   clk_w = 0;
float nbase = 3.0f;
float nk    = 5.0f;
float ntau  = 0.5f;
static float note_rate = 0.0f;
volatile float nrate_rpm = 0.0f;
static uint32_t last_onset_us = 0, nrate_us = 0;
Voice vox[MAX_VOICES];
int poly = MAX_VOICES;

const char* PRE_NAME[NALL] = {"epiano", "fm-epiano", "organ", "guitar", "saw", "square", "triangle", "-",
                              "kick", "snare", "hat", "openhat", "crash", "tom"};
const char* DRUM_NOTES[6] = {"35,36", "37-40", "42,44", "46,51,53,59", "49,52,55,57", "41,43,45,47,48,50"};

// sounds (patches). p1-p3 meaning depends on the waveform (see docs/commands.md)
uint8_t pr_wave[NALL] = {W_SINE, W_FM, W_ORGAN, W_GUITAR, W_SAW, W_PULSE, W_TRI, W_SINE,
                         W_KICK, W_NOISE, W_NOISE, W_NOISE, W_NOISE, W_KICK};
float pr_atk[NALL] = {0.001f, 0.001f, 0.01f, 0.002f, 0.001f, 0.001f, 0.001f, 0.003f,
                      0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0005f};
float pr_dec[NALL] = {2.0f, 2.5f, 1.0f, 3.0f, 1.0f, 1.0f, 1.0f, 2.0f,
                      0.15f, 0.10f, 0.03f, 0.25f, 0.8f, 0.2f};
float pr_sus[NALL] = {0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float pr_rel[NALL] = {0.15f, 0.2f, 0.08f, 0.2f, 0.03f, 0.03f, 0.03f, 0.15f,
                      0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f};
float pr_lev[NALL] = {1.0f, 0.8f, 0.7f, 0.7f, 0.5f, 0.5f, 0.8f, 1.0f,
                      2.0f, 0.35f, 0.2f, 0.2f, 0.2f, 0.9f};
float pr_p1[NALL]  = {0.4f, 2.0f, 0.6f, 4.0f, 0.0f, 0.5f, 16.0f, 0.0f,
                      60.0f, 12000.0f, 40000.0f, 40000.0f, 25000.0f, 90.0f};
float pr_p2[NALL]  = {0.4f, 0.25f, 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      0.04f, 0.0f, 0.0f, 0.0f, 0.0f, 0.06f};
float pr_p3[NALL]  = {0.5f, 1.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      2.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.6f};

static uint8_t chIdx(byte ch) { return (ch >= 1 && ch <= 16) ? ch : 1; }

static float noteToFreq(uint8_t n) { return A4_HZ * powf(2.0f, (n - 69) / 12.0f); }

void updateEnv() {
  for (int p = 0; p < NALL; p++) {
    pr_atk_step[p] = 1.0f / (pr_atk[p] * FS);
    pr_dec_rate[p] = expf(-1.0f / (pr_dec[p] * FS));
    pr_rel_rate[p] = expf(-1.0f / (pr_rel[p] * FS));
    pr_mdec[p] = (pr_p2[p] > 0.0f) ? expf(-1.0f / (pr_p2[p] * FS)) : 0.0f;
    if (pr_wave[p] == W_ORGAN) pr_norm[p] = 1.0f / (1.0f + fabsf(pr_p1[p]) + fabsf(pr_p2[p]) + fabsf(pr_p3[p]));
    else if (pr_wave[p] == W_GUITAR) pr_norm[p] = (pr_p1[p] > 1.0f) ? 1.0f / tanhf(pr_p1[p]) : 1.0f;
    else pr_norm[p] = 1.0f;
  }
}

// free voice first; otherwise the weakest one is taken. A taken voice is removed
// through the declick offset so the cut does not click.
static int allocVoice(uint8_t ch, uint8_t pre) {
  driveWake();
  const bool g = guard_hold > 0;
  int slot = -1, fr = -1; float weakest = 1e9;
  const int vl = poly;
  for (int i = 0; i < vl; i++) {
    if (vox[i].stage == OFF) { if (fr < 0) fr = i; if (!g) break; continue; }
    float e = vox[i].amp * vox[i].vel;
    if (e < weakest) { weakest = e; slot = i; }
  }
  if (fr >= 0 && (!g || slot < 0)) slot = fr;
  if (slot < 0) slot = 0;
  Voice &v = vox[slot];
  noInterrupts();
  if (v.stage != OFF) declick += v.out;
  v.stage = OFF;
  v.out = 0.0f;
  interrupts();
  v.shed = 0;
  v.ch = (ch <= 16) ? ch : 1;
  v.pre = pre;
  const ChState &cs = chs[v.ch];
  bool drum = (pre >= P_KICK);
  float am = drum ? 1.0f : cs.atk_mul, dm = drum ? 1.0f : cs.dec_mul, rm = drum ? 1.0f : cs.rel_mul;
  v.atk_step = pr_atk_step[pre] / am;
  v.dec_rate = (dm == 1.0f) ? pr_dec_rate[pre] : expf(-1.0f / (pr_dec[pre] * dm * FS));
  v.rel_rate = (rm == 1.0f) ? pr_rel_rate[pre] : expf(-1.0f / (pr_rel[pre] * rm * FS));
  v.cg = cs.gain;
  v.lfsr = 1;
  v.mphase = 0.0f; v.phase = 0.0f; v.amp = 0.0f;
  return slot;
}

static float velCurve(uint8_t vel) {
  float vn = vel / 127.0f;
  return (vcurve == 1.0f) ? vn : powf(vn, vcurve);
}

static void noteStart(uint8_t note, uint8_t vel, uint8_t pre, uint8_t ch) {
  Voice &v = vox[allocVoice(ch, (pre < NPRE) ? pre : 0)];
  v.note = note; v.freq = noteToFreq(note); v.gain = 1.0f;
  v.vel = velCurve(vel);
  v.keydown = 1;
  v.menv = pr_p1[v.pre];
  v.stage = ATK;
}

int drumPreset(uint8_t note) {
  switch (note) {
    case 35: case 36: return P_KICK;
    case 37: case 38: case 39: case 40: return P_SNARE;
    case 42: case 44: return P_HATC;
    case 46: case 51: case 53: case 59: return P_HATO;
    case 49: case 52: case 55: case 57: return P_CRASH;
    case 41: case 43: case 45: case 47: case 48: case 50: return P_TOM;
  }
  return -1;
}

static void drumStart(uint8_t note, uint8_t vel) {
  int p = drumPreset(note);
  if (p < 0) return;
  Voice &v = vox[allocVoice(10, p)];
  v.note = note;
  v.freq = (p == P_TOM) ? pr_p1[p] * powf(2.0f, (note - 41) / 12.0f) : pr_p1[p];
  v.gain = dlev;
  v.vel = velCurve(vel);
  v.keydown = 0;
  v.menv = 1.0f;
  v.stage = ATK;
}

static void noteStop(uint8_t ch, uint8_t note) {
  for (int i = 0; i < MAX_VOICES; i++) {
    Voice &v = vox[i];
    if (v.stage != OFF && v.ch == ch && v.note == note && v.keydown) {
      v.keydown = 0;
      if (!chs[ch].pedal) v.stage = REL;
    }
  }
}

static void pedalRelease(uint8_t ch) {
  chs[ch].pedal = 0;
  for (int i = 0; i < MAX_VOICES; i++) {
    Voice &v = vox[i];
    if (v.ch == ch && v.stage != OFF && v.stage != REL && !v.keydown) v.stage = REL;
  }
}

static float ccMul(uint8_t val) { return powf(2.0f, (val - 64) / 16.0f); }

static void chUpdate(uint8_t ch) {
  ChState &c = chs[ch];
  float sm = (c.bend_semi > 0.0f) ? c.bend_semi * (c.bend_raw / 8192.0f) : 0.0f;
  c.bend_ratio = powf(2.0f, sm / 12.0f);
  float v = c.vol / 100.0f, e = c.expr / 127.0f;
  float g = v * v * e * e;
  c.gain = (g > 1.0f) ? 1.0f : g;
  c.vib = (c.mod / 127.0f) * vdep * 0.0577623f;
}

static void chResetControllers(uint8_t ch) {
  ChState &c = chs[ch];
  pedalRelease(ch);
  c.bend_raw = 0; c.expr = 127; c.mod = 0; c.trem = 0.0f;
  c.rpn_msb = c.rpn_lsb = 127;
  chUpdate(ch);
}

static void chResetAll(uint8_t ch) {
  ChState &c = chs[ch];
  c.vol = 100; c.bend_semi = bend_range;
  c.atk_mul = c.dec_mul = c.rel_mul = 1.0f;
  chResetControllers(ch);
}

void chResetEvery() {
  for (int c = 0; c <= 16; c++) chResetAll(c);
}

void chSetBend(float semi) { for (int c = 0; c <= 16; c++) { chs[c].bend_semi = semi; chUpdate(c); } }
void chUpdateAll() { for (int c = 0; c <= 16; c++) chUpdate(c); }

static void releaseCh(uint8_t ch) {
  for (int i = 0; i < MAX_VOICES; i++)
    if (vox[i].stage != OFF && vox[i].ch == ch) { vox[i].keydown = 0; vox[i].stage = REL; }
}

// CC120: silence now (through the declick offset)
static void soundOffCh(uint8_t ch) {
  noInterrupts();
  for (int i = 0; i < MAX_VOICES; i++) {
    Voice &v = vox[i];
    if (v.stage != OFF && v.ch == ch) { declick += v.out; v.out = 0.0f; v.amp = 0.0f; v.keydown = 0; v.stage = OFF; }
  }
  interrupts();
}

static void releaseAll() {
  for (int i = 0; i < MAX_VOICES; i++)
    if (vox[i].stage != OFF) { vox[i].keydown = 0; vox[i].stage = REL; }
}

static void blink() { digitalWriteFast(PIN_LED, HIGH); led_lit = true; led_t = millis(); }

void noteOn(byte ch, byte note, byte vel) {
  if (vel == 0) { noteOff(ch, note, 0); return; }
  if (cpu_on) { cpu_notes++; cpu_notes_tot++; }
  uint8_t c = chIdx(ch);
  if (c == 10) {
    ledSend(0x99, note, vel);
    if (echo) {
      Serial.print("drum "); Serial.print(note); Serial.print(" v"); Serial.print(vel);
      Serial.println(drum_on ? "" : " (off)");
    }
    if (!drum_on) return;
    drumStart(note, vel);
    blink();
    return;
  }
  if (echo) {
    Serial.print("on "); Serial.print(note);
    Serial.print(" v");  Serial.print(vel);
    Serial.print(" ch"); Serial.println(c);
  }
  noteStart(note, vel, ch_pre[c], c);
  ledSend(0x90 | (c - 1), note, vel);
  uint32_t nu = micros();
  if (nu - last_onset_us > CHORD_US) {
    note_rate += 1.0f / ntau;
    last_onset_us = nu;
  }
  blink();
}

void noteOff(byte ch, byte note, byte vel) {
  uint8_t c = chIdx(ch);
  if (c == 10) return;
  if (echo) { Serial.print("off "); Serial.print(note); Serial.print(" ch"); Serial.println(c); }
  noteStop(c, note);
  ledSend(0x80 | (c - 1), note, 0);
}

static void rpnData(uint8_t ch) {
  ChState &c = chs[ch];
  if (c.rpn_msb == 0 && c.rpn_lsb == 0) {
    c.bend_semi = constrain(c.rpn_semi + c.rpn_cent / 100.0f, 0.0f, 24.0f);
    chUpdate(ch);
    if (mon) { Serial.print("bend range ch"); Serial.print(ch); Serial.print(" = "); Serial.println(c.bend_semi, 2); }
  }
}

void onControl(byte ch, byte cc, byte val) {
  uint8_t c = chIdx(ch);
  ChState &s = chs[c];
  if (mon) {
    Serial.print("cc "); Serial.print(cc);
    Serial.print("="); Serial.print(val);
    Serial.print(" ch"); Serial.println(c);
  }
  switch (cc) {
    case 1:   s.mod = val; break;
    case 7:   s.vol = val; break;
    case 11:  s.expr = val; break;
    case 92:  s.trem = val / 127.0f; break;
    case 73:  s.atk_mul = ccMul(val); break;
    case 75:  s.dec_mul = ccMul(val); break;
    case 72:  s.rel_mul = ccMul(val); break;
    case 101: s.rpn_msb = val; break;
    case 100: s.rpn_lsb = val; break;
    case 99: case 98: s.rpn_msb = s.rpn_lsb = 127; break;
    case 6:   s.rpn_semi = val; s.rpn_cent = 0; rpnData(c); break;
    case 38:  s.rpn_cent = val; rpnData(c); break;
    case 64:
      if (val >= 64) s.pedal = 1;
      else if (s.pedal) pedalRelease(c);
      break;
    case 121:
      s.vol = 100; s.atk_mul = s.dec_mul = s.rel_mul = 1.0f;
      chResetControllers(c);
      break;
    case 120:
      chResetControllers(c);
      soundOffCh(c);
      ledSend(0xB0 | (c - 1), 120, 0);
      break;
    case 123:
      chResetControllers(c);
      releaseCh(c);
      ledSend(0xB0 | (c - 1), 123, 0);
      break;
  }
  chUpdate(c);
}

// General MIDI program number -> sound (patch)
static uint8_t gmToPreset(uint8_t prog) {
  if (prog <= 3)  return 0;
  if (prog <= 5)  return 1;
  if (prog >= 16 && prog <= 23) return 2;
  if (prog >= 24 && prog <= 31) return 3;
  if (prog >= 32 && prog <= 39) return 6;
  if (prog == 81) return 4;
  if (prog >= 80 && prog <= 87) return 5;
  return 0;
}

void onProgram(byte ch, byte prog) {
  uint8_t c = chIdx(ch);
  uint8_t p = gmToPreset(prog);
  if (pc_on) ch_pre[c] = p;
  if (mon) {
    Serial.print("pc "); Serial.print(prog + 1);
    Serial.print(" ch"); Serial.print(c);
    Serial.print(pc_on ? " -> patch " : " (ignored, pc 0) would be patch ");
    Serial.print(p + 1); Serial.print(":"); Serial.println(PRE_NAME[p]);
  }
}

void onBend(byte ch, int b) {
  n_bend++;
  uint8_t c = chIdx(ch);
  chs[c].bend_raw = b;
  chUpdate(c);
}

static void syncMark() {
  sync_seen = true;
  sync_last_ms = millis();
}

void onClock() {
  n_clk++;
  syncMark();
  uint32_t now = micros();
  if (clk_n > 0 && (now - clk_prev) > 300000u) clk_n = 0;
  clk_prev = now;
  clk_ts[clk_w] = now;
  clk_w = (clk_w + 1) % CLK_RING;
  if (clk_n < CLK_RING) clk_n++;
  if (clk_n > TWIN) {
    uint32_t old = clk_ts[(clk_w - 1 - TWIN + 2 * CLK_RING) % CLK_RING];
    uint32_t dt = now - old;
    if (dt > 0) {
      float bpm = 60.0e6f * (float)TWIN / 24.0f / (float)dt;
      if (bpm > 20.0f && bpm < 1000.0f) cur_bpm = bpm;
    }
  }
}

void onMtc(byte data) { n_mtc++; syncMark(); }
void onSysex(uint8_t *data, unsigned int size) { n_sx++; }
void onStart() { if (mon) Serial.println("rt start"); clk_n = 0; syncMark(); }
void onContinue() { if (mon) Serial.println("rt continue"); clk_n = 0; syncMark(); }
void onStop() { if (mon) Serial.println("rt stop"); if (sync_seen) sync_last_ms = millis() - 10000u; }
void onSongPos(uint16_t beats) { if (mon) { Serial.print("rt songpos "); Serial.println(beats); } }

int pedalCount() {
  int n = 0;
  for (int c = 1; c <= 16; c++) if (chs[c].pedal) n++;
  return n;
}

// main loop: player start/stop, tempo -> rpm, note rate -> rpm, board LED
void syncTask() {
  uint32_t now = millis();
  bool p = sync_seen && (now - sync_last_ms < 300u);
  if (p != sync_play) {
    sync_play = p;
    if (mon) Serial.println(p ? "sync: play" : "sync: stop");
    if (!p) { chResetEvery(); releaseAll(); force_idle = true; ledAllOff(); }
  }
  if (!sync_play && !fld_running) cur_bpm = 0.0f;
  {
    float bpm = cur_bpm;
    sync_rpm = (bpm > 0.0f) ? constrain(const_rpm * powf(bpm / tbase, tgain), tmin_rpm, tmax_rpm) : 0.0f;
  }
  {
    uint32_t nu = micros();
    uint32_t du = nu - nrate_us;
    if (du >= 1000u) {
      nrate_us = nu;
      float dts = (float)du * 1.0e-6f;
      note_rate *= expf(-dts / ntau);
      nrate_rpm = constrain(nbase + nk * note_rate, nbase, tmax_rpm);
    }
  }
  if (mon && now - mon_t >= 1000u) {
    mon_t = now;
    bool nr = (smode == 2 && note_rate > 0.05f);
    if (n_clk || n_mtc || n_sx || n_bend || nr) {
      Serial.print("rx/s clk="); Serial.print(n_clk);
      Serial.print(" mtc="); Serial.print(n_mtc);
      Serial.print(" sysex="); Serial.print(n_sx);
      Serial.print(" bend="); Serial.print(n_bend);
      Serial.print(" bpm="); Serial.print(cur_bpm, 1);
      Serial.print(" target="); Serial.print(target_rpm, 1);
      Serial.print(" play="); Serial.print(sync_play);
      Serial.print(" pedal_ch="); Serial.println(pedalCount());
    }
    n_clk = n_mtc = n_sx = n_bend = 0;
  }
  if (led_lit && now - led_t >= 30u) { digitalWriteFast(PIN_LED, LOW); led_lit = false; }
}

void allOff() {
  for (int i = 0; i < MAX_VOICES; i++) { vox[i].stage = OFF; vox[i].amp = 0; vox[i].keydown = 0; vox[i].out = 0; }
  declick = 0.0f;
  chResetEvery();
  target_rpm = 0.0f;
  ledAllOff();
}

// ISR stage: envelopes and waveforms of all voices
float synthRender(int &nvoice) {
  const float ifs = 1.0f / FS;
  static float lfo_ph = 0.0f;
  lfo_ph += lfo_hz * ifs;
  if (lfo_ph >= 1.0f) lfo_ph -= 1.0f;
  const float lfo = fsin(TWO_PI_F * lfo_ph);

  float mix = 0.0f;
  int nv = 0;
  for (int i = 0; i < MAX_VOICES; i++) {
    Voice &v = vox[i];
    if (v.stage == OFF) continue;

    const uint8_t pp = v.pre;
    const float sl = pr_sus[pp];
    switch (v.stage) {
      case ATK:
        v.amp += v.atk_step;
        if (v.amp >= 1.0f) { v.amp = 1.0f; v.stage = DECAY; }
        break;
      case DECAY:
        v.amp = sl + (v.amp - sl) * v.dec_rate;
        if (v.amp < ((pp >= P_KICK) ? 0.01f : 0.0005f) && sl < 0.0005f) { v.stage = OFF; v.amp = 0; v.out = 0; continue; }
        break;
      case REL:
        v.amp *= v.rel_rate;
        if (v.amp < 0.0005f) { v.stage = OFF; v.amp = 0; v.out = 0; continue; }
        break;
    }

    const ChState &cs = chs[v.ch];
    v.cg += (cs.gain - v.cg) * 0.005f;

    nv++;
    float sv;
    const uint8_t wv = pr_wave[pp];
    if (wv == W_KICK) {
      float f = v.freq * (1.0f + pr_p3[pp] * v.menv);
      v.menv *= pr_mdec[pp];
      v.phase += f * ifs;
      if (v.phase >= 1.0f) v.phase -= 1.0f;
      sv = 1.0f - 4.0f * fabsf(v.phase - 0.5f);
    } else if (wv == W_NOISE) {
      v.phase += v.freq * ifs;
      while (v.phase >= 1.0f) {
        v.phase -= 1.0f;
        uint16_t r = v.lfsr;
        uint16_t fb = (r ^ (r >> 1)) & 1u;
        v.lfsr = (r >> 1) | (fb << 14);
      }
      sv = (v.lfsr & 1u) ? 1.0f : -1.0f;
    } else {
      const float finc = v.freq * cs.bend_ratio * (1.0f + cs.vib * lfo) * ifs;
      v.phase += TWO_PI_F * finc;
      if (v.phase > TWO_PI_F) v.phase -= TWO_PI_F;
      const float ph = v.phase;
      const float t = ph * (1.0f / TWO_PI_F);
      switch (wv) {
        case W_FM: {
          v.mphase += TWO_PI_F * finc * pr_p3[pp];
          if (v.mphase > TWO_PI_F) v.mphase -= TWO_PI_F;
          float idx = v.menv;
          v.menv = idx * pr_mdec[pp];
          sv = fsin(ph + idx * fsin(v.mphase));
          break;
        }
        case W_ORGAN: {
          float s1 = fsin(ph), c1 = fcos(ph);
          float s2 = 2.0f * s1 * c1, c2 = 1.0f - 2.0f * s1 * s1;
          float s3 = s1 * (3.0f - 4.0f * s1 * s1);
          float s4 = 2.0f * s2 * c2;
          sv = (s1 + pr_p1[pp] * s2 + pr_p2[pp] * s3 + pr_p3[pp] * s4) * pr_norm[pp];
          break;
        }
        case W_GUITAR: {
          float s = fsin(ph);
          float k = pr_p1[pp];
          sv = (k > 1.0f) ? tanhf(k * s) * pr_norm[pp] : s;
          break;
        }
        case W_SAW: {             // PolyBLEP: band-limited edges
          float x = 2.0f * t - 1.0f;
          float dt = finc;
          if (t < dt) { float u = t / dt; x -= u + u - u * u - 1.0f; }
          else if (t > 1.0f - dt) { float u = (t - 1.0f) / dt; x -= u * u + u + u + 1.0f; }
          sv = x;
          break;
        }
        case W_PULSE: {           // PolyBLEP, duty p1
          float d = pr_p1[pp];
          float x = (t < d) ? 1.0f : -1.0f;
          float dt = finc;
          float t2 = t + 1.0f - d; if (t2 >= 1.0f) t2 -= 1.0f;
          float a = 0.0f, b2 = 0.0f;
          if (t < dt) { float u = t / dt; a = u + u - u * u - 1.0f; }
          else if (t > 1.0f - dt) { float u = (t - 1.0f) / dt; a = u * u + u + u + 1.0f; }
          if (t2 < dt) { float u = t2 / dt; b2 = u + u - u * u - 1.0f; }
          else if (t2 > 1.0f - dt) { float u = (t2 - 1.0f) / dt; b2 = u * u + u + u + 1.0f; }
          x += a - b2;
          float pk = 2.0f * ((d > 0.5f) ? d : (1.0f - d));
          sv = (x - (2.0f * d - 1.0f)) / pk;
          break;
        }
        case W_TRI: {             // p1 >= 2: stepped (chiptune)
          float x = 1.0f - 4.0f * fabsf(t - 0.5f);
          float n = pr_p1[pp];
          if (n >= 2.0f) {
            float q = floorf((x + 1.0f) * 0.5f * n);
            if (q > n - 1.0f) q = n - 1.0f;
            x = q / (n - 1.0f) * 2.0f - 1.0f;
          }
          sv = x;
          break;
        }
        case W_SINE: {            // epiano: sine + decaying 2nd/3rd harmonics (p1, p2, p3)
          float e = v.menv;
          if (e > 0.0f) {
            float s1 = fsin(ph), c1 = fcos(ph);
            float s2 = 2.0f * s1 * c1;
            float s3 = s1 * (3.0f - 4.0f * s1 * s1);
            v.menv = e * pr_mdec[pp];
            sv = s1 + e * (s2 + pr_p3[pp] * s3);
          } else sv = fsin(ph);
          break;
        }
        default: sv = fsin(ph); break;
      }
    }
    const float tm = 1.0f - cs.trem * (0.5f - 0.5f * lfo);
    const float o = v.amp * v.vel * v.gain * v.cg * tm * pr_lev[pp] * sv;
    v.out = o;
    mix += o;
  }
  const float d = declick;
  if (d != 0.0f) {
    mix += d;
    declick = (fabsf(d) > 1e-7f) ? d * DECLICK_K : 0.0f;
  }
  nvoice = nv;
  return mix;
}
