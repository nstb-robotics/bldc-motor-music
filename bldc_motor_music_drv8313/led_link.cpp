// BLDC Motor Music - link to the LED board (Serial1, 115200 baud)
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Sends every MIDI note (3 bytes) and, while audio plays, a 12-band spectrum
// frame about 60 times per second. With sjoin 1, Motor 1 (role 1) analyses
// L+R and sends one 24-band frame for both motors (low notes left, high right).

#include "motor_music.h"

int sp_onset = 0, sp_harm = 0;
float sp_buf[SPN];
volatile uint32_t sp_w = 0;
static float sp_re[SPN], sp_im[SPN], sp_win[SPN], sp_cos[SPN / 2], sp_sin[SPN / 2];
int spec_mode = 2;
float sp_gain = 0.0f, sp_range = 30.0f, sp_tilt = 3.0f;
int sp_join = 1, sp_split = 60;
static uint8_t sj_lv[24], sj_nt[24];
uint8_t led_txmem[1024];
uint32_t led_drop = 0;
volatile bool sp_joined = false;
uint32_t sp_frames = 0, sp_drop = 0;
float sp_ms_max = 0.0f;
uint8_t sp_lv[12];
static uint8_t sp_nt[12];

void specInit() {
  for (int i = 0; i < SPN; i++) sp_win[i] = 0.5f - 0.5f * cosf(TWO_PI_F * i / SPN);
  for (int i = 0; i < SPN / 2; i++) { sp_cos[i] = cosf(TWO_PI_F * i / SPN); sp_sin[i] = -sinf(TWO_PI_F * i / SPN); }
}

static void fftRun() {
  int j = 0;
  for (int i = 1; i < SPN; i++) {
    int bit = SPN >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) { float t = sp_re[i]; sp_re[i] = sp_re[j]; sp_re[j] = t; t = sp_im[i]; sp_im[i] = sp_im[j]; sp_im[j] = t; }
  }
  for (int len = 2; len <= SPN; len <<= 1) {
    int half = len >> 1, step = SPN / len;
    for (int i = 0; i < SPN; i += len) {
      for (int k = 0; k < half; k++) {
        float wr = sp_cos[k * step], wi = sp_sin[k * step];
        int a = i + k, b = a + half;
        float xr = sp_re[b] * wr - sp_im[b] * wi, xi = sp_re[b] * wi + sp_im[b] * wr;
        sp_re[b] = sp_re[a] - xr; sp_im[b] = sp_im[a] - xi;
        sp_re[a] += xr; sp_im[a] += xi;
      }
    }
  }
}

static uint8_t spLevel(int i, float db) {
  float lv = constrain((db + sp_range) / sp_range, 0.0f, 1.0f);
  if (sp_onset) {
    static float slow[24];
    static bool init = false;
    if (!init) { for (int k = 0; k < 24; k++) slow[k] = -200.0f; init = true; }
    float fl = -sp_range;
    if (slow[i] < fl) slow[i] = fl;
    float d = (db > fl) ? db : fl;
    float rise = d - slow[i];
    slow[i] += (d - slow[i]) * 0.5f;
    if (d < slow[i]) slow[i] = d;
    lv *= constrain(rise / 6.0f, 0.0f, 1.0f);
  }
  return (uint8_t)(lv * 127.0f + 0.5f);
}

static void spHarm(float *En) {
  static const int HS[4] = {12, 19, 24, 28};
  for (int n = 0; n < 128; n++) {
    float e = En[n];
    if (e <= 0.0f) continue;
    for (int h = 0; h < 4; h++) {
      int m = n + HS[h];
      if (m > 127) break;
      if (En[m] < 0.7f * e) En[m] *= 0.1f;
    }
  }
}

void specTask() {
  static uint32_t last = 0;
  uint32_t now = micros();
  sp_joined = sp_join && role == 1 && (spec_mode == 1 || spec_mode == 2);
  if (spec_mode == 0 || (!a_playing && tone_hz <= 0.0f) || now - last < 16667) return;
  last = now;
  uint32_t c0 = ARM_DWT_CYCCNT;
  uint32_t w = sp_w;
  if (spec_mode == 3) {
    const int NM = 1024;
    float ss = 0.0f, pk = 0.0f;
    for (int i = 0; i < NM; i++) { float v = sp_buf[(w + SPN - NM + i) % SPN]; ss += v * v; float a = fabsf(v); if (a > pk) pk = a; }
    float rdb = 10.0f * log10f(2.0f * ss / NM + 1e-12f) + sp_gain;
    float pdb = 20.0f * log10f(pk + 1e-6f) + sp_gain;
    for (int k = 0; k < 12; k++) { sp_lv[k] = 0; sp_nt[k] = 0; }
    sp_lv[0] = (uint8_t)(constrain((rdb + sp_range) / sp_range, 0.0f, 1.0f) * 127.0f + 0.5f);
    sp_lv[1] = (uint8_t)(constrain((pdb + sp_range) / sp_range, 0.0f, 1.0f) * 127.0f + 0.5f);
  } else {
  for (int i = 0; i < SPN; i++) { sp_re[i] = sp_buf[(w + i) % SPN] * sp_win[i]; sp_im[i] = 0.0f; }
  fftRun();
  const float fsd = FS * 0.5f, df = fsd / SPN;
  const float ref = 1.0f / ((SPN / 4.0f) * (SPN / 4.0f));
  float E[12] = {0};
  if (sp_joined) {
    if (spec_mode == 1) {
      const float f0 = 60.0f, f1 = 8000.0f;
      for (int k = 0; k < 24; k++) {
        float lo = f0 * powf(f1 / f0, k / 24.0f), hi = f0 * powf(f1 / f0, (k + 1) / 24.0f);
        int b0 = (int)(lo / df + 0.5f), b1 = (int)(hi / df + 0.5f);
        if (b1 <= b0) b1 = b0 + 1;
        float e = 0.0f;
        for (int b = b0; b < b1 && b < SPN / 2; b++) e += sp_re[b] * sp_re[b] + sp_im[b] * sp_im[b];
        float db = 10.0f * log10f(e * ref + 1e-12f) + sp_tilt * log2f(sqrtf(lo * hi) / 1000.0f) + sp_gain;
        sj_lv[k] = spLevel(k, db);
        sj_nt[k] = 0;
      }
    } else {
      static float Ej[128];
      for (int n = 0; n < 128; n++) Ej[n] = 0.0f;
      int b0 = (int)(100.0f / df), b1 = (int)(5000.0f / df);
      for (int b = b0; b <= b1; b++) {
        int n = (int)lroundf(69.0f + 12.0f * log2f(b * df / 440.0f));
        if (n < 0 || n > 127) continue;
        Ej[n] += sp_re[b] * sp_re[b] + sp_im[b] * sp_im[b];
      }
      if (sp_harm) spHarm(Ej);
      for (int h = 0; h < 2; h++) {
        for (int pc = 0; pc < 12; pc++) {
          float e = 0.0f, best = -1.0f; int bn = pc + (h ? 72 : 48);
          for (int n = pc; n < 128; n += 12) {
            if ((n >= sp_split) != (h == 1)) continue;
            e += Ej[n];
            if (Ej[n] > best) { best = Ej[n]; bn = n; }
          }
          float db = 10.0f * log10f(e * ref + 1e-12f) + sp_gain;
          sj_lv[h * 12 + pc] = spLevel(h * 12 + pc, db);
          sj_nt[h * 12 + pc] = (uint8_t)bn;
        }
      }
    }
    for (int k = 0; k < 12; k++) { sp_lv[k] = sj_lv[12 + k]; sp_nt[k] = sj_nt[12 + k]; }
    if (Serial1.availableForWrite() >= 50) {
      Serial1.write((uint8_t)0xF5);
      Serial1.write((uint8_t)spec_mode);
      Serial1.write(sj_lv, 24);
      Serial1.write(sj_nt, 24);
      sp_frames++;
    } else sp_drop++;
    float ms = (float)(ARM_DWT_CYCCNT - c0) / (cpu_cyc_us * 1000.0f);
    if (ms > sp_ms_max) sp_ms_max = ms;
    return;
  }
  if (spec_mode == 1) {
    const float f0 = 60.0f, f1 = 8000.0f;
    for (int k = 0; k < 12; k++) {
      float lo = f0 * powf(f1 / f0, k / 12.0f), hi = f0 * powf(f1 / f0, (k + 1) / 12.0f);
      int b0 = (int)(lo / df + 0.5f), b1 = (int)(hi / df + 0.5f);
      if (b1 <= b0) b1 = b0 + 1;
      float e = 0.0f;
      for (int b = b0; b < b1 && b < SPN / 2; b++) e += sp_re[b] * sp_re[b] + sp_im[b] * sp_im[b];
      float db = 10.0f * log10f(e * ref + 1e-12f) + sp_tilt * log2f(sqrtf(lo * hi) / 1000.0f) + sp_gain;
      sp_lv[k] = spLevel(k, db);
      sp_nt[k] = 0;
    }
  } else {
    static float En[128];
    for (int n = 0; n < 128; n++) En[n] = 0.0f;
    int b0 = (int)(100.0f / df), b1 = (int)(5000.0f / df);
    for (int b = b0; b <= b1; b++) {
      float f = b * df;
      int n = (int)lroundf(69.0f + 12.0f * log2f(f / 440.0f));
      if (n < 0 || n > 127) continue;
      En[n] += sp_re[b] * sp_re[b] + sp_im[b] * sp_im[b];
    }
    if (sp_harm) spHarm(En);
    for (int pc = 0; pc < 12; pc++) {
      float e = 0.0f, best = -1.0f; int bn = pc + 60;
      for (int n = pc; n < 128; n += 12) { e += En[n]; if (En[n] > best) { best = En[n]; bn = n; } }
      E[pc] = e;
      float db = 10.0f * log10f(e * ref + 1e-12f) + sp_gain;
      sp_lv[pc] = spLevel(pc, db);
      sp_nt[pc] = (uint8_t)bn;
    }
  }
  }
  if (Serial1.availableForWrite() >= 26) {
    Serial1.write((uint8_t)0xF4);
    Serial1.write((uint8_t)spec_mode);
    Serial1.write(sp_lv, 12);
    Serial1.write(sp_nt, 12);
    sp_frames++;
  } else sp_drop++;
  float ms = (float)(ARM_DWT_CYCCNT - c0) / (cpu_cyc_us * 1000.0f);
  if (ms > sp_ms_max) sp_ms_max = ms;
}

// never wait for the LED link: if the buffer is full, this LED event is skipped (sound is not affected)
void ledSend(uint8_t a, uint8_t b, uint8_t c) {
  if (Serial1.availableForWrite() < 3) { led_drop++; return; }
  uint8_t m[3] = {a, b, c};
  Serial1.write(m, 3);
}

void ledAllOff() {
  if (Serial1.availableForWrite() < 1) { led_drop++; return; }
  Serial1.write((uint8_t)0xFF);
}
