// BLDC Motor Music - the audio interrupt
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Runs at 44.1 kHz and also runs the Teensy core's audio updates (AudioStream)
// for the USB audio input. Each sample:
//   hall angle -> synth voices -> audio input -> rotation -> MIDI EQ, limiter -> PWM.
// If one pass takes longer than GUARD_US, the weakest voice is released
// to help the interrupt keep up (counted as "shed" in stat).

#include "motor_music.h"

IntervalTimer audioTimer;
volatile uint32_t isr_n = 0, isr_max = 0, isr_late = 0, isr_ticks = 0, shed_n = 0, guard_hold = 0;
volatile uint64_t isr_sum = 0;
float isr_hz_meas = 0.0f;
float guard_rel = 0.0f;
volatile float out_lvl01 = 0.0f;
volatile float declick = 0.0f;

// volume 0-100: 100 = imax as calculated with R_EFF, -10 dB per 25, 0 = mute
float levelDb(float v) { return 40.0f * (v - 100.0f) / 100.0f; }
static float levelLin(float v) { return (v <= 0.0f) ? 0.0f : volRef() * powf(10.0f, levelDb(v) / 20.0f); }

void updateLevels() {
  master = levelLin(mvol_v);
  amaster = levelLin(avol_v);
  ostr = ostr_v / vbus;
  d_bias = bias_v / vbus;
  cal_bias = calb_v / vbus;
}

void applyEngine() {
  audioTimer.end();
  for (int i = 0; i < MAX_VOICES; i++) { vox[i].stage = OFF; vox[i].amp = 0; vox[i].keydown = 0; vox[i].out = 0; }
  declick = 0.0f;
  guard_rel = expf(-1.0f / (GUARD_REL_S * FS));
  applyPwm();
  dtUpdate();
  updateEnv();
  updateLimit();
  applyFilters();
  rb_r = rb_w; a_playing = false;
  audioTimer.begin(audioISR, ISR_US);
}

static void guardCheck(uint32_t cyc, int nvoice) {
  if (guard_hold) guard_hold--;
  static uint32_t guard_cool = 0;
  if (guard_cool) guard_cool--;
  if (cyc > (uint32_t)(cpu_cyc_us * GUARD_US) && nvoice > 0) {
    guard_hold = (uint32_t)(GUARD_HOLD_S * FS);
    if (!guard_cool) {
      int k = -1; float e0 = 1e9f;
      for (int i = 0; i < MAX_VOICES; i++) {
        const Voice &v = vox[i];
        if (v.stage == OFF || v.shed) continue;
        float e = v.amp * v.vel;
        if (e < e0) { e0 = e; k = i; }
      }
      if (k >= 0) { vox[k].stage = REL; vox[k].keydown = 0; vox[k].rel_rate = guard_rel; vox[k].shed = 1; shed_n++; }
      guard_cool = (uint32_t)(GUARD_REL_S * 3.0f * FS);
    }
  }
}

void audioISR() {
  uint32_t ts[CPU_NST + 1];
  ts[0] = ARM_DWT_CYCCNT;
  isr_ticks++;
  if ((isr_ticks & (AUDIO_BLOCK_SAMPLES - 1)) == 0) AudioOutputMotor::kick();
  const bool cm = cal_mode;

  // 1 hall sensors -> rotor angle
  hallUpdate();
  ts[1] = ARM_DWT_CYCCNT;

  // 2 synth
  int nvoice = 0;
  const float mix = synthRender(nvoice);
  ts[2] = ARM_DWT_CYCCNT;

  // 3 USB audio (with the audio EQ)
  float xa = 0.0f;
  bool a_have = false;
  if (!cm) xa = audioRender(a_have);
  else a_active = false;
  ts[3] = ARM_DWT_CYCCNT;

  // 4 rotating field
  if (!cm) spinUpdate(nvoice > 0 || a_active || sync_play);
  ts[4] = ARM_DWT_CYCCNT;

  // 5 MIDI EQ, mix, limiter
  FieldOut f;
  spinField(f, nvoice > 0 || a_have);
  float us = master * mix;
  {
    static uint32_t m_tail = 0;
    if (nvoice > 0) m_tail = (uint32_t)(EQ_TAIL_S * FS);
    if (m_tail) {
      us = eqRun(run_m, us);
      if (nvoice == 0 && --m_tail == 0) eqClear(run_m);
    }
  }
  const float y = us + amaster * xa;
  const float utone = cm ? 0.0f : driveLimit(y, f);
  {
    static float oe = 0.0f;
    float ao = fabsf(utone);
    oe = (ao > oe) ? oe + (ao - oe) * 0.01f : oe * 0.99995f;
    if ((isr_ticks & 63) == 0) {
      float od = (20.0f * log10f(oe / volRef() + 1e-6f) + 40.0f) / 40.0f;
      out_lvl01 = constrain(od, 0.0f, 1.0f);
    }
  }
  ts[5] = ARM_DWT_CYCCNT;

  // 6 phases, dead time, PWM, EN
  driveOutput(utone, f, cm || nvoice > 0 || fld_running || a_have);
  ts[6] = ARM_DWT_CYCCNT;

  const uint32_t cyc = ts[6] - ts[0];
  guardCheck(cyc, nvoice);
  isr_n++; isr_sum += cyc;
  if (cyc > isr_max) isr_max = cyc;
  if (cyc > (uint32_t)(cpu_cyc_us * (1.0e6f / FS))) isr_late++;
  if (cpu_on) cpuRecord(cyc, nvoice, fld_running, ts);
}
