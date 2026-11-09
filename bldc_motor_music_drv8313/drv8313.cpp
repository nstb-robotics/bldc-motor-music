// BLDC Motor Music - output stage for SimpleFOCMini (DRV8313)
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Driver-specific part. The DRV8313 has no current sensing, so the current
// is calculated from a simple R-L model of the motor (it cannot be measured).
// The limiter keeps the calculated current under imax and the voltage inside
// the PWM range.
// A version for another driver (e.g. with shunt current sensing) replaces
// this file and config.h.

#include "motor_music.h"

volatile float pwm_top = 6800.0f;
volatile int   pwm_div = 2;
float pwm_hz = 22058.8f;
float vbus  = 12.0f;
float i_max = 2.5f;
volatile float u_lim = 0.0708f;
volatile int   dt_mode = 1;
float dt_ns = 270.0f;
volatile float dtc = 0.006f;
volatile float dti = 0.2f;
volatile int   lmode = 0;
float lim_rel_s = 0.15f;
float lmodel_hz = 1000.0f;
float vmax_set = 0.0f;
volatile float lim_rel = 0.9997f;
volatile float l_a = 0.0f;
volatile float v_max = 5.4f;
volatile float i_pk = 0.0f, v_pk = 0.0f, lim_min = 1.0f;
volatile bool  en_state = false;
static volatile float dt_ia = 0.0f, dt_ib = 0.0f, dt_ic = 0.0f;

// PWM period is an exact multiple of the ISR period (ISR rate / pwm_div).
// The PWM loads a new duty once per period, so the output sample rate is pwm_hz.
void applyPwm() {
  uint32_t div = (uint32_t)(((uint64_t)F_BUS_ACTUAL * ISR_TICKS * pwm_div + PIT_HZ / 2) / PIT_HZ);
  pwm_hz = (float)F_BUS_ACTUAL / (float)div;
  analogWriteFrequency(PIN_A, pwm_hz);
  analogWriteFrequency(PIN_B, pwm_hz);
  analogWriteFrequency(PIN_C, pwm_hz);
  pwm_top = (float)div;
}

void updateLimit() {
  u_lim = constrain(i_max * R_EFF / vbus, 0.0f, 0.45f);
  float lh = R_EFF / (TWO_PI_F * lmodel_hz);
  l_a = (1.0f / FS) / lh;
  float vm = 0.45f * vbus;
  v_max = (vmax_set > 0.0f && vmax_set < vm) ? vmax_set : vm;
  lim_rel = expf(-1.0f / (lim_rel_s * FS));
}

// dead time [ns] as a fraction of the PWM period
void dtUpdate() {
  dtc = constrain(dt_ns * 1e-9f * pwm_hz, 0.0f, 0.03f);
}

// 0 dB reference for mvol / avol: the voltage that drives imax through R_EFF
float volRef() { return i_max * R_EFF / vbus; }

void driveWake() {
  if (!en_state) { digitalWriteFast(PIN_EN, HIGH); en_state = true; }
}

static inline void pwmOut(int pin, float duty) {
  const float top = pwm_top;
  int ic = (int)lroundf(duty * top);
  if (ic < 0) ic = 0;
  if (ic > (int)top - 1) ic = (int)top - 1;
  uint32_t v16 = ((uint32_t)ic * 65536u + (uint32_t)top - 1u) / (uint32_t)top;
  if (v16 > 65535u) v16 = 65535u;
  analogWrite(pin, v16);
}

// y: requested sound voltage on the q axis (fraction of vbus). Returns the limited value.
float driveLimit(float y, const FieldOut &f) {
  const float vb = vbus;
  float V = y * vb;
  // voltage headroom left by the rotating field on the sound axis
  float vr = v_max / vb;
  float ap = 0.0f, an = 0.0f;
  float r2v = vr * vr - f.Pd * f.Pd;
  if (r2v > 0.0f) {
    float rr = sqrtf(r2v);
    ap = -f.Pq + rr; an = -f.Pq - rr;
    if (ap < 0.0f) ap = 0.0f;
    if (an > 0.0f) an = 0.0f;
  }
  // current headroom left by the rotating field
  float irot = sqrtf(f.Pd * f.Pd + f.Pq * f.Pq) * vb / R_EFF;
  float i2 = i_max * i_max - irot * irot;
  const float im = (i2 > 0.09f) ? sqrtf(i2) : 0.3f;
  const float R = R_EFF, la = l_a;
  static float i_det = 0.0f, i_out = 0.0f, aenv = 0.0f;
  i_det += la * (V - R * i_det);
  float vm2 = ((ap > -an) ? ap : -an) * vb;
  float ra = fabsf(i_det) / im, rv = (vm2 > 1e-4f) ? fabsf(V) / vm2 : 10.0f;
  float ratio = (ra > rv) ? ra : rv;
  if (lmode == 1) {
    aenv = 0.0f;
    if (ratio > LIM_KNEE) {
      float g = (LIM_KNEE + (1.0f - LIM_KNEE) * tanhf((ratio - LIM_KNEE) / (1.0f - LIM_KNEE))) / ratio;
      V *= g;
      if (g < lim_min) lim_min = g;
    }
  } else {
    aenv = (ratio > aenv) ? ratio : aenv * lim_rel;
    if (aenv > 1.0f) {
      float g = 1.0f / aenv;
      V *= g;
      if (g < lim_min) lim_min = g;
    }
  }
  // hard clamp: the modelled current may not pass imax in one sample
  float lo = R * i_out + (-im - i_out) / la, hi = R * i_out + (im - i_out) / la;
  if (lo < an * vb) lo = an * vb;
  if (hi > ap * vb) hi = ap * vb;
  if (lo > hi) lo = hi = 0.0f;
  if (V < lo) V = lo;
  if (V > hi) V = hi;
  i_out += la * (V - R * i_out);
  if (fabsf(i_out) > i_pk) i_pk = fabsf(i_out);
  if (fabsf(V) > v_pk) v_pk = fabsf(V);
  return V / vb;
}

// uq: sound voltage on the q axis. Adds the field, applies dead-time compensation, writes PWM.
void driveOutput(float uq, const FieldOut &f, bool want_on) {
  const float Ac = f.Ac - uq * f.sth;
  const float As = f.As + uq * f.cth;
  float ua = Ac;
  float ub = -0.5f * Ac + SQRT3_2 * As;
  float uc = -0.5f * Ac - SQRT3_2 * As;
  {
    float ia = dt_ia, ib = dt_ib, ic = dt_ic;
    if (en_state) {
      const float g = l_a, vb = vbus;
      ia += g * (ua * vb - R_EFF * ia);
      ib += g * (ub * vb - R_EFF * ib);
      ic += g * (uc * vb - R_EFF * ic);
    } else ia = ib = ic = 0.0f;
    dt_ia = ia; dt_ib = ib; dt_ic = ic;
    float k = dtc;
    if (k > 0.0f && dt_mode == 1) {
      float wi = 1.0f / dti;
      ua += k * tanhf(ia * wi);
      ub += k * tanhf(ib * wi);
      uc += k * tanhf(ic * wi);
    }
  }
  pwmOut(PIN_A, constrain(0.5f + ua, 0.02f, 0.98f));
  pwmOut(PIN_B, constrain(0.5f + ub, 0.02f, 0.98f));
  pwmOut(PIN_C, constrain(0.5f + uc, 0.02f, 0.98f));

  // EN on while anything plays; off 0.3 s after silence (cool, wheel turns freely)
  static uint32_t offcnt = 0;
  if (want_on) {
    offcnt = 0;
    if (!en_state) { digitalWriteFast(PIN_EN, HIGH); en_state = true; }
  } else if (en_state) {
    if (++offcnt > (uint32_t)(0.3f * FS)) { digitalWriteFast(PIN_EN, LOW); en_state = false; }
  }
}
