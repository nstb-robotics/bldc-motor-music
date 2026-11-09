// BLDC Motor Music - hall sensors, rotor angle, cal, rotation (smode)
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Sound is injected on the q axis, so the firmware needs the rotor angle:
// hall states give it every 60 electrical degrees, and the angle in between
// is extrapolated from the speed. "cal" maps the six hall states to angles.
// Rotation is an open-loop rotating field added on top of the sound - a show gimmick.
// Without cal, a small d-axis voltage (bias) holds the rotor and sound still plays.

#include "motor_music.h"

const float OACC = 10.0f;     // rpm/s, smode 1
const float TACC = 100.0f;    // rpm/s, smode 2, 3 and tempo sync
const float SDECEL = 100.0f;  // rpm/s when stopping
const float SHOLD_S = 0.3f;   // field kept after stopping [s]
const float ORAMP_S = 0.3f;   // field fade in / out [s]

volatile uint8_t hall_state = 0;
volatile bool hall_mode = false, hall_ok = false, hall_err = false, cal_mode = false, geom_ok = false;
volatile uint32_t hall_quiet = 0;
volatile float theta_cal = 0.0f, theta_est = 0.0f, omega_est = 0.0f, meas_rpm = 0.0f;
volatile float theta_off = -0.2617994f;
float hall_theta[8];
int   hall_cnt[8];
float cal_bias = 0.05f;
volatile float d_bias = 0.02f;
float ostr_v = 0.5f, bias_v = 0.24f, calb_v = 0.6f;
static float h_center[8], h_width[8], h_bndF[8];
static uint8_t h_nextF[8], h_nextB[8];

volatile int   smode = 1;
volatile int   spin_dir = 1;
float const_rpm = 4.0f;
float hold_s = 1.0f;
float tmin_rpm = 3.0f, tmax_rpm = 40.0f;
volatile float ostr = 0.50f / 12.0f;
volatile float hfield_v = 0.5f;
volatile float target_rpm = 0.0f;
volatile float fld_theta = 0.0f, fld_speed = 0.0f, fld_amp = 0.0f, fld_lag = 0.0f;
volatile bool  fld_running = false, force_idle = false;
volatile int   resync_cnt = 0;

uint8_t readHall() {
  return (digitalReadFast(PIN_HA) << 2) | (digitalReadFast(PIN_HB) << 1) | digitalReadFast(PIN_HC);
}

static float wrapPi(float a) {
  while (a >  3.14159265f) a -= TWO_PI_F;
  while (a < -3.14159265f) a += TWO_PI_F;
  return a;
}

void buildGeometry() {
  int ord[6] = {1, 2, 3, 4, 5, 6};
  for (int i = 0; i < 6; i++) for (int j = i + 1; j < 6; j++) {
    float ai = hall_theta[ord[i]], aj = hall_theta[ord[j]];
    if (ai < 0) ai += TWO_PI_F;
    if (aj < 0) aj += TWO_PI_F;
    if (aj < ai) { int t = ord[i]; ord[i] = ord[j]; ord[j] = t; }
  }
  for (int i = 0; i < 6; i++) {
    int s = ord[i], n = ord[(i + 1) % 6];
    h_center[s] = hall_theta[s];
    h_nextF[s] = n; h_nextB[s] = ord[(i + 5) % 6];
    h_bndF[s] = hall_theta[s] + wrapPi(hall_theta[n] - hall_theta[s]) * 0.5f;
  }
  for (int i = 0; i < 6; i++) {
    int s = ord[i], p = ord[(i + 5) % 6];
    float w = wrapPi(h_bndF[s] - h_bndF[p]);
    if (w <= 0) w += TWO_PI_F;
    h_width[s] = w;
  }
  geom_ok = true;
}

void printTable();

// cal: turn the field slowly forward and back, average the angle seen in each hall state
void runCal() {
  Serial.println("cal: start (motor will turn slowly fwd then back)");
  allOff();
  hall_ok = false;
  hall_mode = false;
  geom_ok = false;
  float sx[8] = {0}, sy[8] = {0};
  for (int s = 0; s < 8; s++) hall_cnt[s] = 0;
  const int STEPS = 72, REVS = 3, DWELL = 25;
  const int N = STEPS * REVS;
  const float dA = TWO_PI_F / STEPS;
  theta_cal = 0.0f;
  cal_mode = true;
  delay(500);
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i <= N; i++) {
      float a = (pass == 0) ? i * dA : (N - i) * dA;
      theta_cal = a;
      delay(DWELL);
      uint8_t s = hall_state;
      sx[s] += cosf(a); sy[s] += sinf(a); hall_cnt[s]++;
    }
  }
  cal_mode = false;
  bool ok = (hall_cnt[0] == 0 && hall_cnt[7] == 0);
  for (int s = 1; s <= 6; s++) {
    if (hall_cnt[s] == 0) { ok = false; hall_theta[s] = 0; }
    else hall_theta[s] = atan2f(sy[s], sx[s]);
  }
  hall_ok = ok;
  printTable();
  if (!ok) { Serial.println("cal: NG (check hall wiring / calb)"); return; }
  calSave();
  buildGeometry();
  hall_mode = true;
  bias_v = 0.0f; d_bias = 0.0f;
  Serial.println("cal: OK, saved -> hmode 1, bias 0");
}

// ISR stage: rotor angle and speed from the hall states
void hallUpdate() {
  hall_state = readHall();
  static uint8_t prev_s = 0;
  static uint32_t since = 0;
  static uint32_t still = 0;
  const uint8_t hs = hall_state;
  if (since < 0x7FFFFFFF) since++;
  if (!(geom_ok && hall_ok && !cal_mode)) return;

  bool valid = (hs >= 1 && hs <= 6);
  static uint32_t bad_n = 0, good_n = 0;
  if (hall_quiet) { if (hall_quiet != 0xFFFFFFFFu) hall_quiet--; bad_n = 0; }
  else if (!valid) { good_n = 0; if (++bad_n > (uint32_t)(0.02f * FS)) hall_err = true; }
  else { bad_n = 0; if (hall_err && ++good_n > (uint32_t)(0.5f * FS)) { hall_err = false; good_n = 0; } }
  if (valid && prev_s >= 1 && prev_s <= 6 && hs != prev_s && since >= 8) {
    int d = 0;
    if (hs == h_nextF[prev_s]) d = 1;
    else if (hs == h_nextB[prev_s]) d = -1;
    if (d != 0) {
      float w_new = d * h_width[prev_s] * FS / (float)since;
      omega_est = 0.5f * omega_est + 0.5f * w_new;
      theta_est = (d > 0) ? h_bndF[prev_s] : h_bndF[hs];
      since = 0; still = 0;
    }
    prev_s = hs;
  } else if (valid && (prev_s < 1 || prev_s > 6)) {
    prev_s = hs; theta_est = h_center[hs];
  }
  if (since > (uint32_t)(0.25f * FS)) {
    omega_est *= 0.999f;
    if (++still > (uint32_t)(0.3f * FS)) { omega_est = 0.0f; }
  }
  if (valid) {
    theta_est += omega_est / FS;
    float dlt = wrapPi(theta_est - h_center[hs]);
    float hw = 0.5f * h_width[hs];
    if (dlt > hw) dlt = hw;
    if (dlt < -hw) dlt = -hw;
    if (omega_est == 0.0f) dlt *= 0.9995f;
    theta_est = h_center[hs] + dlt;
  }
  meas_rpm = omega_est * 60.0f / (TWO_PI_F * POLE_PAIRS);
}

// ISR stage: speed and strength of the rotating field. sounding = notes, audio or MIDI clock playing
void spinUpdate(bool sounding) {
  static uint32_t idle = 0xFFFFFFFF;
  static uint32_t bad = 0;
  static uint32_t still_cnt = 0;
  static bool ramped = false;
  const uint32_t hold_n = (uint32_t)(hold_s * FS);
  if (force_idle) { idle = hold_n; force_idle = false; }
  const int rmode = smode;
  const float os = ostr;
  if (sounding) idle = 0; else if (idle < 0xFFFFFFFF) idle++;
  bool can = (rmode != 0) && hall_mode && hall_ok && geom_ok && !hall_err;
  bool active = can && (idle < hold_n);

  float tgt = 0.0f;
  if (active) {
    switch (rmode) {
      case 1: {
        tgt = const_rpm;
        float sr = sync_rpm;
        if (tsync && sr > 0.0f) tgt = sr;
        break;
      }
      case 2: tgt = nrate_rpm; break;
      case 3: tgt = tmin_rpm + (tmax_rpm - tmin_rpm) * out_lvl01; break;
    }
  }
  target_rpm = tgt;

  if (active && !fld_running) {
    fld_theta = theta_est; fld_speed = 0.0f; fld_amp = 0.0f; bad = 0; still_cnt = 0; ramped = false;
    fld_running = true;
  }
  if (fld_running) {
    bool moving = fabsf(fld_speed) > 0.01f;
    if (active || moving) still_cnt = 0;
    else if (still_cnt < 0xFFFFFFFF) still_cnt++;
    bool keep = active || moving || (still_cnt < (uint32_t)(SHOLD_S * FS));
    float amp_tgt = keep ? os : 0.0f;
    float astep = (os > 0.0f ? os : 0.01f) / (ORAMP_S * FS);
    if (fld_amp < amp_tgt) { fld_amp += astep; if (fld_amp > amp_tgt) fld_amp = amp_tgt; }
    else if (fld_amp > amp_tgt) { fld_amp -= astep; if (fld_amp < amp_tgt) fld_amp = amp_tgt; }

    float sp_tgt = (fld_amp >= 0.9f * os) ? tgt : 0.0f;
    if (sp_tgt > 0.0f && fabsf(fld_speed - sp_tgt) < 0.05f) ramped = true;
    float acc;
    if (!active) acc = SDECEL;
    else if (rmode >= 2) acc = TACC;
    else acc = (tsync && sync_play && ramped) ? TACC : OACC;
    float vstep = acc / FS;
    if (fld_speed < sp_tgt) { fld_speed += vstep; if (fld_speed > sp_tgt) fld_speed = sp_tgt; }
    else if (fld_speed > sp_tgt) { fld_speed -= vstep; if (fld_speed < sp_tgt) fld_speed = sp_tgt; }

    fld_theta += spin_dir * fld_speed * (POLE_PAIRS * TWO_PI_F / 60.0f) / FS;
    if (fld_theta >  3.14159265f) fld_theta -= TWO_PI_F;
    if (fld_theta < -3.14159265f) fld_theta += TWO_PI_F;

    // the rotor fell behind the field: restart the field at the rotor angle
    fld_lag = wrapPi(fld_theta - theta_est);
    if (fld_amp > 0.5f * os && fabsf(fld_lag) > 1.75f) {
      if (++bad > (uint32_t)(0.05f * FS)) { fld_theta = theta_est; fld_speed = 0.0f; bad = 0; resync_cnt++; }
    } else bad = 0;

    if (!keep && fld_amp <= 0.0f) fld_running = false;
  }
  if (!can) { fld_running = false; fld_amp = 0.0f; fld_speed = 0.0f; }
}

// ISR stage: sound axis angle and the field voltages (rotation, bias, hold field)
void spinField(FieldOut &f, bool sounding) {
  const bool cm = cal_mode;
  float th, b;
  if (cm) {
    th = theta_cal;
    b  = cal_bias;
  } else {
    th = 0.0f;
    b  = d_bias;
    if (hall_mode && hall_ok) {
      uint8_t s = hall_state;
      if (geom_ok) th = theta_est + theta_off;
      else if (s >= 1 && s <= 6) th = hall_theta[s];
    }
  }
  if (b > u_lim) b = u_lim;

  const float famp = (cm || !hall_mode) ? 0.0f : fld_amp;
  const float fa = fld_theta;

  // hold field: a weak fixed field while sound plays and the wheel does not rotate
  static float hb = 0.0f, hang = 0.0f;
  {
    const float hf = hfield_v / vbus;
    bool snd = sounding && !cm && hall_mode && hall_ok && geom_ok && !hall_err;
    float ht = (snd && !fld_running) ? hf : 0.0f;
    if (ht < 0.0f) ht = 0.0f;
    float hs = (hf > 0.0f ? hf : 0.01f) / (0.02f * FS);
    if (hb <= 0.0f && ht > 0.0f) hang = th - theta_off;
    if (hb < ht) { hb += hs; if (hb > ht) hb = ht; }
    else if (hb > ht) { hb -= hs; if (hb < ht) hb = ht; }
  }
  const float cth = cosf(th), sth = sinf(th);
  float cfa = 0.0f, sfa = 0.0f, chg = 0.0f, shg = 0.0f;
  if (famp != 0.0f) { cfa = cosf(fa); sfa = sinf(fa); }
  if (hb != 0.0f) { chg = cosf(hang); shg = sinf(hang); }
  f.cth = cth; f.sth = sth;
  f.Pd = b + famp * (cfa * cth + sfa * sth) + hb * (chg * cth + shg * sth);
  f.Pq = famp * (sfa * cth - cfa * sth) + hb * (shg * cth - chg * sth);
  f.Ac = b * cth + famp * cfa + hb * chg;
  f.As = b * sth + famp * sfa + hb * shg;
}
