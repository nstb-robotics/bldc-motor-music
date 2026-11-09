// BLDC Motor Music - settings in EEPROM
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// "save" stores every setting at once; "factory" erases them (cal is kept).
// The hall calibration from "cal" is stored separately.
// Layout: 0 magic, 4 size, 6 checksum, 8 settings ... | 1000 cal magic, 1004 cal data.

#include "motor_music.h"

const uint32_t SET_MAGIC = 0x424D4D32;   // "BMM2"
const int EE_SET_MAGIC = 0, EE_SET_SIZE = 4, EE_SET_SUM = 6, EE_SET_DATA = 8;
const uint32_t CAL_MAGIC = 0x424D4D43;   // "BMMC"
const int EE_CAL_MAGIC = 1000, EE_CAL_DATA = 1004;

struct Settings {
  float mvol, avol, a4, vcurve, bend, lfo, vdep, dlev;
  int32_t drum_on, pc_on, voices, echo, mon;
  float voice[NPRE][8];
  float drum[6][5];
  int32_t smode, sdir, tsync;
  float const_rpm, ostr, hold, tbase, tgain, tmin, tmax, nbase, nk, ntau, toff;
  float vbus, imax, dtns, dti, hfield, limrel, lmodel, vmax;
  int32_t pwm_div, dtm, lmode;
  int32_t role, src, usbvol;
  int32_t spec, sjoin, ssplit, sonset, sharm;
  float sgain, srange, stilt;
  EqSet eq_m, eq_a;
};
static_assert(EE_SET_DATA + sizeof(Settings) <= EE_CAL_MAGIC, "Settings do not fit");
static_assert(EE_CAL_DATA + sizeof(hall_theta) <= 1080, "cal does not fit");

void settingsDefaults() {
  eqDefaults(eq_m);
  eqDefaults(eq_a);
  role = (MOTOR_NUMBER == 1 || MOTOR_NUMBER == 2) ? MOTOR_NUMBER : 0;
}

static void get(Settings &s) {
  s.mvol = mvol_v; s.avol = avol_v; s.a4 = A4_HZ; s.vcurve = vcurve;
  s.bend = bend_range; s.lfo = lfo_hz; s.vdep = vdep; s.dlev = dlev;
  s.drum_on = drum_on; s.pc_on = pc_on; s.voices = poly; s.echo = echo; s.mon = mon;
  for (int p = 0; p < NPRE; p++) {
    s.voice[p][0] = pr_atk[p]; s.voice[p][1] = pr_dec[p]; s.voice[p][2] = pr_sus[p]; s.voice[p][3] = pr_rel[p];
    s.voice[p][4] = pr_lev[p]; s.voice[p][5] = pr_p1[p]; s.voice[p][6] = pr_p2[p]; s.voice[p][7] = pr_p3[p];
  }
  for (int k = 0; k < 6; k++) {
    int p = P_KICK + k;
    s.drum[k][0] = pr_lev[p]; s.drum[k][1] = pr_dec[p]; s.drum[k][2] = pr_p1[p]; s.drum[k][3] = pr_p2[p]; s.drum[k][4] = pr_p3[p];
  }
  s.smode = smode; s.sdir = spin_dir; s.tsync = tsync;
  s.const_rpm = const_rpm; s.ostr = ostr_v; s.hold = hold_s; s.tbase = tbase; s.tgain = tgain;
  s.tmin = tmin_rpm; s.tmax = tmax_rpm; s.nbase = nbase; s.nk = nk; s.ntau = ntau; s.toff = theta_off;
  s.vbus = vbus; s.imax = i_max; s.dtns = dt_ns; s.dti = dti; s.hfield = hfield_v;
  s.limrel = lim_rel_s; s.lmodel = lmodel_hz; s.vmax = vmax_set;
  s.pwm_div = pwm_div; s.dtm = dt_mode; s.lmode = lmode;
  s.role = role; s.src = src; s.usbvol = usbvol;
  s.spec = spec_mode; s.sjoin = sp_join; s.ssplit = sp_split; s.sonset = sp_onset; s.sharm = sp_harm;
  s.sgain = sp_gain; s.srange = sp_range; s.stilt = sp_tilt;
  s.eq_m = eq_m; s.eq_a = eq_a;
}

static void eqSet(EqSet &d, const EqSet &s) {
  d.hpf = (s.hpf <= 0.0f) ? 0.0f : constrain(s.hpf, EQ_FMIN, EQ_FMAX);
  d.lpf = (s.lpf <= 0.0f) ? 0.0f : constrain(s.lpf, EQ_FMIN, EQ_FMAX);
  d.bass = constrain(s.bass, -EQ_DB, EQ_DB);
  d.bfreq = constrain(s.bfreq, EQ_FMIN, EQ_FMAX);
  d.treble = constrain(s.treble, -EQ_DB, EQ_DB);
  d.tfreq = constrain(s.tfreq, EQ_FMIN, EQ_FMAX);
  for (int i = 0; i < NMID; i++) {
    d.mid[i].f = constrain(s.mid[i].f, EQ_FMIN, EQ_FMAX);
    d.mid[i].g = constrain(s.mid[i].g, -EQ_DB, EQ_DB);
    d.mid[i].q = constrain(s.mid[i].q, 0.3f, 10.0f);
  }
}

static void set(const Settings &s) {
  mvol_v = constrain(s.mvol, 0.0f, 100.0f);
  avol_v = constrain(s.avol, 0.0f, 100.0f);
  A4_HZ = constrain(s.a4, 400.0f, 480.0f);
  vcurve = constrain(s.vcurve, 0.2f, 4.0f);
  bend_range = constrain(s.bend, 0.0f, 24.0f);
  lfo_hz = constrain(s.lfo, 0.1f, 20.0f);
  vdep = constrain(s.vdep, 0.0f, 2.0f);
  dlev = constrain(s.dlev, 0.0f, 4.0f);
  drum_on = s.drum_on != 0; pc_on = s.pc_on != 0; echo = s.echo != 0; mon = s.mon != 0;
  poly = constrain((int)s.voices, 4, MAX_VOICES);
  for (int p = 0; p < NPRE; p++) {
    float lo = 0.0f, hi = 20.0f;
    if (pr_wave[p] == W_PULSE) { lo = 0.05f; hi = 0.95f; }
    if (pr_wave[p] == W_TRI)   { lo = 0.0f;  hi = 32.0f; }
    pr_atk[p] = constrain(s.voice[p][0], 0.0005f, 1.0f);
    pr_dec[p] = constrain(s.voice[p][1], 0.02f, 20.0f);
    pr_sus[p] = constrain(s.voice[p][2], 0.0f, 1.0f);
    pr_rel[p] = constrain(s.voice[p][3], 0.01f, 5.0f);
    pr_lev[p] = constrain(s.voice[p][4], 0.0f, 4.0f);
    pr_p1[p] = constrain(s.voice[p][5], lo, hi);
    pr_p2[p] = constrain(s.voice[p][6], 0.0f, 20.0f);
    pr_p3[p] = constrain(s.voice[p][7], 0.0f, 20.0f);
  }
  for (int k = 0; k < 6; k++) {
    int p = P_KICK + k;
    bool tonal = (pr_wave[p] == W_KICK);
    pr_lev[p] = constrain(s.drum[k][0], 0.0f, 4.0f);
    pr_dec[p] = constrain(s.drum[k][1], 0.01f, 5.0f);
    pr_p1[p] = constrain(s.drum[k][2], tonal ? 20.0f : 100.0f, tonal ? 1000.0f : 40000.0f);
    if (tonal) { pr_p2[p] = constrain(s.drum[k][3], 0.005f, 1.0f); pr_p3[p] = constrain(s.drum[k][4], 0.0f, 10.0f); }
  }
  smode = constrain((int)s.smode, 0, 3);
  spin_dir = (s.sdir < 0) ? -1 : 1;
  tsync = s.tsync != 0;
  const_rpm = constrain(s.const_rpm, 0.0f, 120.0f);
  ostr_v = constrain(s.ostr, 0.0f, 0.85f);
  hold_s = constrain(s.hold, 0.0f, 30.0f);
  tbase = constrain(s.tbase, 20.0f, 300.0f);
  tgain = constrain(s.tgain, 0.5f, 5.0f);
  tmin_rpm = constrain(s.tmin, 0.0f, 30.0f);
  tmax_rpm = constrain(s.tmax, 1.0f, 60.0f);
  nbase = constrain(s.nbase, 0.0f, 30.0f);
  nk = constrain(s.nk, 0.0f, 20.0f);
  ntau = constrain(s.ntau, 0.05f, 5.0f);
  theta_off = constrain(s.toff, -3.1416f, 3.1416f);
  vbus = constrain(s.vbus, 8.0f, 30.0f);
  i_max = constrain(s.imax, 0.2f, IMAX_LIMIT);
  dt_ns = constrain(s.dtns, 0.0f, 1500.0f);
  dti = constrain(s.dti, 0.02f, 1.0f);
  hfield_v = constrain(s.hfield, 0.0f, 0.85f);
  lim_rel_s = constrain(s.limrel, 0.01f, 2.0f);
  lmodel_hz = constrain(s.lmodel, 50.0f, 5000.0f);
  vmax_set = constrain(s.vmax, 0.0f, 15.0f);
  pwm_div = constrain((int)s.pwm_div, PWM_DIV_MIN, PWM_DIV_MAX);
  dt_mode = s.dtm ? 1 : 0;
  lmode = s.lmode ? 1 : 0;
  role = constrain((int)s.role, 0, 2);
  src = constrain((int)s.src, 0, 2);
  usbvol = s.usbvol != 0;
  spec_mode = constrain((int)s.spec, 0, 3);
  sp_join = s.sjoin ? 1 : 0;
  sp_split = constrain((int)s.ssplit, 36, 96);
  sp_onset = s.sonset ? 1 : 0;
  sp_harm = s.sharm ? 1 : 0;
  sp_gain = constrain(s.sgain, -30.0f, 40.0f);
  sp_range = constrain(s.srange, 10.0f, 80.0f);
  sp_tilt = constrain(s.stilt, 0.0f, 6.0f);
  eqSet(eq_m, s.eq_m);
  eqSet(eq_a, s.eq_a);
}

static uint16_t sum16(const uint8_t *p, int n) {
  uint16_t a = 0x5A5A;
  for (int i = 0; i < n; i++) a = (uint16_t)((a << 1) | (a >> 15)) ^ p[i];
  return a;
}

void settingsLoad() {
  uint32_t m; EEPROM.get(EE_SET_MAGIC, m);
  if (m != SET_MAGIC) { Serial.println("settings: none saved, using defaults"); return; }
  uint16_t n, sum; EEPROM.get(EE_SET_SIZE, n); EEPROM.get(EE_SET_SUM, sum);
  if (n != sizeof(Settings)) { Serial.println("settings: size mismatch, using defaults"); return; }
  Settings s; EEPROM.get(EE_SET_DATA, s);
  if (sum16((const uint8_t *)&s, sizeof(s)) != sum) { Serial.println("settings: checksum error, using defaults"); return; }
  set(s);
  Serial.println("settings loaded");
}

void settingsSave() {
  hall_quiet = 0xFFFFFFFFu;     // EEPROM writes stall the CPU; do not report hall errors meanwhile
  Settings s; get(s);
  EEPROM.put(EE_SET_DATA, s);
  EEPROM.put(EE_SET_SIZE, (uint16_t)sizeof(s));
  EEPROM.put(EE_SET_SUM, sum16((const uint8_t *)&s, sizeof(s)));
  EEPROM.put(EE_SET_MAGIC, SET_MAGIC);
  hall_quiet = (uint32_t)(0.5f * FS);
  Serial.println("settings saved");
}

void settingsFactory() {
  uint32_t z = 0;
  EEPROM.put(EE_SET_MAGIC, z);
  Serial.println("settings erased (cal kept). Reboot to use the defaults.");
}

void calSave() {
  EEPROM.put(EE_CAL_DATA, hall_theta);
  EEPROM.put(EE_CAL_MAGIC, CAL_MAGIC);
}

void calLoad() {
  uint32_t m; EEPROM.get(EE_CAL_MAGIC, m);
  if (m != CAL_MAGIC) { Serial.println("cal: not done yet (type cal with the wheel free to turn)"); return; }
  EEPROM.get(EE_CAL_DATA, hall_theta);
  buildGeometry();
  hall_ok = true;
  hall_mode = true;
  bias_v = 0.0f; d_bias = 0.0f;
  Serial.println("cal loaded");
}
