// BLDC Motor Music - shared header
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// One section per .cpp file, in the same (alphabetical) order as the tabs.
// [isr] marks variables that the audio interrupt reads or writes.

#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>
#include "usb_names.h"
#include <AudioStream.h>
#include <usb_audio.h>

#ifndef AUDIO_INTERFACE
#error "Arduino IDE: Tools > USB Type > Serial + MIDI + Audio"
#endif

#include "config.h"

extern const char* FW_VER;
extern char crash_txt[];
extern uint16_t crash_len;

const float TWO_PI_F = 6.2831853f;
const float SQRT3_2  = 0.86602540f;
const float RAD2DEG  = 57.29578f;

// ---------------------------------------------------------------- audio.cpp
// USB audio input, stereo link between the two boards (Serial5), test tone.
const uint32_t LINK_BAUD = 2000000;
const int   RB_N = 2048;
const float FILL_TGT = 640.0f;
const float ACT_HOLD_S = 0.2f;
const float EQ_TAIL_S = 0.5f;

class AudioOutputMotor : public AudioStream {
public:
  AudioOutputMotor(void) : AudioStream(2, inputQueueArray) { begin(); }
  virtual void update(void);
  void begin(void) { update_responsibility = update_setup(); }
  static void kick(void) { if (update_responsibility) update_all(); }
  static bool update_responsibility;
private:
  audio_block_t *inputQueueArray[2];
};

extern volatile float amaster;            // [isr] audio level (linear), from avol_v
extern float avol_v;                      // audio volume 0-100
extern volatile int   src;                // 0 L+R, 1 L, 2 R (role 0 only)
extern volatile int   role;               // 0 one motor, 1 right (sends left), 2 left (from Motor 1)
extern volatile bool  usbvol;             // follow the PC's volume slider
extern volatile float tone_hz;            // [isr] test tone, 0 = off
extern volatile bool  a_playing, a_active;  // [isr]
extern int16_t rb[RB_N], rbm[RB_N];
extern volatile uint32_t rb_w, rb_r;      // [isr]
extern volatile float rate_now;
extern volatile uint32_t last_blk_ms;
extern volatile uint32_t n_blk, n_over, n_under, n_upd, fill_min, fill_max;
extern volatile float pk_in;
extern uint8_t link_txmem[2048], link_rxmem[4096];
extern volatile uint32_t lk_tx, lk_drop, lk_rx, lk_bad, lk_gap;
extern volatile uint8_t lk_vol;
extern AudioInputUSB usbIn;
extern AudioOutputMotor motOut;
void linkRx();
float audioRender(bool &have);            // ISR stage: next audio sample after the audio EQ

// ---------------------------------------------------------------- console.cpp
// Serial commands, status, stat, cpu load measurement, hall/fault messages.
const int CPU_NST = 6;
extern bool echo, mon;
extern volatile bool cpu_on;
extern uint32_t cpu_cyc_us;
extern volatile uint32_t cpu_notes, cpu_notes_tot;
extern uint32_t n_clk, n_mtc, n_sx, n_bend;
extern uint32_t mon_t;
void consoleTask();
void help();
void status();
void id();
void cpuRecord(uint32_t cyc, int nv, bool rot, const uint32_t *ts);
void cpuLoop(uint32_t cyc);
void checkNodeName();

// ---------------------------------------------------------------- drv8313.cpp
// Driver-specific output stage: current model and limiter, dead-time
// compensation, PWM, EN pin. A port to another driver replaces this file.
const int   PWM_DIV_MIN = 2, PWM_DIV_MAX = 5;   // PWM = ISR rate / div: 22059 14706 11029 8824 Hz
const float LIM_KNEE = 0.7f;

struct FieldOut {          // what the rotation stage adds to the phase voltages
  float cth, sth;          // sound axis (q axis = th + 90 deg)
  float Pd, Pq;            // field voltage on the d / q axis of th (fraction of vbus)
  float Ac, As;            // field voltage in the stationary frame (fraction of vbus)
};

extern volatile float pwm_top;            // [isr] PWM counts per period
extern volatile int   pwm_div;            // ISR samples per PWM period
extern float pwm_hz;
extern float vbus, i_max;
extern volatile float u_lim;              // [isr]
extern volatile int   dt_mode;            // [isr] 0 off, 1 current-model dead-time compensation
extern float dt_ns;
extern volatile float dtc, dti;           // [isr]
extern volatile int   lmode;              // [isr] 0 clean, 1 loud
extern float lim_rel_s, lmodel_hz, vmax_set;
extern volatile float lim_rel, l_a, v_max;   // [isr]
extern volatile float i_pk, v_pk, lim_min;   // [isr]
extern volatile bool  en_state;           // [isr]
void applyPwm();
void updateLimit();
void dtUpdate();
float volRef();
void driveWake();
float driveLimit(float y, const FieldOut &f);                  // ISR stage: current/voltage limiter
void driveOutput(float uq, const FieldOut &f, bool want_on);   // ISR stage: phases, dead time, PWM, EN

// ---------------------------------------------------------------- engine.cpp
// The audio interrupt (44.1 kHz). Calls the stages in order.
const uint32_t PIT_HZ = 24000000;
const uint32_t ISR_TICKS = 544;
const float ISR_US = 22.666f;
const float FS = (float)PIT_HZ / (float)ISR_TICKS;   // 44117.6 Hz
const float RATE_NOM = 44100.0f / FS;
const float KP = 2e-6f;
const float GUARD_US = 19.0f;      // above this ISR time, the weakest voice is released
const float GUARD_HOLD_S = 0.1f;
const float GUARD_REL_S = 0.003f;
const float DECLICK_S = 0.002f;

extern IntervalTimer audioTimer;
extern volatile uint32_t isr_n, isr_max, isr_late, isr_ticks, shed_n, guard_hold;
extern volatile uint64_t isr_sum;
extern float isr_hz_meas;
extern float guard_rel;
extern volatile float out_lvl01;          // [isr] output level 0..1 (smode 3, stat)
extern volatile float declick;            // [isr] removes the step when a voice is cut
void applyEngine();
void audioISR();
void updateLevels();                      // volumes 0-100 and volts -> internal values (after imax / vbus change)
float levelDb(float v);

// ---------------------------------------------------------------- eq.cpp
// Two separate EQs: MIDI (synth) and audio (USB input). Same controls each.
const int NMID = 4;
struct MidBand { float f, g, q; };
struct EqSet {
  float hpf, lpf;            // Hz, 0 = off
  float bass, bfreq;         // low shelf dB, Hz
  float treble, tfreq;       // high shelf dB, Hz
  MidBand mid[NMID];         // peaking bands: Hz, dB, Q
};
const float EQ_DB = 30.0f, EQ_FMIN = 20.0f, EQ_FMAX = 20000.0f;   // ranges for every EQ value
struct BQ { float b0, b1, b2, a1, a2, z1, z2; };
const int EQ_NBQ = 2 + NMID + 3;
struct EqRun { BQ f[EQ_NBQ]; int n; };

extern EqSet eq_m, eq_a;
extern EqRun run_m, run_a;                // [isr]
void eqDefaults(EqSet &e);
void applyFilters();
void eqClear(EqRun &r);

inline float bq(BQ &f, float x) {
  float y = f.b0 * x + f.z1;
  f.z1 = f.b1 * x - f.a1 * y + f.z2;
  f.z2 = f.b2 * x - f.a2 * y;
  return y;
}
inline float eqRun(EqRun &r, float x) {
  const int n = r.n;
  for (int i = 0; i < n; i++) x = bq(r.f[i], x);
  return x;
}

// ---------------------------------------------------------------- led_link.cpp
// Notes and spectrum frames to the LED board (Serial1).
const int SPN = 4096;
extern int spec_mode, sp_join, sp_split, sp_onset, sp_harm;
extern float sp_gain, sp_range, sp_tilt;
extern volatile bool sp_joined;           // [isr]
extern float sp_buf[SPN];
extern volatile uint32_t sp_w;            // [isr]
extern uint8_t led_txmem[1024];
extern uint32_t led_drop;
extern uint32_t sp_frames, sp_drop;
extern float sp_ms_max;
extern uint8_t sp_lv[12];
void specInit();
void specTask();
void ledSend(uint8_t a, uint8_t b, uint8_t c);
void ledAllOff();
inline void specPush(float x) {
  static float acc = 0.0f;
  static bool odd = false;
  if (odd) { sp_buf[sp_w % SPN] = 0.5f * (acc + x); sp_w++; }
  else acc = x;
  odd = !odd;
}

// ---------------------------------------------------------------- settings.cpp
// EEPROM: settings (save / factory) and hall calibration (cal) are separate.
void settingsDefaults();
void settingsLoad();
void settingsSave();
void settingsFactory();
void calSave();
void calLoad();

// ---------------------------------------------------------------- spin.cpp
// Hall sensors, rotor angle, cal, and the rotating field (smode).
extern volatile uint8_t hall_state;       // [isr]
extern volatile bool hall_mode, hall_ok, hall_err, cal_mode, geom_ok;   // [isr]
extern volatile uint32_t hall_quiet;      // [isr]
extern volatile float theta_cal, theta_est, omega_est, meas_rpm, theta_off;   // [isr]
extern float hall_theta[8];
extern int   hall_cnt[8];
extern float cal_bias;                    // fraction of vbus, from calb_v
extern volatile float d_bias;             // [isr] d-axis hold voltage (used without cal), from bias_v
extern float ostr_v, bias_v, calb_v;      // user values in volts
extern volatile int   smode;              // [isr] 0 off, 1 constant, 2 note rate, 3 level
extern volatile int   spin_dir;
extern float const_rpm, hold_s;
extern float tmin_rpm, tmax_rpm;
extern volatile float ostr;               // [isr] field strength (fraction of vbus), from ostr_v
extern volatile float hfield_v;           // [isr] hold field when not rotating [V]
extern volatile float target_rpm, fld_speed, fld_amp, fld_lag;   // [isr]
extern volatile bool  fld_running, force_idle;                   // [isr]
extern volatile int   resync_cnt;
uint8_t readHall();
void buildGeometry();
void runCal();
void hallUpdate();                                     // ISR stage
void spinUpdate(bool sounding);                        // ISR stage
void spinField(FieldOut &f, bool sounding);            // ISR stage

// ---------------------------------------------------------------- synth.cpp
// MIDI input, 16-voice synth, drums, tempo sync, note rate.
const int MAX_VOICES = 16;
const int NPRE = 7;                       // sounds (patches); index 0..6 = patch 1..7
const int P_KICK = 8, P_SNARE = 9, P_HATC = 10, P_HATO = 11, P_CRASH = 12, P_TOM = 13;
const int NALL = 14;
const uint32_t CHORD_US = 30000;
enum { W_SINE, W_FM, W_ORGAN, W_GUITAR, W_SAW, W_PULSE, W_TRI, W_KICK, W_NOISE };
enum { OFF, ATK, DECAY, REL };

struct ChState {
  volatile uint8_t pedal;
  volatile float bend_ratio, gain, vib, trem;
  int bend_raw;
  float bend_semi;
  uint8_t vol, expr, mod;
  float atk_mul, dec_mul, rel_mul;
  uint8_t rpn_msb, rpn_lsb, rpn_semi, rpn_cent;
};
struct Voice {                            // [isr] the interrupt owns a voice once stage != OFF
  volatile uint8_t note, stage, keydown, pre, ch, shed;
  volatile float freq, phase, amp, vel, gain;
  volatile float mphase, menv, atk_step, dec_rate, rel_rate, cg, out;
  volatile uint16_t lfsr;
};

extern volatile float master;             // [isr] MIDI level (linear), from mvol_v
extern float mvol_v;                      // MIDI volume 0-100
extern float A4_HZ, bend_range, lfo_hz, vdep, dlev;
extern volatile float vcurve;
extern bool drum_on, pc_on;
extern int edit_p, poly;
extern const char* PRE_NAME[NALL];
extern const char* DRUM_NOTES[6];
extern uint8_t pr_wave[NALL];             // per sound: waveform and parameters
extern float pr_atk[NALL], pr_dec[NALL], pr_sus[NALL], pr_rel[NALL], pr_lev[NALL];
extern float pr_p1[NALL], pr_p2[NALL], pr_p3[NALL];
extern uint8_t ch_pre[17];
extern ChState chs[17];
extern Voice vox[MAX_VOICES];
extern bool tsync;
extern float tbase, tgain, nbase, nk, ntau;
extern volatile float sync_rpm, cur_bpm, nrate_rpm;   // [isr]
extern volatile bool sync_play;           // [isr]
void updateEnv();
void allOff();
void chResetEvery();
void chSetBend(float semi);
void chUpdateAll();
int  drumPreset(uint8_t note);
int  pedalCount();
void syncTask();
void noteOn(byte ch, byte note, byte vel);
void noteOff(byte ch, byte note, byte vel);
void onControl(byte ch, byte cc, byte val);
void onProgram(byte ch, byte prog);
void onBend(byte ch, int b);
void onClock();
void onStart();
void onContinue();
void onStop();
void onMtc(byte data);
void onSongPos(uint16_t beats);
void onSysex(uint8_t *data, unsigned int size);
float synthRender(int &nvoice);           // ISR stage: sum of all voices

inline float fsin(float x) {              // fast sine, error below -110 dB
  const float n = (float)(int)(x * 0.15915494f + (x >= 0.0f ? 0.5f : -0.5f));
  float r = x - n * TWO_PI_F;
  if (r > 1.57079633f) r = 3.14159265f - r;
  else if (r < -1.57079633f) r = -3.14159265f - r;
  const float r2 = r * r;
  return r * (1.0f + r2 * (-1.6666667e-1f + r2 * (8.3333333e-3f + r2 * (-1.9841270e-4f + r2 * (2.7557319e-6f + r2 * -2.5052108e-8f)))));
}
inline float fcos(float x) { return fsin(x + 1.57079633f); }
