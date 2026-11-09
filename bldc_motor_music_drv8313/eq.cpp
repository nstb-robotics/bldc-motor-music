// BLDC Motor Music - EQ for MIDI and for audio (separate settings, same controls)
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Chain: high-pass, 4 mid bands, low shelf, high shelf, low-pass (4th order).
// The low-pass also keeps content under half the PWM frequency
// (pwmf 22059 -> 11 kHz; with pwmf 11029, set it near 5000 to avoid aliasing).

#include "motor_music.h"

const float HPF_DEF = 60.0f;
const float LPF_DEF = 9265.0f;

EqSet eq_m, eq_a;
EqRun run_m, run_a;

static void setBQ(BQ &f, double b0, double b1, double b2, double a0, double a1, double a2) {
  f.b0 = b0 / a0; f.b1 = b1 / a0; f.b2 = b2 / a0; f.a1 = a1 / a0; f.a2 = a2 / a0; f.z1 = f.z2 = 0;
}

static void designLP(BQ &f, double fc, double q) {
  double w = 2 * M_PI * fc / FS, c = cos(w), al = sin(w) / (2 * q);
  setBQ(f, (1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
}

static void designHP(BQ &f, double fc, double q) {
  double w = 2 * M_PI * fc / FS, c = cos(w), al = sin(w) / (2 * q);
  setBQ(f, (1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
}

static void designLowShelf(BQ &f, double fc, double db) {
  double A = pow(10.0, db / 40.0), w = 2 * M_PI * fc / FS, c = cos(w), s = sin(w);
  double al = s / 2 * sqrt(2.0), sa = 2 * sqrt(A) * al;
  setBQ(f, A * ((A + 1) - (A - 1) * c + sa), 2 * A * ((A - 1) - (A + 1) * c), A * ((A + 1) - (A - 1) * c - sa),
        (A + 1) + (A - 1) * c + sa, -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - sa);
}

static void designHighShelf(BQ &f, double fc, double db) {
  double A = pow(10.0, db / 40.0), w = 2 * M_PI * fc / FS, c = cos(w), s = sin(w);
  double al = s / 2 * sqrt(2.0), sa = 2 * sqrt(A) * al;
  setBQ(f, A * ((A + 1) + (A - 1) * c + sa), -2 * A * ((A - 1) + (A + 1) * c), A * ((A + 1) + (A - 1) * c - sa),
        (A + 1) - (A - 1) * c + sa, 2 * ((A - 1) - (A + 1) * c), (A + 1) - (A - 1) * c - sa);
}

static void designPeak(BQ &f, double fc, double db, double q) {
  double A = pow(10.0, db / 40.0), w = 2 * M_PI * fc / FS, c = cos(w), al = sin(w) / (2 * q);
  setBQ(f, 1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A);
}

void eqDefaults(EqSet &e) {
  e.hpf = HPF_DEF; e.lpf = LPF_DEF;
  e.bass = 0.0f; e.bfreq = 120.0f;
  e.treble = 0.0f; e.tfreq = 2500.0f;
  const float F[NMID] = {250.0f, 500.0f, 1000.0f, 3000.0f};
  for (int i = 0; i < NMID; i++) e.mid[i] = {F[i], 0.0f, 2.0f};
}

// only the active filters are kept in the chain
static void design(const EqSet &e, EqRun &r) {
  int n = 0;
  if (e.hpf > 0.0f) designHP(r.f[n++], e.hpf, 0.7071068);
  for (int i = 0; i < NMID; i++)
    if (fabsf(e.mid[i].g) > 0.05f && e.mid[i].f > 0.0f) designPeak(r.f[n++], e.mid[i].f, e.mid[i].g, e.mid[i].q);
  if (fabsf(e.bass) > 0.05f) designLowShelf(r.f[n++], e.bfreq, e.bass);
  if (fabsf(e.treble) > 0.05f) designHighShelf(r.f[n++], e.tfreq, e.treble);
  if (e.lpf > 0.0f) { designLP(r.f[n++], e.lpf, 0.5411961); designLP(r.f[n++], e.lpf, 1.3065630); }
  r.n = n;
}

void applyFilters() {
  EqRun m, a;
  design(eq_m, m);
  design(eq_a, a);
  noInterrupts();
  run_m = m;
  run_a = a;
  interrupts();
}

void eqClear(EqRun &r) {
  for (int i = 0; i < EQ_NBQ; i++) { r.f[i].z1 = 0.0f; r.f[i].z2 = 0.0f; }
}
