// BLDC Motor Music - serial commands and status
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Type "<name> <value>" in the Serial Monitor (several with ";").
// st shows every setting, help lists the commands, save stores them.
// stat and cpu measure the interrupt load and print the result once.

#include "motor_music.h"

bool echo = false;
bool mon = false;
uint32_t mon_t = 0;
uint32_t n_clk = 0, n_mtc = 0, n_sx = 0, n_bend = 0;
volatile bool cpu_on = false;
uint32_t cpu_cyc_us = 600;
volatile uint32_t cpu_notes = 0, cpu_notes_tot = 0;
static bool hall_err_shown = false, fault_prev = false, hallmon = false;
static uint8_t hall_prev = 255;
static char cmdbuf[64];
static uint8_t cmdlen = 0;

// cpu measurement
static uint32_t cpu_n = 0, cpu_max = 0, cpu_over = 0, cpu_t0 = 0, cpu_t1 = 0, cpu_lmax = 0;
static uint64_t cpu_sum = 0;
static uint32_t cpu_vn[MAX_VOICES + 1], cpu_vmax[MAX_VOICES + 1];
static uint64_t cpu_vsum[MAX_VOICES + 1];
static uint64_t cpu_st_sum[CPU_NST];
static uint32_t cpu_st_max[CPU_NST];
static uint8_t cpu_max_voices = 0, cpu_max_rot = 0, cpu_max_smode = 0;
static bool cpu_lskip = false;


// ---------------------------------------------------------------- cpu
void cpuRecord(uint32_t cyc, int nv, bool rot, const uint32_t *ts) {
  cpu_n++; cpu_sum += cyc;
  if (cyc > cpu_max) { cpu_max = cyc; cpu_max_voices = nv; cpu_max_rot = rot; cpu_max_smode = smode; }
  if (cyc > (uint32_t)(cpu_cyc_us * (1.0e6f / FS))) cpu_over++;
  int vi = (nv > MAX_VOICES) ? MAX_VOICES : nv;
  cpu_vn[vi]++; cpu_vsum[vi] += cyc;
  if (cyc > cpu_vmax[vi]) cpu_vmax[vi] = cyc;
  for (int i = 0; i < CPU_NST; i++) {
    uint32_t d = ts[i + 1] - ts[i];
    cpu_st_sum[i] += d;
    if (d > cpu_st_max[i]) cpu_st_max[i] = d;
  }
}

void cpuLoop(uint32_t cyc) {
  if (!cpu_on) return;
  if (cpu_lskip) { cpu_lskip = false; return; }
  if (cyc > cpu_lmax) cpu_lmax = cyc;
}

static void cpuStart() {
  noInterrupts();
  cpu_on = false;
  cpu_n = 0; cpu_max = 0; cpu_over = 0; cpu_sum = 0; cpu_lmax = 0;
  for (int i = 0; i <= MAX_VOICES; i++) { cpu_vn[i] = 0; cpu_vmax[i] = 0; cpu_vsum[i] = 0; }
  for (int i = 0; i < CPU_NST; i++) { cpu_st_sum[i] = 0; cpu_st_max[i] = 0; }
  cpu_max_voices = 0; cpu_max_rot = 0; cpu_max_smode = 0;
  cpu_notes = 0; cpu_notes_tot = 0;
  cpu_lskip = true;
  cpu_t0 = millis();
  cpu_on = true;
  interrupts();
  Serial.println("cpu: measuring (cpu 0 to stop and print)");
}

static void printNodeName() {
  int n = ((int)usb_string_product_name.bLength - 2) / 2;
  for (int i = 0; i < n; i++) Serial.print((char)usb_string_product_name.wString[i]);
}

static void cpuStop() {
  if (!cpu_on && cpu_n == 0) { Serial.println("cpu: no data (cpu 1 to start)"); return; }
  if (cpu_on) { cpu_on = false; cpu_t1 = millis(); }
  float cu = (float)cpu_cyc_us;
  Serial.println("#cpu_summary");
  Serial.print("fw,"); Serial.println(FW_VER);
  Serial.print("node,"); printNodeName(); Serial.println();
  Serial.print("duration_s,"); Serial.println((cpu_t1 - cpu_t0) / 1000.0f, 2);
  Serial.print("isr_count,"); Serial.println(cpu_n);
  Serial.print("isr_avg_us,"); Serial.println(cpu_n ? (float)((double)cpu_sum / cpu_n) / cu : 0.0f, 3);
  Serial.print("isr_max_us,"); Serial.println(cpu_max / cu, 3);
  Serial.print("isr_budget_us,"); Serial.println(1e6f / FS, 2);
  Serial.print("isr_max_pct_of_budget,"); Serial.println(cpu_max / cu * FS * 1e-4f, 1);
  Serial.print("over_budget_count,"); Serial.println(cpu_over);
  Serial.print("shed,"); Serial.println(shed_n);
  Serial.print("max_at_voices,"); Serial.println(cpu_max_voices);
  Serial.print("max_at_rotating,"); Serial.println(cpu_max_rot);
  Serial.print("max_at_smode,"); Serial.println(cpu_max_smode);
  Serial.print("loop_max_us,"); Serial.println(cpu_lmax / cu, 1);
  Serial.print("notes_total,"); Serial.println(cpu_notes_tot);
  Serial.println("#by_voices");
  Serial.println("voices,count,avg_us,max_us");
  for (int i = 0; i <= MAX_VOICES; i++) {
    if (!cpu_vn[i]) continue;
    Serial.print(i); Serial.print(","); Serial.print(cpu_vn[i]); Serial.print(",");
    Serial.print((float)((double)cpu_vsum[i] / cpu_vn[i]) / cu, 3); Serial.print(",");
    Serial.println(cpu_vmax[i] / cu, 3);
  }
  Serial.println("#by_stage");
  Serial.println("stage,avg_us,max_us");
  static const char *SN[CPU_NST] = {"hall", "synth", "audio_in", "rotation", "eq_limiter", "output"};
  for (int i = 0; i < CPU_NST; i++) {
    Serial.print(SN[i]); Serial.print(",");
    Serial.print(cpu_n ? (float)((double)cpu_st_sum[i] / cpu_n) / cu : 0.0f, 3); Serial.print(",");
    Serial.println(cpu_st_max[i] / cu, 3);
  }
  Serial.println("#end");
}

// ---------------------------------------------------------------- stat
static void statClear() {
  noInterrupts();
  n_blk = n_over = n_under = n_upd = 0;
  fill_min = 0xFFFFFFFF; fill_max = 0;
  pk_in = 0.0f; lim_min = 1.0f; i_pk = 0.0f; v_pk = 0.0f;
  isr_n = 0; isr_max = 0; isr_late = 0; isr_sum = 0; shed_n = 0;
  lk_tx = lk_drop = lk_rx = lk_bad = lk_gap = 0;
  interrupts();
}

static float dbfs(float a) { return (a > 1e-6f) ? 20.0f * log10f(a) : -120.0f; }

static void stat() {
  noInterrupts();
  uint32_t nb = n_blk, no = n_over, nu = n_under, nup = n_upd, fmn = fill_min, fmx = fill_max;
  float pk = pk_in, lm = lim_min;
  uint32_t in = isr_n, imx = isr_max, il = isr_late; uint64_t is = isr_sum;
  interrupts();
  float cu = (float)cpu_cyc_us;
  Serial.println("#stat");
  Serial.print("blocks="); Serial.print(nb); Serial.print(" updates="); Serial.print(nup);
  Serial.print(" overrun="); Serial.print(no); Serial.print(" underrun="); Serial.println(nu);
  Serial.print("fill min="); Serial.print(fmn == 0xFFFFFFFF ? 0 : fmn); Serial.print(" max="); Serial.println(fmx);
  Serial.print("isr max="); Serial.print((float)imx / cu, 2); Serial.print("us avg=");
  Serial.print(in ? (float)(is / in) / cu : 0.0f, 2); Serial.print("us budget=");
  Serial.print(1e6f / FS, 2); Serial.print("us late="); Serial.print(il);
  Serial.print(" shed="); Serial.println(shed_n);
  Serial.print("peak in="); Serial.print(dbfs(pk), 1); Serial.print("dBFS  limiter max reduction=");
  Serial.print(-dbfs(lm), 1); Serial.println("dB");
  Serial.print("peak V="); Serial.print(v_pk, 2); Serial.print("V  peak I(calc)="); Serial.print(i_pk, 2); Serial.println("A");
  Serial.print("link tx="); Serial.print(lk_tx); Serial.print(" txdrop="); Serial.print(lk_drop);
  Serial.print(" rx="); Serial.print(lk_rx); Serial.print(" bad="); Serial.print(lk_bad); Serial.print(" gap="); Serial.println(lk_gap);
  Serial.print("led notes dropped="); Serial.println(led_drop);
  Serial.print("spec frames="); Serial.print(sp_frames); Serial.print(" drop="); Serial.print(sp_drop);
  Serial.print(" fft max="); Serial.print(sp_ms_max, 2); Serial.print("ms levels:");
  for (int k = 0; k < 12; k++) { Serial.print(' '); Serial.print(sp_lv[k]); }
  Serial.println();
  Serial.println("#end");
}

// ---------------------------------------------------------------- status
static void printBits(uint8_t s) {
  Serial.print((s >> 2) & 1); Serial.print((s >> 1) & 1); Serial.print(s & 1);
}

static void printHall() {
  uint8_t s = hall_state;
  Serial.print("hall ABC="); printBits(s);
  Serial.print(" ("); Serial.print(s); Serial.println(")");
}

void printTable() {
  Serial.println("state ABC   deg   count");
  for (int s = 1; s <= 6; s++) {
    float d = hall_theta[s] * RAD2DEG;
    if (d < 0) d += 360.0f;
    Serial.print("  "); Serial.print(s); Serial.print("   ");
    printBits(s); Serial.print("  ");
    Serial.print(d, 1); Serial.print("  ");
    Serial.println(hall_cnt[s]);
  }
  Serial.print("invalid 000="); Serial.print(hall_cnt[0]);
  Serial.print(" 111="); Serial.print(hall_cnt[7]);
  Serial.print("  hall_ok="); Serial.println(hall_ok);
}

void id() {
  Serial.print("node=");  printNodeName();
  Serial.print(" fw=");   Serial.print(FW_VER);
  Serial.print(" driver=DRV8313");
  Serial.print(" pp=");   Serial.print(POLE_PAIRS);
  Serial.print(" reff="); Serial.print(R_EFF, 2);
  if (crash_len) Serial.print(" crash=1");
  Serial.println();
}

static void eqLine(const char *name, const EqSet &e) {
  Serial.print(name);
  Serial.print(" hpf="); Serial.print(e.hpf, 0);
  Serial.print(" lpf="); Serial.print(e.lpf, 0);
  Serial.print(" bass="); Serial.print(e.bass, 1); Serial.print("dB@"); Serial.print(e.bfreq, 0);
  Serial.print(" treble="); Serial.print(e.treble, 1); Serial.print("dB@"); Serial.print(e.tfreq, 0);
  for (int i = 0; i < NMID; i++) {
    Serial.print(" mid"); Serial.print(i + 1); Serial.print("="); Serial.print(e.mid[i].f, 0);
    Serial.print("/"); Serial.print(e.mid[i].g, 1); Serial.print("/"); Serial.print(e.mid[i].q, 1);
  }
  Serial.println();
}

void status() {
  id();
  Serial.print("midi:  mvol="); Serial.print(mvol_v, 1); Serial.print("("); Serial.print(levelDb(mvol_v), 1); Serial.print("dB)");
  Serial.print(" poly="); Serial.print(poly);
  Serial.print(" a4="); Serial.print(A4_HZ, 1);
  Serial.print(" vcurve="); Serial.print(vcurve, 2);
  Serial.print(" bend="); Serial.print(bend_range, 1);
  Serial.print(" lfo="); Serial.print(lfo_hz, 1);
  Serial.print(" vdep="); Serial.print(vdep, 2);
  Serial.print(" pc="); Serial.print(pc_on);
  Serial.print(" drum="); Serial.print(drum_on);
  Serial.print(" dlev="); Serial.print(dlev, 2);
  Serial.print(" ch1-4="); for (int c = 1; c <= 4; c++) { Serial.print(ch_pre[c] + 1); if (c < 4) Serial.print(","); }
  Serial.println();
  Serial.print("patch: edit="); Serial.print(edit_p + 1); Serial.print(":"); Serial.print(PRE_NAME[edit_p]);
  Serial.print(" atk="); Serial.print(pr_atk[edit_p], 3);
  Serial.print(" dec="); Serial.print(pr_dec[edit_p], 2);
  Serial.print(" sus="); Serial.print(pr_sus[edit_p], 2);
  Serial.print(" rel="); Serial.print(pr_rel[edit_p], 2);
  Serial.print(" plev="); Serial.print(pr_lev[edit_p], 2);
  Serial.print(" p1="); Serial.print(pr_p1[edit_p], 3);
  Serial.print(" p2="); Serial.print(pr_p2[edit_p], 3);
  Serial.print(" p3="); Serial.println(pr_p3[edit_p], 3);
  eqLine("meq:  ", eq_m);
  Serial.print("audio: avol="); Serial.print(avol_v, 1); Serial.print("("); Serial.print(levelDb(avol_v), 1); Serial.print("dB)");
  Serial.print(" role="); Serial.print(role);
  Serial.print(role == 0 ? "(one motor)" : role == 1 ? "(right, sends left)" : "(left)");
  Serial.print(" src="); Serial.print(src);
  Serial.print(" usbvol="); Serial.print(usbvol);
  Serial.print("("); Serial.print((role == 2 ? lk_vol / 255.0f : usbIn.volume()) * 100.0f, 0); Serial.print("%)");
  Serial.print(" tone="); Serial.print(tone_hz, 0);
  bool stream = (millis() - last_blk_ms) < 200;
  Serial.print(" | stream="); Serial.print(stream ? "yes" : "no");
  Serial.print(" playing="); Serial.print(a_playing);
  Serial.print(" pc_rate="); Serial.println(rate_now * FS, 1);
  eqLine("aeq:  ", eq_a);
  Serial.print("spin:  smode="); Serial.print(smode);
  Serial.print(" orpm="); Serial.print(const_rpm, 1);
  Serial.print(" ostr="); Serial.print(ostr_v, 2); Serial.print("V");
  Serial.print(" hold="); Serial.print(hold_s, 1);
  Serial.print(" sdir="); Serial.print(spin_dir);
  Serial.print(" | tsync="); Serial.print(tsync);
  Serial.print(" tbase="); Serial.print(tbase, 0);
  Serial.print(" tgain="); Serial.print(tgain, 2);
  Serial.print(" tmin="); Serial.print(tmin_rpm, 1);
  Serial.print(" tmax="); Serial.print(tmax_rpm, 0);
  Serial.print(" | nbase="); Serial.print(nbase, 1);
  Serial.print(" nk="); Serial.print(nk, 2);
  Serial.print(" ntau="); Serial.println(ntau, 2);
  Serial.print("hall:  hmode="); Serial.print(hall_mode);
  Serial.print(" hall_ok="); Serial.print(hall_ok);
  Serial.print(" hall="); printBits(hall_state);
  Serial.print(" hall_err="); Serial.print(hall_err);
  Serial.print(" toff="); Serial.print(theta_off * RAD2DEG, 0);
  Serial.print(" calb="); Serial.print(calb_v, 2); Serial.print("V");
  Serial.print(" bias="); Serial.print(bias_v, 2); Serial.println("V");
  Serial.print("drive: vbus="); Serial.print(vbus, 1);
  Serial.print(" imax="); Serial.print(i_max, 2);
  Serial.print(" pwmf="); Serial.print(pwm_hz, 0);
  Serial.print(" dtm="); Serial.print(dt_mode);
  Serial.print(" dtns="); Serial.print(dt_ns, 0);
  Serial.print(" dti="); Serial.print(dti, 3);
  Serial.print(" hfield="); Serial.print(hfield_v, 2); Serial.print("V");
  Serial.print(" lmode="); Serial.print(lmode); Serial.print(lmode ? "(loud)" : "(clean)");
  Serial.print(" limrel="); Serial.print(lim_rel_s, 2);
  Serial.print(" lmodel="); Serial.print(lmodel_hz, 0);
  Serial.print(" vmax="); Serial.print(v_max, 2); Serial.print("V");
  Serial.print(" | EN="); Serial.print(en_state);
  Serial.print(" fault="); Serial.println(fault_prev);
  Serial.print("sent to LED board: spec="); Serial.print(spec_mode);
  Serial.print(" sgain="); Serial.print(sp_gain, 1);
  Serial.print(" srange="); Serial.print(sp_range, 1);
  Serial.print(" stilt="); Serial.print(sp_tilt, 1);
  Serial.print(" sjoin="); Serial.print(sp_join); Serial.print(sp_joined ? "(on)" : "(off)");
  Serial.print(" ssplit="); Serial.print(sp_split);
  Serial.print(" sonset="); Serial.print(sp_onset);
  Serial.print(" sharm="); Serial.print(sp_harm);
  Serial.print(" | lvl="); Serial.println(out_lvl01, 2);
  Serial.print("misc:  echo="); Serial.print(echo);
  Serial.print(" mon="); Serial.print(mon);
  Serial.print(" play="); Serial.print(sync_play);
  Serial.print(" bpm="); Serial.print(cur_bpm, 1);
  Serial.print(" pedal_ch="); Serial.print(pedalCount());
  Serial.print(" isr="); Serial.print(isr_hz_meas, 0); Serial.println("Hz");
}

static void drumShow(int p) {
  Serial.print("  "); Serial.print(PRE_NAME[p]);
  Serial.print(" ("); Serial.print(DRUM_NOTES[p - P_KICK]); Serial.print(")");
  Serial.print(" dvol="); Serial.print(pr_lev[p], 2);
  Serial.print(" ddec="); Serial.print(pr_dec[p], 3);
  Serial.print(" dfreq="); Serial.print(pr_p1[p], 0);
  if (pr_wave[p] == W_KICK) { Serial.print(" dsweep="); Serial.print(pr_p3[p], 2); Serial.print(" dstime="); Serial.print(pr_p2[p], 3); }
  Serial.println();
}

static void plist() {
  for (int p = 0; p < NPRE; p++) {
    Serial.print(p + 1); Serial.print(" "); Serial.print(PRE_NAME[p]);
    Serial.print(" atk="); Serial.print(pr_atk[p], 3);
    Serial.print(" dec="); Serial.print(pr_dec[p], 2);
    Serial.print(" sus="); Serial.print(pr_sus[p], 2);
    Serial.print(" rel="); Serial.print(pr_rel[p], 2);
    Serial.print(" plev="); Serial.print(pr_lev[p], 2);
    Serial.print(" p1="); Serial.print(pr_p1[p], 3);
    Serial.print(" p2="); Serial.print(pr_p2[p], 3);
    Serial.print(" p3="); Serial.println(pr_p3[p], 3);
  }
  Serial.println("drums:");
  for (int p = P_KICK; p <= P_TOM; p++) drumShow(p);
}

static void spd() {
  Serial.print("smode="); Serial.print(smode);
  Serial.print(" running="); Serial.print(fld_running);
  Serial.print(" EN="); Serial.print(en_state);
  Serial.print(" target="); Serial.print(target_rpm, 1);
  Serial.print(" field_rpm="); Serial.print(fld_speed, 1);
  Serial.print(" meas_rpm="); Serial.print(meas_rpm, 1);
  Serial.print(" field="); Serial.print(fld_amp * vbus, 2); Serial.print("V");
  Serial.print(" lag="); Serial.print(fld_lag * RAD2DEG, 0);
  Serial.print(" resync="); Serial.print(resync_cnt);
  Serial.print(" bpm="); Serial.print(cur_bpm, 1);
  Serial.print(" play="); Serial.println(sync_play);
}

void help() {
  Serial.println("--- <name> <value>, several with ;   (st = all settings, save = store them) ---");
  Serial.println("show:   st  id  plist  spd  hall  htab  stat  statclr  cpu <1 start / 0 stop+print>  crash  help");
  Serial.println("do:     save  factory  panic  cal");
  Serial.println("midi:   mvol <0-100>  poly <4-16>  patch <1-7>  atk dec sus rel plev p1 p2 p3  a4  vcurve  bend  lfo  vdep  pc <0/1>");
  Serial.println("        patches: 1 epiano 2 fm-epiano 3 organ 4 guitar 5 saw 6 square 7 triangle   (volume: 100 = imax as calculated with R_EFF, -10 dB per 25, 0 = off)");
  Serial.println("drum:   drum <0/1>  dlev  dvol <note> <x>  ddec <note> <s>  dfreq <note> <Hz>  dsweep <kick/tom> <x>  dstime <kick/tom> <s>");
  Serial.println("eq:     m... for MIDI, a... for audio:  mhpf mlpf <Hz, 0 off>  mbass <dB> mbfreq <Hz>  mtreble <dB> mtfreq <Hz>  mmid <1-4> <Hz> <dB> [Q]   (dB -30..30, Hz 20..20000)");
  Serial.println("audio:  avol <0-100>  role <0 one motor / 1 right / 2 left>  src <0 L+R 1 L 2 R>  usbvol <0/1>  tone <Hz, 0 off>");
  Serial.println("spin:   smode <0 off 1 constant 2 note-rate 3 level>  orpm  ostr <V>  hold <s>  sdir <1/-1>");
  Serial.println("        tsync <0/1>  tbase  tgain  tmin  tmax | nbase  nk  ntau");
  Serial.println("hall:   hmode <0/1>  calb <V>  bias <V>  toff <deg>  hallmon <0/1>");
  Serial.println("drive:  vbus <V>  imax <A>  pwmf <22059 14706 11029 8824>  dtm <0/1>  dtns <ns>  dti <A>  hfield <V>  lmode <0 clean/1 loud>  limrel  lmodel  vmax");
  Serial.println("sent to LED board:  spec <0-3>  sgain  srange  stilt  sjoin <0/1>  ssplit <note>  sonset <0/1>  sharm <0/1>");
  Serial.println("misc:   echo <0/1>  mon <0/1>");
}

// ---------------------------------------------------------------- commands
static int parseNums(char* s, float* out, int maxn) {
  int n = 0;
  if (!s) return 0;
  while (*s && n < maxn) {
    while (*s == ' ') s++;
    if (!*s) break;
    char* e; out[n] = strtod(s, &e);
    if (e == s) break;
    n++; s = e;
  }
  return n;
}

// m.../a... EQ commands. Returns false if the name is not an EQ command.
static bool eqCmd(const char* line, const float* nv, int nn) {
  if (line[0] != 'm' && line[0] != 'a') return false;
  const char* k = line + 1;
  EqSet &e = (line[0] == 'm') ? eq_m : eq_a;
  if (!strcmp(k, "mid")) {
    if (nn < 3) { Serial.print("usage: "); Serial.print(line); Serial.println(" <1-4> <Hz> <dB> [Q]"); return true; }
    int b = constrain((int)nv[0], 1, NMID) - 1;
    e.mid[b].f = constrain(nv[1], EQ_FMIN, EQ_FMAX);
    e.mid[b].g = constrain(nv[2], -EQ_DB, EQ_DB);
    if (nn >= 4) e.mid[b].q = constrain(nv[3], 0.3f, 10.0f);
  } else {
    if (strcmp(k, "hpf") && strcmp(k, "lpf") && strcmp(k, "bass") && strcmp(k, "bfreq") && strcmp(k, "treble") && strcmp(k, "tfreq")) return false;
    if (nn < 1) { Serial.print("need value: "); Serial.println(line); return true; }
    const float v = nv[0];
    if      (!strcmp(k, "hpf"))    e.hpf = (v <= 0.0f) ? 0.0f : constrain(v, EQ_FMIN, EQ_FMAX);
    else if (!strcmp(k, "lpf"))    e.lpf = (v <= 0.0f) ? 0.0f : constrain(v, EQ_FMIN, EQ_FMAX);
    else if (!strcmp(k, "bass"))   e.bass = constrain(v, -EQ_DB, EQ_DB);
    else if (!strcmp(k, "bfreq"))  e.bfreq = constrain(v, EQ_FMIN, EQ_FMAX);
    else if (!strcmp(k, "treble")) e.treble = constrain(v, -EQ_DB, EQ_DB);
    else                           e.tfreq = constrain(v, EQ_FMIN, EQ_FMAX);
  }
  applyFilters();
  eqLine(line[0] == 'm' ? "meq:  " : "aeq:  ", e);
  return true;
}

static void processCmd(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  int L = strlen(line);
  while (L > 0 && (line[L - 1] == ' ' || line[L - 1] == '\t')) line[--L] = 0;
  if (L == 0) return;
  char* sp = strchr(line, ' ');
  char* rest = nullptr;
  if (sp) { *sp = 0; rest = sp + 1; }
  float nv[4]; int nn = parseNums(rest, nv, 4);
  float val = (nn > 0) ? nv[0] : 0; bool hasVal = (nn > 0);

  // commands without a value
  if (!strcmp(line, "st") || !strcmp(line, "status") || !strcmp(line, "s")) { status(); return; }
  if (!strcmp(line, "help") || !strcmp(line, "?")) { help(); return; }
  if (!strcmp(line, "id"))      { id(); return; }
  if (!strcmp(line, "plist"))   { plist(); return; }
  if (!strcmp(line, "spd"))     { spd(); return; }
  if (!strcmp(line, "hall"))    { printHall(); return; }
  if (!strcmp(line, "htab"))    { printTable(); return; }
  if (!strcmp(line, "stat"))    { stat(); return; }
  if (!strcmp(line, "statclr")) { statClear(); Serial.println("stat cleared"); return; }
  if (!strcmp(line, "crash"))   { if (crash_len) Serial.print(crash_txt); else Serial.println("crash: no report from the last run"); return; }
  if (!strcmp(line, "save"))    { settingsSave(); return; }
  if (!strcmp(line, "factory")) { settingsFactory(); return; }
  if (!strcmp(line, "panic"))   { allOff(); Serial.println("panic"); return; }
  if (!strcmp(line, "cal"))     { runCal(); return; }

  if (eqCmd(line, nv, nn)) return;

  // drums: <note> <value>
  if (!strcmp(line, "dsweep") || !strcmp(line, "dstime")) {
    if (nn < 2) { Serial.print("usage: "); Serial.print(line); Serial.println(" <kick/tom note> <value>"); return; }
    int p = drumPreset((uint8_t)constrain((int)nv[0], 0, 127));
    if (p < 0 || pr_wave[p] != W_KICK) { Serial.println("kick/tom notes only"); return; }
    if (line[2] == 'w') pr_p3[p] = constrain(nv[1], 0.0f, 10.0f);
    else { pr_p2[p] = constrain(nv[1], 0.005f, 1.0f); updateEnv(); }
    drumShow(p);
    return;
  }
  if (!strcmp(line, "dvol") || !strcmp(line, "ddec") || !strcmp(line, "dfreq")) {
    if (nn < 2) { Serial.print("usage: "); Serial.print(line); Serial.println(" <drum note> <value>"); return; }
    int p = drumPreset((uint8_t)constrain((int)nv[0], 0, 127));
    if (p < 0) { Serial.print("not a drum note: "); Serial.println((int)nv[0]); return; }
    if (line[1] == 'v') pr_lev[p] = constrain(nv[1], 0.0f, 4.0f);
    else if (line[1] == 'd') { pr_dec[p] = constrain(nv[1], 0.01f, 5.0f); updateEnv(); }
    else {
      bool tonal = (pr_wave[p] == W_KICK);
      pr_p1[p] = constrain(nv[1], tonal ? 20.0f : 100.0f, tonal ? 1000.0f : 40000.0f);
    }
    drumShow(p);
    return;
  }

  if (!hasVal) { Serial.print("need value (or unknown): "); Serial.println(line); return; }

  // midi
  if      (!strcmp(line, "mvol"))   { mvol_v = constrain(val, 0.0f, 100.0f); updateLevels(); }
  else if (!strcmp(line, "poly"))   poly = constrain((int)val, 4, MAX_VOICES);
  else if (!strcmp(line, "patch"))  { int p = constrain((int)val, 1, NPRE) - 1; for (int c = 0; c <= 16; c++) ch_pre[c] = p; edit_p = p; }
  else if (!strcmp(line, "atk"))    { pr_atk[edit_p] = constrain(val, 0.0005f, 1.0f); updateEnv(); }
  else if (!strcmp(line, "dec"))    { pr_dec[edit_p] = constrain(val, 0.02f, 20.0f);  updateEnv(); }
  else if (!strcmp(line, "sus"))    pr_sus[edit_p] = constrain(val, 0.0f, 1.0f);
  else if (!strcmp(line, "rel"))    { pr_rel[edit_p] = constrain(val, 0.01f, 5.0f);   updateEnv(); }
  else if (!strcmp(line, "plev"))   pr_lev[edit_p] = constrain(val, 0.0f, 4.0f);
  else if (!strcmp(line, "p1")) {
    float lo = 0.0f, hi = 20.0f;
    if (pr_wave[edit_p] == W_PULSE) { lo = 0.05f; hi = 0.95f; }
    if (pr_wave[edit_p] == W_TRI)   { lo = 0.0f;  hi = 32.0f; }
    pr_p1[edit_p] = constrain(val, lo, hi); updateEnv();
  }
  else if (!strcmp(line, "p2"))     { pr_p2[edit_p] = constrain(val, 0.0f, 20.0f); updateEnv(); }
  else if (!strcmp(line, "p3"))     { pr_p3[edit_p] = constrain(val, 0.0f, 20.0f); updateEnv(); }
  else if (!strcmp(line, "a4"))     A4_HZ = constrain(val, 400.0f, 480.0f);
  else if (!strcmp(line, "vcurve")) vcurve = constrain(val, 0.2f, 4.0f);
  else if (!strcmp(line, "bend"))   { bend_range = constrain(val, 0.0f, 24.0f); chSetBend(bend_range); }
  else if (!strcmp(line, "lfo"))    lfo_hz = constrain(val, 0.1f, 20.0f);
  else if (!strcmp(line, "vdep"))   { vdep = constrain(val, 0.0f, 2.0f); chUpdateAll(); }
  else if (!strcmp(line, "pc"))     pc_on = (val != 0);
  else if (!strcmp(line, "drum"))   drum_on = (val != 0);
  else if (!strcmp(line, "dlev"))   dlev = constrain(val, 0.0f, 4.0f);
  // audio
  else if (!strcmp(line, "avol"))   { avol_v = constrain(val, 0.0f, 100.0f); updateLevels(); }
  else if (!strcmp(line, "role"))   { role = constrain((int)val, 0, 2); rb_r = rb_w; }
  else if (!strcmp(line, "src"))    src = constrain((int)val, 0, 2);
  else if (!strcmp(line, "usbvol")) usbvol = val != 0;
  else if (!strcmp(line, "tone"))   tone_hz = constrain(val, 0.0f, 10000.0f);
  // spin
  else if (!strcmp(line, "smode")) {
    int m = constrain((int)val, 0, 3);
    if (m != 0 && !(hall_ok && geom_ok)) { Serial.println("smode: run cal first"); return; }
    smode = m;
    if (m != 0) hall_mode = true;
  }
  else if (!strcmp(line, "orpm"))   const_rpm = constrain(val, 0.0f, 120.0f);
  else if (!strcmp(line, "ostr"))   { ostr_v = constrain(val, 0.0f, 0.85f); updateLevels(); }
  else if (!strcmp(line, "hold"))   hold_s = constrain(val, 0.0f, 30.0f);
  else if (!strcmp(line, "sdir"))   spin_dir = (val < 0) ? -1 : 1;
  else if (!strcmp(line, "tsync"))  tsync = (val != 0);
  else if (!strcmp(line, "tbase"))  tbase = constrain(val, 20.0f, 300.0f);
  else if (!strcmp(line, "tgain"))  tgain = constrain(val, 0.5f, 5.0f);
  else if (!strcmp(line, "tmin"))   tmin_rpm = constrain(val, 0.0f, 30.0f);
  else if (!strcmp(line, "tmax"))   tmax_rpm = constrain(val, 1.0f, 60.0f);
  else if (!strcmp(line, "nbase"))  nbase = constrain(val, 0.0f, 30.0f);
  else if (!strcmp(line, "nk"))     nk = constrain(val, 0.0f, 20.0f);
  else if (!strcmp(line, "ntau"))   ntau = constrain(val, 0.05f, 5.0f);
  // hall
  else if (!strcmp(line, "hmode")) {
    if (val != 0 && !hall_ok) { Serial.println("hmode: run cal first"); return; }
    hall_mode = (val != 0);
  }
  else if (!strcmp(line, "calb"))   { calb_v = constrain(val, 0.0f, 1.8f); updateLevels(); }
  else if (!strcmp(line, "bias"))   { bias_v = constrain(val, 0.0f, 3.6f); updateLevels(); }
  else if (!strcmp(line, "toff"))   theta_off = constrain(val, -180.0f, 180.0f) / RAD2DEG;
  else if (!strcmp(line, "hallmon")) { hallmon = (val != 0); hall_prev = 255; }
  // drive
  else if (!strcmp(line, "vbus"))   { vbus = constrain(val, 8.0f, 30.0f); updateLimit(); updateLevels(); }
  else if (!strcmp(line, "imax"))   { i_max = constrain(val, 0.2f, IMAX_LIMIT); updateLimit(); updateLevels(); }
  else if (!strcmp(line, "pwmf")) {
    int k = (val > 0.0f) ? (int)lroundf(FS / val) : PWM_DIV_MIN;
    pwm_div = constrain(k, PWM_DIV_MIN, PWM_DIV_MAX);
    applyEngine();
  }
  else if (!strcmp(line, "dtm"))    dt_mode = (val != 0) ? 1 : 0;
  else if (!strcmp(line, "dtns"))   { dt_ns = constrain(val, 0.0f, 1500.0f); dtUpdate(); }
  else if (!strcmp(line, "dti"))    dti = constrain(val, 0.02f, 1.0f);
  else if (!strcmp(line, "hfield")) hfield_v = constrain(val, 0.0f, 0.85f);
  else if (!strcmp(line, "lmode"))  lmode = (val != 0) ? 1 : 0;
  else if (!strcmp(line, "limrel")) { lim_rel_s = constrain(val, 0.01f, 2.0f); updateLimit(); }
  else if (!strcmp(line, "lmodel")) { lmodel_hz = constrain(val, 50.0f, 5000.0f); updateLimit(); }
  else if (!strcmp(line, "vmax"))   { vmax_set = constrain(val, 0.0f, 15.0f); updateLimit(); }
  // led
  else if (!strcmp(line, "spec"))   spec_mode = constrain((int)val, 0, 3);
  else if (!strcmp(line, "sgain"))  sp_gain = constrain(val, -30.0f, 40.0f);
  else if (!strcmp(line, "srange")) sp_range = constrain(val, 10.0f, 80.0f);
  else if (!strcmp(line, "stilt"))  sp_tilt = constrain(val, 0.0f, 6.0f);
  else if (!strcmp(line, "sjoin"))  sp_join = (val != 0) ? 1 : 0;
  else if (!strcmp(line, "ssplit")) sp_split = constrain((int)val, 36, 96);
  else if (!strcmp(line, "sonset")) sp_onset = (val != 0) ? 1 : 0;
  else if (!strcmp(line, "sharm"))  sp_harm = (val != 0) ? 1 : 0;
  // misc
  else if (!strcmp(line, "echo"))   echo = (val != 0);
  else if (!strcmp(line, "mon"))    mon = (val != 0);
  else if (!strcmp(line, "cpu"))    { if (val != 0) cpuStart(); else cpuStop(); return; }
  else { Serial.print("? unknown: "); Serial.println(line); return; }

  status();
}

// ---------------------------------------------------------------- main loop helpers
static void checkFault() {
  bool f = (digitalRead(PIN_FAULT) == LOW);
  if (f != fault_prev) {
    fault_prev = f;
    if (f) Serial.println("FAULT (if it stays, power-cycle the motor supply)");
    else   Serial.println("fault clear");
  }
}

static void checkHallErr() {
  if (hall_err && !hall_err_shown) {
    hall_err_shown = true;
    Serial.print("HALL ERROR (hall="); printBits(hall_state);
    Serial.println("): rotation stopped. check hall 5V / GND / wiring");
  } else if (!hall_err && hall_err_shown) {
    hall_err_shown = false;
    Serial.println("hall ok: rotation enabled");
  }
}

static void checkHallMon() {
  if (!hallmon) return;
  uint8_t s = hall_state;
  if (s != hall_prev) { hall_prev = s; printHall(); }
}

void checkNodeName() {
  if (role != 0 && role != MOTOR_NUMBER) {
    Serial.print("WARNING: config.h says MOTOR_NUMBER "); Serial.print(MOTOR_NUMBER);
    Serial.print(" but the saved role is "); Serial.print(role);
    Serial.println(". Check MOTOR_NUMBER, or type role <n>; save");
  }
}

void consoleTask() {
  checkFault();
  checkHallMon();
  checkHallErr();
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r' || ch == ';') {
      if (cmdlen > 0) { cmdbuf[cmdlen] = 0; processCmd(cmdbuf); cmdlen = 0; }
    } else if (ch == 8 || ch == 127) {
      if (cmdlen > 0) cmdlen--;
    } else if (ch >= 32 && ch <= 126 && cmdlen < sizeof(cmdbuf) - 1) {
      cmdbuf[cmdlen++] = ch;
    }
  }
}
