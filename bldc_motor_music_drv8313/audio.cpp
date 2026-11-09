// BLDC Motor Music - USB audio input, stereo link, test tone
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// The PC sees each board as a USB audio device. Role 1 (Motor 1) plays the
// right channel and forwards the left channel to role 2 over Serial5.
// Role 0 plays one motor alone (src chooses L+R, L or R).

#include "motor_music.h"

volatile float amaster = 0.05f;
float avol_v = 92.5f;
volatile int src = 0;
volatile int role = 0;
volatile bool usbvol = true;
volatile float tone_hz = 0.0f;
uint8_t link_txmem[2048], link_rxmem[4096];
volatile uint32_t lk_tx = 0, lk_drop = 0, lk_rx = 0, lk_bad = 0, lk_gap = 0;
volatile uint8_t lk_vol = 255;
int16_t rb[RB_N];
int16_t rbm[RB_N];
volatile uint32_t rb_w = 0, rb_r = 0;
volatile float rate_now = RATE_NOM;
volatile bool a_playing = false;
volatile bool a_active = false;
volatile uint32_t last_blk_ms = 0;
volatile uint32_t n_blk = 0, n_over = 0, n_under = 0, n_upd = 0;
volatile uint32_t fill_min = 0xFFFFFFFF, fill_max = 0;
volatile float pk_in = 0.0f;
bool AudioOutputMotor::update_responsibility = false;

static void rbPush(int16_t v) {
  uint32_t w = rb_w;
  if (w - rb_r < (uint32_t)RB_N) { rb[w % RB_N] = v; rb_w = w + 1; }
  else n_over++;
}

void AudioOutputMotor::update(void) {
  audio_block_t *l = receiveReadOnly(0);
  audio_block_t *r = receiveReadOnly(1);
  n_upd++;
  if ((l || r) && role != 2) {
    int s0 = (role == 1) ? 2 : src;
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      int32_t a = l ? l->data[i] : 0, b = r ? r->data[i] : 0;
      int32_t s = (s0 == 1) ? a : (s0 == 2) ? b : ((a + b) >> 1);
      rbm[rb_w % RB_N] = (int16_t)((a + b) >> 1);
      rbPush((int16_t)s);
    }
    if (role == 1) {
      static uint8_t pkt[2 + 2 + 2 * AUDIO_BLOCK_SAMPLES + 1];
      static uint8_t seq = 0;
      if (Serial5.availableForWrite() >= (int)sizeof(pkt)) {
        pkt[0] = 0xA5; pkt[1] = 0x5A; pkt[2] = seq++;
        pkt[3] = (uint8_t)(usbIn.volume() * 255.0f + 0.5f);
        uint8_t ck = pkt[2] + pkt[3];
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
          int16_t v = l ? l->data[i] : 0;
          uint8_t lo = (uint8_t)(v & 0xFF), hi = (uint8_t)((uint16_t)v >> 8);
          pkt[4 + 2 * i] = lo; pkt[5 + 2 * i] = hi; ck += lo + hi;
        }
        pkt[sizeof(pkt) - 1] = ck;
        Serial5.write(pkt, sizeof(pkt));
        lk_tx++;
      } else lk_drop++;
    }
    n_blk++;
    last_blk_ms = millis();
  }
  if (l) release(l);
  if (r) release(r);
}

void linkRx() {
  static uint8_t st = 0, buf[2 + 2 * AUDIO_BLOCK_SAMPLES + 1];
  static int n = 0;
  static int last_seq = -1;
  const int PL = sizeof(buf);
  int k = 0;
  while (Serial5.available() && k++ < 2048) {
    uint8_t c = Serial5.read();
    if (st == 0) { if (c == 0xA5) st = 1; }
    else if (st == 1) { st = (c == 0x5A) ? 2 : (c == 0xA5 ? 1 : 0); n = 0; }
    else {
      buf[n++] = c;
      if (n == PL) {
        st = 0;
        uint8_t ck = 0;
        for (int i = 0; i < PL - 1; i++) ck += buf[i];
        if (ck != buf[PL - 1]) { lk_bad++; continue; }
        if (last_seq >= 0 && buf[0] != (uint8_t)(last_seq + 1)) lk_gap++;
        last_seq = buf[0];
        lk_vol = buf[1];
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) rbPush((int16_t)(buf[2 + 2 * i] | (buf[3 + 2 * i] << 8)));
        lk_rx++; n_blk++;
        last_blk_ms = millis();
      }
    }
  }
}

float audioRender(bool &have) {
  float x = 0.0f;
  bool a_have = false;
  if (tone_hz > 0.0f) {
    static float ph = 0.0f;
    ph += tone_hz / FS; if (ph >= 1.0f) ph -= 1.0f;
    x = 0.5f * sinf(TWO_PI_F * ph);
    a_have = true;
    rb_r = rb_w; a_playing = false;
  } else {
    // ring buffer -> cubic interpolation, rate follows the buffer fill (PC clock drift)
    uint32_t w = rb_w, r = rb_r, fill = w - r;
    static float frac = 0.0f, fill_s = FILL_TGT;
    if (!a_playing && fill >= (uint32_t)FILL_TGT) { a_playing = true; frac = 0.0f; fill_s = (float)fill; }
    if (a_playing) {
      if (fill < 4) { a_playing = false; n_under++; }
      else {
        fill_s += ((float)fill - fill_s) * 5e-5f;
        float rt = RATE_NOM + KP * (fill_s - FILL_TGT);
        if (rt < 0.99f) rt = 0.99f;
        if (rt > 1.01f) rt = 1.01f;
        rate_now = rt;
        float p0 = rb[(r - 1) % RB_N], p1 = rb[r % RB_N], p2 = rb[(r + 1) % RB_N], p3 = rb[(r + 2) % RB_N];
        float t = frac;
        x = (p1 + 0.5f * t * (p2 - p0 + t * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3 + t * (3.0f * (p1 - p2) + p3 - p0)))) * (1.0f / 32768.0f);
        frac += rt;
        uint32_t adv = (uint32_t)frac;
        frac -= (float)adv;
        rb_r = r + adv;
        a_have = true;
        if (fill < fill_min) fill_min = fill;
        if (fill > fill_max) fill_max = fill;
      }
    }
  }
  if (sp_joined && tone_hz <= 0.0f) specPush(a_have ? rbm[(rb_r - 1) % RB_N] * (1.0f / 32768.0f) : 0.0f);
  else specPush(x);
  {
    static uint32_t act = 0;
    if (a_have && fabsf(x) > 0.003f) act = (uint32_t)(ACT_HOLD_S * FS);
    else if (act) act--;
    a_active = a_have && act > 0;
  }
  if (usbvol && tone_hz <= 0.0f) x *= (role == 2) ? lk_vol * (1.0f / 255.0f) : usbIn.volume();
  float axx = fabsf(x);
  if (axx > pk_in) pk_in = axx;
  // audio EQ; stops 0.5 s after the input stops
  static uint32_t a_tail = 0;
  if (a_have) a_tail = (uint32_t)(EQ_TAIL_S * FS);
  if (a_tail) {
    x = eqRun(run_a, x);
    if (!a_have && --a_tail == 0) eqClear(run_a);
  }
  have = a_have;
  return x;
}

AudioInputUSB usbIn;
AudioOutputMotor motOut;
AudioConnection pc1(usbIn, 0, motOut, 0);
AudioConnection pc2(usbIn, 1, motOut, 1);
