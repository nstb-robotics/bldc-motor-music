// BLDC Motor Music - motor firmware for SimpleFOCMini (DRV8313)
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// Plays MIDI (built-in 16-voice synth) and PC audio (USB audio device)
// through the motor itself, and can turn the wheel while it plays.
// Board: Teensy 4.0.  Arduino IDE: Tools > USB Type > Serial + MIDI + Audio.
// Before flashing, set MOTOR_NUMBER in config.h.

#include "motor_music.h"

const char* FW_VER = "1.0";
char crash_txt[1536];
uint16_t crash_len = 0;

class CrashBuf : public Print {
public:
  size_t write(uint8_t c) override { if (crash_len < sizeof(crash_txt) - 1) crash_txt[crash_len++] = c; return 1; }
};

void setup() {
  if (CrashReport) { CrashBuf cb; cb.print(CrashReport); crash_txt[crash_len] = 0; }
  Serial.begin(115200);
  Serial1.begin(115200);                       // to the LED board
  Serial1.addMemoryForWrite(led_txmem, sizeof(led_txmem));
  Serial5.begin(LINK_BAUD);                    // stereo link between the motor boards
  Serial5.addMemoryForWrite(link_txmem, sizeof(link_txmem));
  Serial5.addMemoryForRead(link_rxmem, sizeof(link_rxmem));
  specInit();
  AudioMemory(16);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);
  pinMode(PIN_FAULT, INPUT_PULLUP);
  pinMode(PIN_HA, INPUT_PULLUP);
  pinMode(PIN_HB, INPUT_PULLUP);
  pinMode(PIN_HC, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;               // cycle counter for stat / cpu
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  cpu_cyc_us = F_CPU_ACTUAL / 1000000;
  analogWriteResolution(16);

  while (!Serial && millis() < 3000) {}
  if (crash_len) Serial.println("WARNING: the last run stopped with a crash (type crash to show the report)");
  settingsDefaults();
  settingsLoad();
  checkNodeName();
  allOff();
  calLoad();
  updateLevels();

  usbMIDI.setHandleNoteOn(noteOn);
  usbMIDI.setHandleNoteOff(noteOff);
  usbMIDI.setHandleControlChange(onControl);
  usbMIDI.setHandlePitchChange(onBend);
  usbMIDI.setHandleProgramChange(onProgram);
  usbMIDI.setHandleClock(onClock);
  usbMIDI.setHandleStart(onStart);
  usbMIDI.setHandleContinue(onContinue);
  usbMIDI.setHandleStop(onStop);
  usbMIDI.setHandleTimeCodeQuarterFrame(onMtc);
  usbMIDI.setHandleSongPosition(onSongPos);
  usbMIDI.setHandleSystemExclusive(onSysex);

  applyEngine();
  help();
  status();
}

void loop() {
  const uint32_t c0 = ARM_DWT_CYCCNT;
  if (role == 2) linkRx();
  specTask();
  if (role == 2) linkRx();
  {
    static uint32_t t0 = millis(), k0 = isr_ticks;
    uint32_t now = millis();
    if (now - t0 >= 2000) { uint32_t k = isr_ticks; isr_hz_meas = (float)(k - k0) * 1000.0f / (float)(now - t0); t0 = now; k0 = k; }
  }
  while (usbMIDI.read()) {}
  syncTask();
  consoleTask();
  cpuLoop(ARM_DWT_CYCCNT - c0);
}
