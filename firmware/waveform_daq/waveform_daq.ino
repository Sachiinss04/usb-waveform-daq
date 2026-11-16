/*
 * waveform_daq.ino -- USB waveform generator + data acquisition (Arduino Uno / ATmega328P, 16 MHz)
 *
 * Outputs
 *   D9  (OC1A, Timer1 fast-PWM mode 14): square wave, frequency AND duty configurable
 *   D3  (OC2B, Timer2 fast-PWM @ 62.5 kHz carrier): sine / triangle via DDS; needs an RC low-pass
 *       filter (see docs/circuit.png) to turn the PWM into an analog waveform
 * Input
 *   A0  (ADC, 10-bit, AVcc reference): sampled by hardware auto-trigger from Timer0 so the
 *       sample period is exact and jitter-free; samples stream to the PC over USB serial.
 *
 * Serial: 115200 8N1. PC -> MCU: ASCII commands, one per line. MCU -> PC: ASCII replies
 * ("OK ..." / "ERR ...") plus, while streaming, 2 bytes per sample (see docs/PROTOCOL.md).
 *
 * NOTE: Timer0 is reprogrammed, so delay()/millis() do not work in this sketch (not used).
 */
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

// ---------------- configuration ----------------
static const float DDS_RATE_HZ   = 16000000.0f / 256.0f / 8.0f;      // 7812.5 Hz update rate
static const float DDS_STEP_PER_HZ = 4294967296.0f / DDS_RATE_HZ;    // phase step per Hz
static const uint32_t ADC_TICK_HZ = 250000UL;                        // Timer0: 16 MHz / 64
static const uint16_t ADC_DIV_MIN = 50;                              // 5000 S/s max
static const uint16_t ADC_DIV_MAX = 256;                             // ~977 S/s min

// ---------------- sine table (256 x 8 bit) in flash ----------------
static const uint8_t SINE[256] PROGMEM = {
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
  245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
  255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249, 248, 246,
  245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220,
  218, 215, 213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 188, 185, 182, 179,
  176, 173, 170, 167, 165, 162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131,
  128, 124, 121, 118, 115, 112, 109, 106, 103, 100, 97, 93, 90, 88, 85, 82,
  79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40,
  37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11,
  10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0,
  0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9,
  10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35,
  37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76,
  79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124,
};

// ---------------- DDS state (Timer2 overflow ISR) ----------------
enum { WF_OFF = 0, WF_SINE = 1, WF_TRI = 2 };
static volatile uint8_t  wf_shape = WF_OFF;
static volatile uint32_t dds_phase = 0, dds_step = 0;
static volatile uint16_t dds_amp = 256;       // 256 = 100 %
static volatile uint8_t  dds_div = 0;
static float dds_freq_actual = 0.0f;

ISR(TIMER2_OVF_vect) {
  if ((++dds_div & 7) != 0) return;           // update every 8th overflow = 7812.5 Hz
  uint8_t shape = wf_shape;
  if (shape == WF_OFF) return;
  dds_phase += dds_step;
  uint8_t v;
  if (shape == WF_SINE) {
    v = pgm_read_byte(&SINE[dds_phase >> 24]);
  } else {
    uint16_t p = (uint16_t)(dds_phase >> 23);  // 0..511
    v = (p < 256) ? (uint8_t)p : (uint8_t)(511 - p);
  }
  int16_t d = ((int16_t)v - 128) * (int16_t)dds_amp;
  OCR2B = (uint8_t)(128 + (d >> 8));
}

// ---------------- ADC sampling (Timer0 compare-match trigger) ----------------
static const uint8_t RING_N = 128;            // power of two; 128 x 2 B = 256 B
static volatile uint16_t ring[RING_N];
static volatile uint8_t  ring_head = 0, ring_tail = 0;
static volatile uint16_t ring_drops = 0;
static volatile bool     streaming = false;
static uint16_t adc_div = 125;                // 2000 S/s default

ISR(ADC_vect) {
  uint16_t v = ADC;
  TIFR0 = _BV(OCF0A);                         // clear trigger flag so the next match re-triggers
  uint8_t next = (ring_head + 1) & (RING_N - 1);
  if (next == ring_tail) { ring_drops++; return; }
  ring[ring_head] = v;
  ring_head = next;
}

static float adc_rate() { return (float)ADC_TICK_HZ / adc_div; }

static void adc_start() {
  ring_head = ring_tail = 0;
  OCR0A  = adc_div - 1;
  TCCR0A = _BV(WGM01);                        // CTC
  TCCR0B = _BV(CS01) | _BV(CS00);             // /64
  TCNT0  = 0;
  TIFR0  = _BV(OCF0A);
  ADMUX  = _BV(REFS0);                        // AVcc reference, channel A0
  ADCSRB = _BV(ADTS1) | _BV(ADTS0);           // trigger: Timer0 compare match A
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // /128 = 125 kHz
  streaming = true;
}

static void adc_stop() {
  ADCSRA &= ~(_BV(ADATE) | _BV(ADIE));
  streaming = false;
}

// ---------------- generators ----------------
static float sq_freq_actual = 0.0f, sq_duty_actual = 0.0f;

static void square_off() {
  TCCR1B = 0; TCCR1A = 0;
  PORTB &= ~_BV(PB1);
  sq_freq_actual = 0;
}

static bool square_set(float f, float duty) {
  if (f < 0.25f || f > 100000.0f || duty < 0.0f || duty > 100.0f) return false;
  static const uint16_t pre[5] = {1, 8, 64, 256, 1024};
  static const uint8_t  cs[5]  = {1, 2, 3, 4, 5};
  for (uint8_t i = 0; i < 5; i++) {
    float top = 16000000.0f / ((float)pre[i] * f) - 1.0f;
    if (top >= 1.0f && top <= 65535.0f) {
      uint16_t icr = (uint16_t)(top + 0.5f);
      uint16_t ocr = (uint16_t)(((float)icr + 1.0f) * duty / 100.0f + 0.5f);
      if (ocr > icr) ocr = icr;
      TCCR1B = 0;
      TCNT1 = 0;
      ICR1 = icr;
      OCR1A = ocr;
      DDRB |= _BV(PB1);
      TCCR1A = _BV(COM1A1) | _BV(WGM11);                  // mode 14, non-inverting on OC1A
      TCCR1B = _BV(WGM13) | _BV(WGM12) | cs[i];
      sq_freq_actual = 16000000.0f / ((float)pre[i] * ((float)icr + 1.0f));
      sq_duty_actual = 100.0f * (float)ocr / ((float)icr + 1.0f);
      return true;
    }
  }
  return false;
}

static bool dds_set(uint8_t shape, float f, float amp_pct) {
  if (f < 0.1f || f > 1000.0f || amp_pct < 0.0f || amp_pct > 100.0f) return false;
  uint32_t step = (uint32_t)(f * DDS_STEP_PER_HZ + 0.5f);
  noInterrupts();
  dds_step = step;
  dds_amp = (uint16_t)(amp_pct * 2.56f + 0.5f);
  wf_shape = shape;
  interrupts();
  dds_freq_actual = (float)step / DDS_STEP_PER_HZ;
  return true;
}

static void dds_off() {
  wf_shape = WF_OFF;
  OCR2B = 0;
  dds_freq_actual = 0;
}

// ---------------- command parser ----------------
static char line[40];
static uint8_t line_len = 0;

static void reply_float(const __FlashStringHelper *tag, float v, uint8_t dp) {
  Serial.print(tag); Serial.println(v, dp);
}

static void handle(char *s) {
  char *cmd = strtok(s, " ");
  if (!cmd) return;
  char *a1 = strtok(NULL, " ");
  char *a2 = strtok(NULL, " ");
  float x = a1 ? atof(a1) : 0.0f;
  float y = a2 ? atof(a2) : -1.0f;

  if (!strcasecmp(cmd, "SQ")) {
    if (!a1 || !square_set(x, a2 ? y : 50.0f)) { Serial.println(F("ERR sq f[0.25..100000] duty[0..100]")); return; }
    Serial.print(F("OK SQ f=")); Serial.print(sq_freq_actual, 3);
    Serial.print(F(" duty=")); Serial.println(sq_duty_actual, 2);
  } else if (!strcasecmp(cmd, "SIN") || !strcasecmp(cmd, "TRI")) {
    uint8_t sh = (cmd[0] == 'S' || cmd[0] == 's') ? WF_SINE : WF_TRI;
    if (!a1 || !dds_set(sh, x, a2 ? y : 100.0f)) { Serial.println(F("ERR f[0.1..1000] amp[0..100]")); return; }
    Serial.print(F("OK ")); Serial.print(sh == WF_SINE ? F("SIN") : F("TRI"));
    Serial.print(F(" f=")); Serial.println(dds_freq_actual, 4);
  } else if (!strcasecmp(cmd, "OFF")) {
    square_off(); dds_off();
    Serial.println(F("OK OFF"));
  } else if (!strcasecmp(cmd, "RATE")) {
    if (!a1 || x < 100.0f) { Serial.println(F("ERR rate hz")); return; }
    long d = (long)((float)ADC_TICK_HZ / x + 0.5f);
    if (d < ADC_DIV_MIN) d = ADC_DIV_MIN;
    if (d > ADC_DIV_MAX) d = ADC_DIV_MAX;
    adc_div = (uint16_t)d;
    reply_float(F("OK RATE fs="), adc_rate(), 3);
  } else if (!strcasecmp(cmd, "START")) {
    adc_start();
    reply_float(F("OK START fs="), adc_rate(), 3);
  } else if (!strcasecmp(cmd, "STOP")) {
    adc_stop();
    Serial.print(F("OK STOP drops=")); Serial.println(ring_drops);
  } else if (!strcasecmp(cmd, "INFO")) {
    Serial.print(F("OK INFO fs=")); Serial.print(adc_rate(), 3);
    Serial.print(F(" sq=")); Serial.print(sq_freq_actual, 3);
    Serial.print(F(" wf=")); Serial.print(wf_shape);
    Serial.print(F(" f=")); Serial.print(dds_freq_actual, 4);
    Serial.print(F(" drops=")); Serial.println(ring_drops);
  } else if (!strcasecmp(cmd, "ID")) {
    Serial.println(F("OK ID waveform_daq 1.0 uno"));
  } else {
    Serial.println(F("ERR unknown"));
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  // Timer2: fast PWM 8-bit, no prescale -> 62.5 kHz carrier on D3 (OC2B)
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2B = 0;
  TIMSK2 = _BV(TOIE2);
  Serial.println(F("OK READY waveform_daq"));
}

void loop() {
  // drain sample ring buffer: 2 bytes/sample, byte0 = 0x80|(v>>5), byte1 = v&0x1F
  while (ring_tail != ring_head && Serial.availableForWrite() >= 2) {
    uint16_t v = ring[ring_tail];
    ring_tail = (ring_tail + 1) & (RING_N - 1);
    Serial.write((uint8_t)(0x80 | (v >> 5)));
    Serial.write((uint8_t)(v & 0x1F));
  }
  // command line input
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line_len) { line[line_len] = 0; handle(line); line_len = 0; }
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = c;
    }
  }
}
