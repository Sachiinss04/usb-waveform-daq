# Design decisions

**Sample timing.** Timer0 runs in CTC mode (250 kHz tick) and its compare-match flag auto-triggers
the ADC, so the sample period is set by hardware: no software jitter, and the reported sample rate
(`fs=` in the reply to `RATE`/`START`) is exact for the crystal frequency.

**Waveform synthesis.** Timer2 produces an 8-bit PWM carrier at 16 MHz/256 = 62.5 kHz on D3. Every 8th
overflow (7812.5 Hz) a 32-bit phase accumulator advances and the PWM compare value is updated from a
256-entry sine table in flash (triangle is computed arithmetically). Frequency resolution is
7812.5/2^32 Hz, independent of table size. Amplitude scales around mid-level in integer arithmetic.

**Filter.** Two RC stages, R = 2.2 k, C = 47 nF each: fc = 1/(2 pi R C) = 1.54 kHz per stage. The
62.5 kHz carrier is attenuated by ~64 dB; the 7.8 kHz DDS update images by ~29 dB (about 3.4 % of the
signal amplitude). Cost: passband droop of the cascaded ladder, |H| = 1/|1 + 3x + x^2|, x = j2 pi f R C:

| f (Hz) | 5 | 10 | 20 | 50 | 100 | 200 |
|---|---|---|---|---|---|---|
| gain | 1.0000 | 0.9999 | 0.9994 | 0.9963 | 0.9855 | 0.9456 |

`daq_app.py report` includes this droop in the expected sine amplitude. Output impedance is about
4.4 k, within the ~10 k source-impedance guidance for the AVR ADC.

**Square wave.** Timer1 fast-PWM mode 14 (TOP = ICR1) gives independent frequency and duty on D9.
The firmware picks the smallest prescaler that fits 16 bits and replies with the actual frequency.

**Analysis.** Frequency: Schmitt-trigger zero crossings with linear interpolation, cross-checked by
FFT. Sinusoids: 4-parameter least-squares fit (amplitude, noise from the residual). Triangle (shape
hint): fit of an ideal triangle. Square (shape hint): median plateau levels. Reasoning and the
failures that motivated each choice are in the README.

**Why an emulator.** `FakeDevice` implements the same commands and the same frequency quantisation
as the firmware, so the host protocol, parser and application are tested without hardware. It does
not test the firmware itself.
