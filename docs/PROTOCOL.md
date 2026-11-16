# Serial protocol (115200 8N1)

PC -> MCU: ASCII, one command per line (`\n`).
`SIN f [amp]`, `TRI f [amp]`, `SQ f [duty]`, `OFF`, `RATE hz`, `START`, `STOP`, `INFO`, `ID`.

MCU -> PC: reply lines `OK ...` / `ERR ...`, plus while streaming 2 bytes per sample:

    byte0 = 0x80 | (v >> 5)      bit 7 set marks the start of a sample
    byte1 = v & 0x1F             bit 7 clear
    v     = 10-bit ADC count (0..1023);  volts = v * Vref / 1024

Because only first bytes have bit 7 set, a decoder that loses a byte resynchronises on the next
sample (tested in `tests/test_daq.py`). Throughput at 5 kS/s is 10 kB/s, under the ~11.5 kB/s limit
of 115200 baud. The firmware counts samples dropped when the 128-entry ring buffer fills; `STOP`
reports `drops=`.
