# Hardware measurement procedure

Equipment: Arduino Uno, breadboard, 2x 2.2 kOhm, 2x 47 nF, resistors/cap for the input network, USB
cable, multimeter (note its limits), ideally an oscilloscope or frequency counter and a function
generator. Phone-based scope/frequency apps are acceptable if you state it.

1. Measure the Uno's actual 5 V rail with the multimeter; pass it as `--vref`.
2. Flash `waveform_daq.ino`; confirm `python daq_app.py --port X gen sin 50` replies `OK SIN f=50.0000`.
3. **Loopback**: connect filter output -> A0. Run
   `python daq_app.py --port X --vref <vcc> sweep --only sin,tri --out ../results/hardware_measurements.csv`.
4. **Square**: connect D9 -> A0 (through R5). `sweep --only sq --append --out <same csv>`.
5. **Independent reference**: for 3 frequencies, put the scope/counter on the filter output (sine)
   and D9 (square); write the reading into `f_ref` and `vpp_ref` of that row.
6. **External source**: feed a function generator (kept within 0-5 V, or through the C3/R3/R4 bias
   network) into A0 and `capture`; compare the reported frequency/Vpp to the generator and scope.
7. `python daq_app.py report ../results/hardware_measurements.csv`; copy the table and your
   observations (noise, droop, anything surprising) into the README results table.
8. Save a `capture --png` image for each shape.

Record the equipment, its stated accuracy and the Vcc you measured. Report the errors you got, even
if larger than the simulation suggests; the usual causes are resonator tolerance, Vcc error, ADC
gain/offset error and filter component tolerance (5 % capacitors shift droop at 200 Hz).
