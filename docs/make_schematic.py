"""Generate docs/circuit.png with schemdraw (pip install schemdraw)."""
import schemdraw
import schemdraw.elements as elm

with schemdraw.Drawing(show=False) as d:
    d.config(unit=2.6, fontsize=10)
    # --- sine / triangle: PWM + 2nd-order RC low-pass --------------------------------------
    d += elm.Dot().label("Uno D3\n(PWM 62.5 kHz)", "left")
    d += elm.Resistor().right().label("R1 2.2k")
    d += (n1 := elm.Dot())
    d += elm.Capacitor().down().label("C1 47nF", "right")
    d += elm.Ground()
    d.move_from(n1.start, dx=0, dy=0)
    d += elm.Resistor().right().at(n1.start).label("R2 2.2k")
    d += (n2 := elm.Dot())
    d += elm.Capacitor().down().label("C2 47nF", "right")
    d += elm.Ground()
    d += elm.Line().right().at(n2.start).length(1.5)
    d += (ao := elm.Dot().label("analog out\n(0..5 V, DC 2.5 V)", "bottom"))
    d += elm.Line().right().length(1.6)
    d += elm.Dot(open=True).label("to A0 for\nloopback test", "top")

with schemdraw.Drawing(show=False) as d2:
    d2.config(unit=2.6, fontsize=10)
    # --- input conditioning for the ADC ----------------------------------------------------
    d2 += elm.Dot(open=True).label("signal in\n(AC or 0..5 V)", "left")
    d2 += elm.Capacitor().right().label("C3 1uF\n(AC coupling)")
    d2 += (m := elm.Dot())
    d2 += elm.Resistor().up().label("R3 10k", "left")
    d2 += elm.Vdd().label("+5 V")
    d2 += elm.Resistor().down().at(m.start).label("R4 10k", "left")
    d2 += elm.Ground()
    d2 += elm.Resistor().right().at(m.start).label("R5 1k\n(series)")
    d2 += elm.Dot().label("Uno A0", "top")
    d2 += elm.Line().right().length(0.8)
    d2 += elm.Dot(open=True).label("A0 node", "right")
    d2.save("docs/_input.png", dpi=150)
    d.save("docs/_output.png", dpi=150)
