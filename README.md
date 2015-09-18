# TestBench
Collection of small test sketches for Teensy 3.x based stepper motor control system.

Framework for proof of concept bench test configuration of prototype lathe hardware, consisting of:
- Teensy 3.1 (or 3.2)  https://www.pjrc.com/store/teensy31.html & https://www.pjrc.com/store/teensy32_pins.html
- Mounted on a Teensy 3* Proto Board  https://www.tindie.com/products/freto/teensy-3-breakout-board-and-shield/
- ILI9341 TFT screen  https://www.pjrc.com/store/display_ili9341.html
- 2 rotary encoders  PEC11-4215F-S24  http://www.adafruit.com/datasheets/pec11.pdf
- 4 user interface / menu buttons  (push-to-break, e.g. normally closed)
- E-Stop button (emergency stop)  (push-to-break, e.g. normally closed)
- 2 Pololu DRV8825 stepper drivers  https://www.pololu.com/product/2133
- 2 Vexta PX245-02AA-C8 stepper motors  http://www.interinar.com/public_docs/PK245-02AA.pdf
	-> N.B. stepper motors 0.57A/phase, Vref for driver = current/2 = 0.285V on DRV8825
- DC/DC voltage converter  Recom R-78C5.0-1.0  http://www.recom-power.com/pdf/Innoline/R-78Cxx-1.0.pdf

The two stepper motors will drive the lathe spindle and sliderest.
