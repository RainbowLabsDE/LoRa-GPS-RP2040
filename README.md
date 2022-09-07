# LoRa GPS TTNMapper based on RP2040

> ATTENTION! The lmic library is (as usual) broken. I don't want to fork the lib for a single line fix.  
> 
> So simply paste the following line into `.pio/libdeps/pico/MCCI LoRaWAN LMIC library/src/hal/hal.h` at line 16, until I find a more elegant solution:  
> `#include "lmic/oslmic.h"`

Uplink payload formatter for TTN Console: [Gist](https://gist.github.com/LeoDJ/a0b9a0bd32054f4c696a73353b66599a)

TTNMapper webhook setup tutorial: https://docs.ttnmapper.org/integration/tts-integration-v3.html

TODO: Pinout and docs and stuff
