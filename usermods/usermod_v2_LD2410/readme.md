# LD2410 Usermod

Communicates to a LD2410 Radar module and dims the lights when no presense is detected.

## Installation 

define `USERMOD_LD2410` e.g.

`#define USERMOD_LD2410` in my_config.h

or add `-D USERMOD_LD2410` to `build_flags` in platformio_override.ini

Connect an LD2410 module RX and TX pins to suitable pins on your microcontroller:

ESP32s2: connect RX and TX to any unused pins (Check the drop down menu in the usermod settings to see which pins are unallocated)

