// PlatformIO entry point; Arduino IDE uses the root .ino instead.
#if defined(PLATFORMIO)
#include "../firmware.h"
void setup() { ppgapp::setup(); }
void loop() { ppgapp::loop(); }
#endif
