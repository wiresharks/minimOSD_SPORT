#include "Arduino.h"
#include "types.h"
#include "KV_Team_OSD.h"
#include "mobius.h"

#ifdef MOBIUS_CONTROL

static uint8_t control_cycle_count = 0;
static bool cameraStarted = false;

void mobius_control_initialize(void) {
  pinMode(MOBIUS_CONTROL_PIN, OUTPUT);
}

void mobius_control_update(void) {
  static const uint8_t pinMask = digitalPinToBitMask(MOBIUS_CONTROL_PIN);
  volatile uint8_t *port = portOutputRegister(digitalPinToPort(MOBIUS_CONTROL_PIN));
  uint8_t portValue = *port;

  if(control_cycle_count == 0) {
    if((armed && !cameraStarted) || (!armed && cameraStarted)) {
      control_cycle_count = MOBIUS_CONTROL_CYCLES;
      cameraStarted = armed;
      *port = portValue | pinMask;
    }
  }
  else {
    control_cycle_count--;
    if(control_cycle_count == 0) {
      *port = portValue & (~pinMask);
    }
  }
}

#endif
