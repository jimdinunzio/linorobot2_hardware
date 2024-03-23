#include <Arduino.h>
#include "traxxas_remctl.h"

// put function declarations here:

Traxxas_RemCtl remote(1000, 2000, true, 1000, 2000, true);

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Starting test of Traxxas_RemCtl library");
  // put your setup code here, to run once:
  remote.setup();
}

static uint32_t next_print_time = 0;
void loop() {


  if (Serial.available() > 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if (incomingByte == 's') {
      remote.enable_steering_channel(!remote.steering_enabled());
    } else if (incomingByte == 't') {
      remote.enable_throttle_channel(!remote.throttle_enabled());
    }
  }
  if (millis() > next_print_time)
  {
    next_print_time = millis() + 100;
    Serial.print("Throttle: ");
    Serial.print(remote.throttle_percent());
    Serial.print("%, Steering: ");
    Serial.print(remote.steering_percent());
    Serial.print("%");
    Serial.print(", estop_assert: ");
    Serial.println(remote.estop_assert());

  }

  // put your main code here, to run repeatedly:
  remote.update();
}

