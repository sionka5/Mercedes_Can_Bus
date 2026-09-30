#include <Arduino.h>

#include "VehicleController.h"

VehicleController controller;


void setup() {
    controller.begin();
    analogWrite(21, LOW);
}

void loop() {
    controller.service();
}
