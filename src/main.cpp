#include <Arduino.h>

#include "VehicleController.h"

VehicleController controller;

#define TOP_AMBIENT 21

void setup() {
    controller.begin();
    analogWrite(TOP_AMBIENT, 70);
}

void loop() {
    controller.service();
}
