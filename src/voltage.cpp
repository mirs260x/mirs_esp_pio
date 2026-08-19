#include "config.hpp"
#include "VoltageSensor.hpp"
#include <Arduino.h>
#include <cstdlib>

extern VoltageSensor voltage_sensor_1;
extern VoltageSensor voltage_sensor_2;

void vlt_setup() {
  voltage_sensor_1.begin(PIN_BATT_1, VOLTAGE_DIVIDER_RATIO);
  voltage_sensor_2.begin(PIN_BATT_2, VOLTAGE_DIVIDER_RATIO);

  vlt_msg.data.capacity = 2;
  vlt_msg.data.size = 2;
  vlt_msg.data.data = static_cast<double *>(
      malloc(vlt_msg.data.capacity * sizeof(double)));
  if (vlt_msg.data.data != nullptr) {
    vlt_msg.data.data[0] = 0.0;
    vlt_msg.data.data[1] = 0.0;
  }
}

