#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

namespace esphome {

class RedlinkThermostatHumiditySensor : public sensor::Sensor, public Component {
 public:
  RedlinkThermostatHumiditySensor() = default;
};

class RedlinkThermostatInfoSensor : public text_sensor::TextSensor, public Component {
 public:
  RedlinkThermostatInfoSensor() = default;
};

class RedlinkThermostatTemperatureSourceSensor : public binary_sensor::BinarySensor, public Component {
 public:
  RedlinkThermostatTemperatureSourceSensor() = default;
};

class RedlinkDiagnosticSensor : public sensor::Sensor, public Component {
 public:
  RedlinkDiagnosticSensor() = default;
};

class RedlinkConnectionSensor : public binary_sensor::BinarySensor, public Component {
 public:
  RedlinkConnectionSensor() = default;
};

class RedlinkControlSourceSensor : public text_sensor::TextSensor, public Component {
 public:
  RedlinkControlSourceSensor() = default;
};

}  // namespace esphome
