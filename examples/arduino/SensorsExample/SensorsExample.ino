#include <pypilot_sensors.hpp>

ship_data_model::DataModel<float> model;
pypilot_sensors::SensorDataModelWriter<float> writer;

void setup() {
  Serial.begin(115200);
  pypilot_sensors::GpsSample<float> gps;
  gps.time_us = 1000;
  gps.source = ship_data_model::SensorSource::serial;
  gps.speed_valid = true;
  gps.speed_kn = 4.0f;
  writer.write_gps(model, gps);
  Serial.println(model.navigation.gps.speed_kn.value);
}

void loop() {}
