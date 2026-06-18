#include <cassert>
#include <cmath>
#include <pypilot_sensors.hpp>

using namespace pypilot_sensors;
using pypilot_data_model::SensorSource;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    pypilot_data_model::DataModel<float> model;
    SensorDataModelWriter<float> writer;

    GpsSample<float> serial_gps;
    serial_gps.time_us = 100;
    serial_gps.device_id = "serial0";
    serial_gps.source = SensorSource::serial;
    serial_gps.speed_valid = true;
    serial_gps.speed_kn = 4.0f;
    assert(writer.write_gps(model, serial_gps));
    assert(model.navigation.gps.source.value == SensorSource::serial);
    assert(nearf(model.navigation.gps.speed_kn.value, 4.0f));

    GpsSample<float> signalk_gps;
    signalk_gps.time_us = 200;
    signalk_gps.device_id = "signalk0";
    signalk_gps.source = SensorSource::signalk;
    signalk_gps.speed_valid = true;
    signalk_gps.speed_kn = 9.0f;
    assert(!writer.write_gps(model, signalk_gps));
    assert(model.navigation.gps.source.value == SensorSource::serial);
    assert(nearf(model.navigation.gps.speed_kn.value, 4.0f));

    GpsSample<float> gpsd_gps;
    gpsd_gps.time_us = 300;
    gpsd_gps.device_id = "gpsd0";
    gpsd_gps.source = SensorSource::gpsd;
    gpsd_gps.speed_valid = true;
    gpsd_gps.speed_kn = 5.0f;
    assert(writer.write_gps(model, gpsd_gps));
    assert(model.navigation.gps.source.value == SensorSource::gpsd);
    assert(nearf(model.navigation.gps.speed_kn.value, 5.0f));

    WindSample<float> wind0;
    wind0.time_us = 1000;
    wind0.device_id = "nmea0";
    wind0.source = SensorSource::serial;
    wind0.speed_valid = true;
    wind0.speed_kn = 10.0f;
    assert(writer.write_wind(model, wind0));
    assert(model.wind.apparent.source.value == SensorSource::serial);
    assert(nearf(model.wind.apparent.speed_kn.value, 10.0f));

    WindSample<float> wind1 = wind0;
    wind1.time_us = 1100;
    wind1.device_id = "nmea1";
    wind1.speed_kn = 20.0f;
    assert(!writer.write_wind(model, wind1));
    assert(nearf(model.wind.apparent.speed_kn.value, 10.0f));

    wind0.time_us = 1200;
    wind0.speed_kn = 11.0f;
    assert(writer.write_wind(model, wind0));
    assert(nearf(model.wind.apparent.speed_kn.value, 11.0f));

    wind1.time_us = 9000001ULL;
    wind1.speed_kn = 21.0f;
    assert(writer.write_wind(model, wind1));
    assert(nearf(model.wind.apparent.speed_kn.value, 21.0f));

    WindSample<float> none_wind;
    none_wind.time_us = 9000100ULL;
    none_wind.source = SensorSource::none;
    none_wind.speed_valid = true;
    none_wind.speed_kn = 30.0f;
    assert(!writer.write_wind(model, none_wind));
    assert(nearf(model.wind.apparent.speed_kn.value, 21.0f));

    return 0;
}
