#include <cassert>
#include <cmath>
#include <pypilot_sensors.hpp>

using namespace pypilot_sensors;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    pypilot_data_model::DataModel<float> model;
    SensorDataModelWriter<float> writer;

    GpsSample<float> gps;
    gps.time_us = 100;
    gps.source = pypilot_data_model::SensorSource::serial;
    gps.speed_valid = true;
    gps.speed_kn = 4.2f;
    gps.track_valid = true;
    gps.track_deg = 370.0f;
    gps.fix_valid = true;
    gps.latitude_deg = 10.0f;
    gps.longitude_deg = -20.0f;
    writer.write_gps(model, gps);

    assert(model.navigation.gps.source.value == pypilot_data_model::SensorSource::serial);
    assert(model.navigation.gps.last_update_us == 100);
    assert(nearf(model.navigation.gps.speed_kn.value, 4.2f));
    assert(nearf(model.navigation.gps.track_deg.value, 10.0f));
    assert(nearf(model.navigation.gps.fix_lat_deg.value, 10.0f));
    assert(nearf(model.navigation.gps.fix_lon_deg.value, -20.0f));

    WindSample<float> wind;
    wind.time_us = 200;
    wind.source = pypilot_data_model::SensorSource::signalk;
    wind.speed_valid = true;
    wind.speed_kn = 12.0f;
    wind.direction_valid = true;
    wind.direction_deg = 190.0f;
    writer.write_wind(model, wind);

    assert(model.wind.apparent.source.value == pypilot_data_model::SensorSource::signalk);
    assert(model.wind.apparent.last_update_us == 200);
    assert(nearf(model.wind.apparent.direction_deg.value, -170.0f));

    return 0;
}
