#include <cassert>
#include <cmath>
#include <pypilot_sensors.hpp>
#include <pypilot_signalk_connector.hpp>

using namespace pypilot_sensors;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    pypilot_data_model::DataModel<float> model;
    SignalKSensorAdapter<float> adapter;

    assert(adapter.apply_number(pypilot_signalk_connector::SignalKPath::gps_speed_over_ground,
                                2.0f,
                                model,
                                1000));
    assert(model.navigation.gps.source.value == pypilot_data_model::SensorSource::signalk);
    assert(model.navigation.gps.last_update_us == 1000);
    assert(model.navigation.gps.speed_kn.valid);
    assert(model.navigation.gps.speed_kn.value > 3.8f && model.navigation.gps.speed_kn.value < 3.9f);

    assert(adapter.apply_number(pypilot_signalk_connector::SignalKPath::rudder_angle,
                                0.174532925f,
                                model,
                                2000));
    assert(model.rudder.source.value == pypilot_data_model::SensorSource::signalk);
    assert(model.rudder.last_update_us == 2000);
    assert(nearf(model.rudder.angle_deg.value, -10.0f));

    return 0;
}
