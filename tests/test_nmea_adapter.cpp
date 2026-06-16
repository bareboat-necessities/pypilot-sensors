#include <cassert>
#include <cmath>
#include <pypilot_sensors.hpp>

using namespace pypilot_sensors;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    pypilot_data_model::DataModel<float> model;
    Nmea0183SensorAdapter<float> adapter;

    // $IIVHW,,,,,5.50,N,,*19
    assert(adapter.parse_and_apply_line("$IIVHW,,,,,5.50,N,,*19", model, 1000,
                                        pypilot_data_model::SensorSource::serial));
    assert(model.water.source.value == pypilot_data_model::SensorSource::serial);
    assert(model.water.last_update_us == 1000);
    assert(model.water.speed_kn.valid);
    assert(nearf(model.water.speed_kn.value, 5.5f));

    // $IIRSA,12.5,A,,V*4F
    assert(adapter.parse_and_apply_line("$IIRSA,12.5,A,,V*4F", model, 2000,
                                        pypilot_data_model::SensorSource::serial));
    assert(model.rudder.source.value == pypilot_data_model::SensorSource::serial);
    assert(model.rudder.last_update_us == 2000);
    assert(nearf(model.rudder.angle_deg.value, 12.5f));

    return 0;
}
