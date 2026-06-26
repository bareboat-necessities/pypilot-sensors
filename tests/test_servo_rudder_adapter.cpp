#include <cassert>
#include <cmath>
#include <pypilot_sensors.hpp>

using namespace pypilot_sensors;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    ship_data_model::DataModel<float> model;
    model.rudder.range_deg.value = 30.0f;

    pypilot_servo_protocol::Telemetry telemetry;
    telemetry.has_flags = true;
    telemetry.flags = pypilot_servo_protocol::ENGAGED_FLAG;
    telemetry.has_voltage = true;
    telemetry.voltage_v = 12.4f;
    telemetry.has_current = true;
    telemetry.current_a = 1.2f;
    telemetry.has_rudder = true;
    telemetry.rudder_valid = true;
    telemetry.rudder = 0.25f;

    ServoRudderSensorAdapter<float> adapter;
    adapter.apply_telemetry(telemetry, model, 1000);

    assert(model.servo.flags.value == pypilot_servo_protocol::ENGAGED_FLAG);
    assert(model.servo.engaged.value);
    assert(model.servo.has_controller);
    assert(nearf(model.servo.voltage_v.value, 12.4f));
    assert(nearf(model.servo.current_a.value, 1.2f));
    assert(model.rudder.source.value == ship_data_model::SensorSource::servo);
    assert(nearf(model.rudder.angle_deg.value, 7.5f));

    return 0;
}
