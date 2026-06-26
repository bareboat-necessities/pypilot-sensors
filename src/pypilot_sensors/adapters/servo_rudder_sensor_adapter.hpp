#pragma once

#include <pypilot_servo_protocol.hpp>
#include <ship_data_model.hpp>
#include "../samples.hpp"
#include "../data_model_writer.hpp"

namespace pypilot_sensors {

template<typename Real = float>
class ServoRudderSensorAdapter {
public:
    ServoRudderSensorAdapter() {}

    ServoTelemetrySample<Real> make_sample(const pypilot_servo_protocol::Telemetry& telemetry,
                                           uint64_t now_us,
                                           Real rudder_range_deg,
                                           const char* device_id = 0) const {
        ServoTelemetrySample<Real> sample;
        sample.time_us = now_us;
        sample.device_id = device_id;
        if (telemetry.has_flags) { sample.flags_valid = true; sample.flags = telemetry.flags; }
        if (telemetry.has_voltage) { sample.voltage_valid = true; sample.voltage_v = static_cast<Real>(telemetry.voltage_v); }
        if (telemetry.has_current) { sample.current_valid = true; sample.current_a = static_cast<Real>(telemetry.current_a); }
        if (telemetry.has_controller_temp) { sample.controller_temp_valid = true; sample.controller_temp_c = static_cast<Real>(telemetry.controller_temp_c); }
        if (telemetry.has_motor_temp) { sample.motor_temp_valid = true; sample.motor_temp_c = static_cast<Real>(telemetry.motor_temp_c); }
        if (telemetry.has_rudder) {
            sample.rudder.time_us = now_us;
            sample.rudder.device_id = device_id;
            sample.rudder.source = ship_data_model::SensorSource::servo;
            sample.rudder.raw_valid = telemetry.rudder_valid;
            sample.rudder.raw_0_1 = static_cast<Real>(telemetry.rudder + 0.5f);
            if (telemetry.rudder_valid && rudder_range_deg > Real(0)) {
                sample.rudder.angle_valid = true;
                sample.rudder.angle_deg = static_cast<Real>(telemetry.rudder) * rudder_range_deg;
            }
        }
        return sample;
    }

    bool apply_telemetry(const pypilot_servo_protocol::Telemetry& telemetry,
                         ship_data_model::DataModel<Real>& model,
                         uint64_t now_us,
                         const char* device_id = 0) const {
        ServoTelemetrySample<Real> sample = make_sample(telemetry, now_us, model.rudder.range_deg.value, device_id);
        return writer_.write_servo(model, sample);
    }

    void reset_arbitration() const { writer_.reset_arbitration(); }

private:
    mutable SensorDataModelWriter<Real> writer_;
};

} // namespace pypilot_sensors
