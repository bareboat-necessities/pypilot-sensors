#pragma once

#include <pypilot_data_model.hpp>
#include <pypilot_algorithms.hpp>
#include "samples.hpp"
#include "source_policy.hpp"

namespace pypilot_sensors {

template<typename Real = float>
class SensorDataModelWriter {
public:
    SensorDataModelWriter() : has_last_apb_update_(false), last_apb_update_us_(0) {}

    void reset_arbitration() {
        arbiter_.reset();
        has_last_apb_update_ = false;
        last_apb_update_us_ = 0;
    }
    void set_source_timeout_us(uint64_t timeout_us) { arbiter_.set_timeout_us(timeout_us); }
    uint64_t source_timeout_us() const { return arbiter_.timeout_us(); }

    bool write_imu(pypilot_data_model::DataModel<Real>& model, const ImuSample<Real>& sample) const {
        if (sample.heading_valid) model.imu.heading_deg.set(pypilot_algorithms::wrap_360_deg(sample.heading_deg), sample.time_us);
        if (sample.heading_rate_valid) model.imu.heading_rate_deg_s.set(sample.heading_rate_deg_s, sample.time_us);
        if (sample.heading_rate_rate_valid) model.imu.heading_rate_rate_deg_s2.set(sample.heading_rate_rate_deg_s2, sample.time_us);
        if (sample.pitch_valid) model.imu.pitch_deg.set(sample.pitch_deg, sample.time_us);
        if (sample.roll_valid) model.imu.roll_deg.set(sample.roll_deg, sample.time_us);
        if (sample.heel_valid) model.imu.heel_deg.set(sample.heel_deg, sample.time_us);
        if (sample.accel_valid) model.imu.accel_g.set(sample.accel_g, sample.time_us);
        if (sample.gyro_valid) model.imu.gyro_deg_s.set(sample.gyro_deg_s, sample.time_us);
        if (sample.compass_valid) model.imu.compass_raw.set(sample.compass_raw, sample.time_us);
        return true;
    }

    bool write_gps(pypilot_data_model::DataModel<Real>& model, const GpsSample<Real>& sample) const {
        if (!arbiter_.accept(SourceArbitrationSlot::gps,
                             model.navigation.gps.source.value,
                             model.navigation.gps.last_update_us,
                             sample.source,
                             sample.device_id,
                             sample.time_us)) {
            return false;
        }
        if (sample.source != pypilot_data_model::SensorSource::none) model.navigation.gps.source.value = sample.source;
        if (sample.speed_valid) model.navigation.gps.speed_kn.set(sample.speed_kn, sample.time_us);
        if (sample.track_valid) model.navigation.gps.track_deg.set(pypilot_algorithms::wrap_360_deg(sample.track_deg), sample.time_us);
        if (sample.fix_valid) {
            model.navigation.gps.fix_lat_deg.set(sample.latitude_deg, sample.time_us);
            model.navigation.gps.fix_lon_deg.set(sample.longitude_deg, sample.time_us);
        }
        if (sample.altitude_valid) model.navigation.gps.fix_alt_m.set(sample.altitude_m, sample.time_us);
        if (sample.declination_valid) model.navigation.gps.declination_deg.set(sample.declination_deg, sample.time_us);
        model.navigation.gps.last_update_us = sample.time_us;
        return true;
    }

    bool write_apb(pypilot_data_model::DataModel<Real>& model, const ApbSample<Real>& sample) const {
        if (has_last_apb_update_ && !apb_update_rate_allows(sample.time_us, last_apb_update_us_)) {
            return false;
        }
        if (!arbiter_.accept(SourceArbitrationSlot::apb,
                             model.navigation.apb.source.value,
                             model.navigation.apb.last_update_us,
                             sample.source,
                             sample.device_id,
                             sample.time_us)) {
            return false;
        }
        if (sample.source != pypilot_data_model::SensorSource::none) model.navigation.apb.source.value = sample.source;
        if (sample.track_valid) model.navigation.apb.track_deg.set(pypilot_algorithms::wrap_360_deg(sample.track_deg), sample.time_us);
        if (sample.xte_valid) model.navigation.apb.xte_nmi.set(sample.xte_nmi, sample.time_us);
        model.navigation.apb.last_update_us = sample.time_us;
        last_apb_update_us_ = sample.time_us;
        has_last_apb_update_ = true;
        return true;
    }

    bool write_wind(pypilot_data_model::DataModel<Real>& model, const WindSample<Real>& sample) const {
        pypilot_data_model::WindSensorData<Real>& target = sample.true_wind ? model.wind.truewind : model.wind.apparent;
        const SourceArbitrationSlot slot = sample.true_wind ? SourceArbitrationSlot::wind_true : SourceArbitrationSlot::wind_apparent;
        if (!arbiter_.accept(slot,
                             target.source.value,
                             target.last_update_us,
                             sample.source,
                             sample.device_id,
                             sample.time_us)) {
            return false;
        }
        if (sample.source != pypilot_data_model::SensorSource::none) target.source.value = sample.source;
        if (sample.speed_valid) target.speed_kn.set(sample.speed_kn, sample.time_us);
        if (sample.direction_valid) target.direction_deg.set(pypilot_algorithms::wrap_180_deg(sample.direction_deg), sample.time_us);
        target.last_update_us = sample.time_us;
        return true;
    }

    bool write_water(pypilot_data_model::DataModel<Real>& model, const WaterSample<Real>& sample) const {
        if (!arbiter_.accept(SourceArbitrationSlot::water,
                             model.water.source.value,
                             model.water.last_update_us,
                             sample.source,
                             sample.device_id,
                             sample.time_us)) {
            return false;
        }
        if (sample.source != pypilot_data_model::SensorSource::none) model.water.source.value = sample.source;
        if (sample.speed_valid) model.water.speed_kn.set(sample.speed_kn, sample.time_us);
        if (sample.leeway_valid) {
            model.water.leeway_deg.set(sample.leeway_deg, sample.time_us);
            model.water.leeway_source.value = sample.source;
        }
        if (sample.current_valid) {
            model.water.current_speed_kn.set(sample.current_speed_kn, sample.time_us);
            model.water.current_direction_deg.set(pypilot_algorithms::wrap_360_deg(sample.current_direction_deg), sample.time_us);
        }
        model.water.last_update_us = sample.time_us;
        return true;
    }

    bool write_rudder(pypilot_data_model::DataModel<Real>& model, const RudderSample<Real>& sample) const {
        if (!arbiter_.accept(SourceArbitrationSlot::rudder,
                             model.rudder.source.value,
                             model.rudder.last_update_us,
                             sample.source,
                             sample.device_id,
                             sample.time_us)) {
            return false;
        }
        if (sample.source != pypilot_data_model::SensorSource::none) model.rudder.source.value = sample.source;
        if (sample.angle_valid) model.rudder.angle_deg.set(sample.angle_deg, sample.time_us);
        if (sample.speed_valid) model.rudder.speed_deg_s.set(sample.speed_deg_s, sample.time_us);
        if (sample.raw_valid) model.rudder.raw_0_1.set(sample.raw_0_1, sample.time_us);
        model.rudder.last_update_us = sample.time_us;
        return true;
    }

    bool write_servo(pypilot_data_model::DataModel<Real>& model, const ServoTelemetrySample<Real>& sample) const {
        bool ok = true;
        if (sample.flags_valid) {
            model.servo.flags.value = sample.flags;
            model.servo.faults.value = sample.flags;
            model.servo.engaged.value = (sample.flags & pypilot_data_model::servo_engaged_flag) != 0;
            model.servo.has_state = true;
            model.servo.has_controller = true;
        }
        if (sample.voltage_valid) model.servo.voltage_v.set(sample.voltage_v, sample.time_us);
        if (sample.current_valid) model.servo.current_a.set(sample.current_a, sample.time_us);
        if (sample.controller_temp_valid) model.servo.controller_temp_c.set(sample.controller_temp_c, sample.time_us);
        if (sample.motor_temp_valid) model.servo.motor_temp_c.set(sample.motor_temp_c, sample.time_us);
        if (sample.rudder.angle_valid || sample.rudder.raw_valid) ok = write_rudder(model, sample.rudder);
        return ok;
    }

    bool write_batch(pypilot_data_model::DataModel<Real>& model, const SensorBatch<Real>& batch) const {
        bool ok = true;
        if (batch.has_imu) ok = write_imu(model, batch.imu) && ok;
        if (batch.has_gps) ok = write_gps(model, batch.gps) && ok;
        if (batch.has_apb) ok = write_apb(model, batch.apb) && ok;
        if (batch.has_wind) ok = write_wind(model, batch.wind) && ok;
        if (batch.has_water) ok = write_water(model, batch.water) && ok;
        if (batch.has_rudder) ok = write_rudder(model, batch.rudder) && ok;
        if (batch.has_servo) ok = write_servo(model, batch.servo) && ok;
        return ok;
    }

private:
    mutable SourceDeviceArbitrator arbiter_;
    mutable bool has_last_apb_update_;
    mutable uint64_t last_apb_update_us_;
};

} // namespace pypilot_sensors
