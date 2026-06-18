#pragma once

#include <stdint.h>

#include <pypilot_data_model.hpp>
#include <pypilot_algorithms.hpp>
#include "source_policy.hpp"

namespace pypilot_sensors {

template<typename StampedValue>
inline void invalidate_sensor_value(StampedValue& value) {
    value.valid = false;
    value.last_update_us = 0;
}

template<typename Real = float>
class SensorSourceTimeoutManager {
public:
    SensorSourceTimeoutManager() : timeout_us_(default_source_device_timeout_us) {}

    void set_timeout_us(uint64_t timeout_us) { timeout_us_ = timeout_us; }
    uint64_t timeout_us() const { return timeout_us_; }

    bool poll(pypilot_data_model::DataModel<Real>& model, uint64_t now_us) const {
        bool changed = false;
        changed = timeout_gps(model, now_us) || changed;
        changed = timeout_apb(model, now_us) || changed;
        changed = timeout_wind(model.wind.apparent, now_us) || changed;
        changed = timeout_wind(model.wind.truewind, now_us) || changed;
        changed = timeout_water(model, now_us) || changed;
        changed = timeout_rudder(model, now_us) || changed;
        return changed;
    }

private:
    bool expired(pypilot_data_model::SensorSource source, uint64_t last_update_us, uint64_t now_us) const {
        return source != pypilot_data_model::SensorSource::none &&
               last_update_us != 0 &&
               pypilot_algorithms::pypilot_source_is_stale(now_us, last_update_us, timeout_us_);
    }

    bool timeout_gps(pypilot_data_model::DataModel<Real>& model, uint64_t now_us) const {
        if (!expired(model.navigation.gps.source.value, model.navigation.gps.last_update_us, now_us)) {
            return false;
        }
        model.navigation.gps.source.value = pypilot_data_model::SensorSource::none;
        invalidate_sensor_value(model.navigation.gps.track_deg);
        invalidate_sensor_value(model.navigation.gps.speed_kn);
        model.navigation.gps.last_update_us = 0;
        return true;
    }

    bool timeout_apb(pypilot_data_model::DataModel<Real>& model, uint64_t now_us) const {
        if (!expired(model.navigation.apb.source.value, model.navigation.apb.last_update_us, now_us)) {
            return false;
        }
        model.navigation.apb.source.value = pypilot_data_model::SensorSource::none;
        model.navigation.apb.xte_nmi.set(Real(0), now_us);
        model.navigation.apb.last_update_us = 0;
        return true;
    }

    bool timeout_wind(pypilot_data_model::WindSensorData<Real>& wind, uint64_t now_us) const {
        if (!expired(wind.source.value, wind.last_update_us, now_us)) {
            return false;
        }
        wind.source.value = pypilot_data_model::SensorSource::none;
        invalidate_sensor_value(wind.direction_deg);
        invalidate_sensor_value(wind.speed_kn);
        invalidate_sensor_value(wind.filtered_direction_deg);
        invalidate_sensor_value(wind.filtered_speed_kn);
        wind.last_update_us = 0;
        return true;
    }

    bool timeout_water(pypilot_data_model::DataModel<Real>& model, uint64_t now_us) const {
        if (!expired(model.water.source.value, model.water.last_update_us, now_us)) {
            return false;
        }
        model.water.source.value = pypilot_data_model::SensorSource::none;
        model.water.leeway_source.value = pypilot_data_model::SensorSource::none;
        invalidate_sensor_value(model.water.speed_kn);
        invalidate_sensor_value(model.water.leeway_deg);
        invalidate_sensor_value(model.water.current_speed_kn);
        invalidate_sensor_value(model.water.current_direction_deg);
        model.water.last_update_us = 0;
        return true;
    }

    bool timeout_rudder(pypilot_data_model::DataModel<Real>& model, uint64_t now_us) const {
        if (!expired(model.rudder.source.value, model.rudder.last_update_us, now_us)) {
            return false;
        }
        model.rudder.source.value = pypilot_data_model::SensorSource::none;
        invalidate_sensor_value(model.rudder.angle_deg);
        invalidate_sensor_value(model.rudder.speed_deg_s);
        invalidate_sensor_value(model.rudder.raw_0_1);
        model.rudder.last_update_us = 0;
        return true;
    }

    uint64_t timeout_us_;
};

} // namespace pypilot_sensors
