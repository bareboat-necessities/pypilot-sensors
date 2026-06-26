#pragma once

#include <pypilot_signalk_connector.hpp>
#include <ship_data_model.hpp>
#include "../source_policy.hpp"

namespace pypilot_sensors {

template<typename Real = float>
class SignalKSensorAdapter {
public:
    SignalKSensorAdapter()
        : has_last_apb_update_(false), last_apb_update_us_(0), last_error_("") {}

    const char* last_error() const { return last_error_; }

    bool apply_number(const char* path,
                      Real value,
                      ship_data_model::DataModel<Real>& model,
                      uint64_t now_us,
                      ship_data_model::SensorSource source = ship_data_model::SensorSource::signalk,
                      const char* device_id = 0) {
        last_error_ = "";
        SourceArbitrationSlot slot;
        const bool has_slot = classify_path(path, slot);
        if (has_slot && slot == SourceArbitrationSlot::apb &&
            has_last_apb_update_ && !apb_update_rate_allows(now_us, last_apb_update_us_)) {
            last_error_ = "APB rate limited";
            return false;
        }
        if (has_slot && !can_accept_slot(slot, model, source, device_id, now_us)) {
            last_error_ = "source rejected";
            return false;
        }
        if (!connector_.apply_number(path, value, model, now_us, source)) {
            last_error_ = connector_.last_error();
            return false;
        }
        if (has_slot) {
            arbiter_.accepted(slot, source, device_id);
            if (slot == SourceArbitrationSlot::apb) {
                last_apb_update_us_ = now_us;
                has_last_apb_update_ = true;
            }
        }
        return true;
    }

    bool apply_put_number(const char* path, Real value, ship_data_model::DataModel<Real>& model, uint64_t now_us) {
        last_error_ = "";
        if (!connector_.apply_put_number(path, value, model, now_us)) {
            last_error_ = connector_.last_error();
            return false;
        }
        return true;
    }

    bool apply_put_bool(const char* path, bool value, ship_data_model::DataModel<Real>& model, uint64_t now_us) {
        last_error_ = "";
        if (!connector_.apply_put_bool(path, value, model, now_us)) {
            last_error_ = connector_.last_error();
            return false;
        }
        return true;
    }

    bool apply_put_string(const char* path, const char* value, ship_data_model::DataModel<Real>& model, uint64_t now_us) {
        last_error_ = "";
        if (!connector_.apply_put_string(path, value, model, now_us)) {
            last_error_ = connector_.last_error();
            return false;
        }
        return true;
    }

    void reset_arbitration() {
        arbiter_.reset();
        has_last_apb_update_ = false;
        last_apb_update_us_ = 0;
    }

private:
    pypilot_signalk_connector::SignalKConnector<Real> connector_;
    SourceDeviceArbitrator arbiter_;
    bool has_last_apb_update_;
    uint64_t last_apb_update_us_;
    const char* last_error_;

    static bool classify_path(const char* path, SourceArbitrationSlot& slot) {
        using namespace pypilot_signalk_connector;
        if (streq(path, SignalKPath::wind_speed_apparent) || streq(path, SignalKPath::wind_angle_apparent)) {
            slot = SourceArbitrationSlot::wind_apparent;
            return true;
        }
        if (streq(path, SignalKPath::wind_speed_true) || streq(path, SignalKPath::wind_angle_true)) {
            slot = SourceArbitrationSlot::wind_true;
            return true;
        }
        if (streq(path, SignalKPath::gps_course_over_ground_true) ||
            streq(path, SignalKPath::gps_speed_over_ground) ||
            streq(path, SignalKPath::gps_position_latitude) ||
            streq(path, SignalKPath::gps_position_longitude)) {
            slot = SourceArbitrationSlot::gps;
            return true;
        }
        if (streq(path, SignalKPath::rudder_angle)) {
            slot = SourceArbitrationSlot::rudder;
            return true;
        }
        if (streq(path, SignalKPath::apb_heading_true)) {
            slot = SourceArbitrationSlot::apb;
            return true;
        }
        if (streq(path, SignalKPath::water_speed) || streq(path, SignalKPath::water_leeway)) {
            slot = SourceArbitrationSlot::water;
            return true;
        }
        return false;
    }

    bool can_accept_slot(SourceArbitrationSlot slot,
                         const ship_data_model::DataModel<Real>& model,
                         ship_data_model::SensorSource source,
                         const char* device_id,
                         uint64_t now_us) const {
        switch (slot) {
        case SourceArbitrationSlot::gps:
            return arbiter_.can_accept(slot, model.navigation.gps.source.value, model.navigation.gps.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::apb:
            return arbiter_.can_accept(slot, model.navigation.apb.source.value, model.navigation.apb.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::wind_apparent:
            return arbiter_.can_accept(slot, model.wind.apparent.source.value, model.wind.apparent.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::wind_true:
            return arbiter_.can_accept(slot, model.wind.truewind.source.value, model.wind.truewind.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::water:
            return arbiter_.can_accept(slot, model.water.source.value, model.water.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::rudder:
            return arbiter_.can_accept(slot, model.rudder.source.value, model.rudder.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::count:
            return true;
        }
        return true;
    }
};

} // namespace pypilot_sensors
