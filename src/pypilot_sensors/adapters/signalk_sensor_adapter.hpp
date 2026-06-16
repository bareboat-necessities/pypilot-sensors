#pragma once

#include <pypilot_signalk_connector.hpp>
#include <pypilot_data_model.hpp>

namespace pypilot_sensors {

template<typename Real = float>
class SignalKSensorAdapter {
public:
    SignalKSensorAdapter() : last_error_("") {}

    const char* last_error() const { return last_error_; }

    bool apply_number(const char* path,
                      Real value,
                      pypilot_data_model::DataModel<Real>& model,
                      uint64_t now_us,
                      pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::signalk) {
        last_error_ = "";
        if (!connector_.apply_number(path, value, model, now_us, source)) {
            last_error_ = connector_.last_error();
            return false;
        }
        return true;
    }

    bool apply_put_number(const char* path, Real value, pypilot_data_model::DataModel<Real>& model, uint64_t now_us) {
        last_error_ = "";
        if (!connector_.apply_put_number(path, value, model, now_us)) {
            last_error_ = connector_.last_error();
            return false;
        }
        return true;
    }

    bool apply_put_bool(const char* path, bool value, pypilot_data_model::DataModel<Real>& model, uint64_t now_us) {
        last_error_ = "";
        if (!connector_.apply_put_bool(path, value, model, now_us)) {
            last_error_ = connector_.last_error();
            return false;
        }
        return true;
    }

    bool apply_put_string(const char* path, const char* value, pypilot_data_model::DataModel<Real>& model, uint64_t now_us) {
        last_error_ = "";
        if (!connector_.apply_put_string(path, value, model, now_us)) {
            last_error_ = connector_.last_error();
            return false;
        }
        return true;
    }

private:
    pypilot_signalk_connector::SignalKConnector<Real> connector_;
    const char* last_error_;
};

} // namespace pypilot_sensors
