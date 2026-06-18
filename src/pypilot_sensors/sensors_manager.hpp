#pragma once

#include <pypilot_data_model.hpp>
#include "samples.hpp"
#include "data_model_writer.hpp"
#include "source_timeout.hpp"

namespace pypilot_sensors {

template<typename Real = float>
class SensorsManager {
public:
    SensorsManager() {}

    bool apply_batch(pypilot_data_model::DataModel<Real>& model, const SensorBatch<Real>& batch) {
        return writer_.write_batch(model, batch);
    }

    bool poll_timeouts(pypilot_data_model::DataModel<Real>& model, uint64_t now_us) const {
        return timeout_manager_.poll(model, now_us);
    }

    void reset_arbitration() { writer_.reset_arbitration(); }

    void set_source_timeout_us(uint64_t timeout_us) {
        writer_.set_source_timeout_us(timeout_us);
        timeout_manager_.set_timeout_us(timeout_us);
    }

    uint64_t source_timeout_us() const { return timeout_manager_.timeout_us(); }

private:
    SensorDataModelWriter<Real> writer_;
    SensorSourceTimeoutManager<Real> timeout_manager_;
};

} // namespace pypilot_sensors
