#pragma once

#include <pypilot_data_model.hpp>
#include "samples.hpp"
#include "data_model_writer.hpp"

namespace pypilot_sensors {

template<typename Real = float>
class SensorsManager {
public:
    SensorsManager() {}

    void apply_batch(pypilot_data_model::DataModel<Real>& model, const SensorBatch<Real>& batch) {
        writer_.write_batch(model, batch);
    }

private:
    SensorDataModelWriter<Real> writer_;
};

} // namespace pypilot_sensors
