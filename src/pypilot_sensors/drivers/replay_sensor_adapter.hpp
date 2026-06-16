#pragma once

#include <stddef.h>
#include "../samples.hpp"

namespace pypilot_sensors {

template<typename Real = float, size_t Capacity = 16>
class ReplaySensorAdapter {
public:
    ReplaySensorAdapter() : count_(0), index_(0) {}

    bool push(const SensorBatch<Real>& batch) {
        if (count_ >= Capacity) return false;
        batches_[count_++] = batch;
        return true;
    }

    void reset() { index_ = 0; }

    bool poll(SensorBatch<Real>& out) {
        if (index_ >= count_) return false;
        out = batches_[index_++];
        return true;
    }

    size_t size() const { return count_; }
    size_t index() const { return index_; }

private:
    SensorBatch<Real> batches_[Capacity];
    size_t count_;
    size_t index_;
};

} // namespace pypilot_sensors
