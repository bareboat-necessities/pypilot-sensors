#pragma once

#include <stdint.h>
#include <string.h>

#include <pypilot_data_model.hpp>
#include <pypilot_algorithms.hpp>

namespace pypilot_sensors {

static const uint64_t default_source_device_timeout_us = 8000000ULL;

enum class SourceArbitrationSlot : uint8_t {
    gps = 0,
    apb,
    wind_apparent,
    wind_true,
    water,
    rudder,
    count
};

inline bool sensor_source_is_live(uint64_t now_us, uint64_t last_update_us, uint64_t timeout_us) {
    return !pypilot_algorithms::pypilot_source_is_stale(now_us, last_update_us, timeout_us);
}

inline bool sensor_device_id_empty(const char* device_id) {
    return device_id == 0 || device_id[0] == '\0';
}

inline bool sensor_device_id_equal(const char* a, const char* b) {
    if (a == 0) a = "";
    if (b == 0) b = "";
    return strncmp(a, b, 32) == 0;
}

class SensorDevicePolicy {
public:
    SensorDevicePolicy() : source_(pypilot_data_model::SensorSource::none), has_device_(false) {
        device_[0] = '\0';
    }

    void reset() {
        source_ = pypilot_data_model::SensorSource::none;
        has_device_ = false;
        device_[0] = '\0';
    }

    bool accepts(pypilot_data_model::SensorSource current_source,
                 uint64_t current_last_update_us,
                 pypilot_data_model::SensorSource incoming_source,
                 const char* incoming_device,
                 uint64_t now_us,
                 uint64_t timeout_us = default_source_device_timeout_us) const {
        const bool current_live = current_source != pypilot_data_model::SensorSource::none &&
                                  sensor_source_is_live(now_us, current_last_update_us, timeout_us);

        if (incoming_source == pypilot_data_model::SensorSource::none) {
            return !current_live;
        }
        if (!current_live || current_source == pypilot_data_model::SensorSource::none) {
            return true;
        }

        const int incoming_priority = pypilot_data_model::source_priority(incoming_source);
        const int current_priority = pypilot_data_model::source_priority(current_source);
        if (incoming_priority < current_priority) {
            return true;
        }
        if (incoming_priority > current_priority) {
            return false;
        }

        if (current_source == incoming_source && has_device_) {
            return !sensor_device_id_empty(incoming_device) && sensor_device_id_equal(device_, incoming_device);
        }
        return true;
    }

    void accepted(pypilot_data_model::SensorSource incoming_source, const char* incoming_device) {
        if (incoming_source == pypilot_data_model::SensorSource::none) {
            reset();
            return;
        }
        source_ = incoming_source;
        if (sensor_device_id_empty(incoming_device)) {
            has_device_ = false;
            device_[0] = '\0';
            return;
        }
        copy_device(incoming_device);
        has_device_ = true;
    }

    pypilot_data_model::SensorSource source() const { return source_; }
    const char* device_id() const { return has_device_ ? device_ : ""; }
    bool has_device() const { return has_device_; }

private:
    void copy_device(const char* incoming_device) {
        size_t i = 0;
        for (; i + 1 < sizeof(device_) && incoming_device[i] != '\0'; ++i) {
            device_[i] = incoming_device[i];
        }
        device_[i] = '\0';
    }

    pypilot_data_model::SensorSource source_;
    bool has_device_;
    char device_[32];
};

class SourceDeviceArbitrator {
public:
    SourceDeviceArbitrator() : timeout_us_(default_source_device_timeout_us) {}

    void reset() {
        for (uint8_t i = 0; i < static_cast<uint8_t>(SourceArbitrationSlot::count); ++i) {
            policies_[i].reset();
        }
    }

    void set_timeout_us(uint64_t timeout_us) { timeout_us_ = timeout_us; }
    uint64_t timeout_us() const { return timeout_us_; }

    bool can_accept(SourceArbitrationSlot slot,
                    pypilot_data_model::SensorSource current_source,
                    uint64_t current_last_update_us,
                    pypilot_data_model::SensorSource incoming_source,
                    const char* incoming_device,
                    uint64_t now_us) const {
        return policy(slot).accepts(current_source,
                                    current_last_update_us,
                                    incoming_source,
                                    incoming_device,
                                    now_us,
                                    timeout_us_);
    }

    void accepted(SourceArbitrationSlot slot,
                  pypilot_data_model::SensorSource incoming_source,
                  const char* incoming_device) {
        policy(slot).accepted(incoming_source, incoming_device);
    }

    bool accept(SourceArbitrationSlot slot,
                pypilot_data_model::SensorSource current_source,
                uint64_t current_last_update_us,
                pypilot_data_model::SensorSource incoming_source,
                const char* incoming_device,
                uint64_t now_us) {
        if (!can_accept(slot, current_source, current_last_update_us, incoming_source, incoming_device, now_us)) {
            return false;
        }
        accepted(slot, incoming_source, incoming_device);
        return true;
    }

private:
    SensorDevicePolicy& policy(SourceArbitrationSlot slot) {
        return policies_[static_cast<uint8_t>(slot)];
    }

    const SensorDevicePolicy& policy(SourceArbitrationSlot slot) const {
        return policies_[static_cast<uint8_t>(slot)];
    }

    SensorDevicePolicy policies_[static_cast<uint8_t>(SourceArbitrationSlot::count)];
    uint64_t timeout_us_;
};

} // namespace pypilot_sensors
