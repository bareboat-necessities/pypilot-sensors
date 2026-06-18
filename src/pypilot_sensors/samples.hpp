#pragma once

#include <stdint.h>
#include <pypilot_data_model.hpp>

namespace pypilot_sensors {

template<typename Real = float>
struct ImuSample {
    uint64_t time_us = 0;
    const char* device_id = 0;
    bool heading_valid = false;
    Real heading_deg = Real(0);
    bool heading_rate_valid = false;
    Real heading_rate_deg_s = Real(0);
    bool heading_rate_rate_valid = false;
    Real heading_rate_rate_deg_s2 = Real(0);
    bool pitch_valid = false;
    Real pitch_deg = Real(0);
    bool roll_valid = false;
    Real roll_deg = Real(0);
    bool heel_valid = false;
    Real heel_deg = Real(0);
    bool accel_valid = false;
    pypilot_data_model::Vec3<Real> accel_g;
    bool gyro_valid = false;
    pypilot_data_model::Vec3<Real> gyro_deg_s;
    bool compass_valid = false;
    pypilot_data_model::Vec3<Real> compass_raw;
};

template<typename Real = float>
struct GpsSample {
    uint64_t time_us = 0;
    const char* device_id = 0;
    pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::none;
    bool speed_valid = false;
    Real speed_kn = Real(0);
    bool track_valid = false;
    Real track_deg = Real(0);
    bool fix_valid = false;
    Real latitude_deg = Real(0);
    Real longitude_deg = Real(0);
    bool altitude_valid = false;
    Real altitude_m = Real(0);
    bool declination_valid = false;
    Real declination_deg = Real(0);
};

template<typename Real = float>
struct ApbSample {
    uint64_t time_us = 0;
    const char* device_id = 0;
    pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::none;
    bool track_valid = false;
    Real track_deg = Real(0);
    bool xte_valid = false;
    Real xte_nmi = Real(0);
};

template<typename Real = float>
struct WindSample {
    uint64_t time_us = 0;
    const char* device_id = 0;
    pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::none;
    bool true_wind = false;
    bool speed_valid = false;
    Real speed_kn = Real(0);
    bool direction_valid = false;
    Real direction_deg = Real(0);
};

template<typename Real = float>
struct WaterSample {
    uint64_t time_us = 0;
    const char* device_id = 0;
    pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::none;
    bool speed_valid = false;
    Real speed_kn = Real(0);
    bool leeway_valid = false;
    Real leeway_deg = Real(0);
    bool current_valid = false;
    Real current_speed_kn = Real(0);
    Real current_direction_deg = Real(0);
};

template<typename Real = float>
struct RudderSample {
    uint64_t time_us = 0;
    const char* device_id = 0;
    pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::none;
    bool angle_valid = false;
    Real angle_deg = Real(0);
    bool speed_valid = false;
    Real speed_deg_s = Real(0);
    bool raw_valid = false;
    Real raw_0_1 = Real(0);
};

template<typename Real = float>
struct ServoTelemetrySample {
    uint64_t time_us = 0;
    const char* device_id = 0;
    bool flags_valid = false;
    uint32_t flags = 0;
    bool voltage_valid = false;
    Real voltage_v = Real(0);
    bool current_valid = false;
    Real current_a = Real(0);
    bool controller_temp_valid = false;
    Real controller_temp_c = Real(0);
    bool motor_temp_valid = false;
    Real motor_temp_c = Real(0);
    RudderSample<Real> rudder;
};

template<typename Real = float>
struct SensorBatch {
    bool has_imu = false;
    ImuSample<Real> imu;
    bool has_gps = false;
    GpsSample<Real> gps;
    bool has_apb = false;
    ApbSample<Real> apb;
    bool has_wind = false;
    WindSample<Real> wind;
    bool has_water = false;
    WaterSample<Real> water;
    bool has_rudder = false;
    RudderSample<Real> rudder;
    bool has_servo = false;
    ServoTelemetrySample<Real> servo;
    void clear() {
        has_imu = has_gps = has_apb = has_wind = has_water = has_rudder = has_servo = false;
        imu = ImuSample<Real>();
        gps = GpsSample<Real>();
        apb = ApbSample<Real>();
        wind = WindSample<Real>();
        water = WaterSample<Real>();
        rudder = RudderSample<Real>();
        servo = ServoTelemetrySample<Real>();
    }
};

} // namespace pypilot_sensors
