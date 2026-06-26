#include <cassert>
#include <cmath>
#include <pypilot_sensors.hpp>

using namespace pypilot_sensors;
using ship_data_model::SensorSource;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    {
        ship_data_model::DataModel<float> model;
        SensorSourceTimeoutManager<float> timeouts;

        assert(!timeouts.poll(model, 10000000ULL));
    }

    {
        ship_data_model::DataModel<float> model;
        SensorsManager<float> manager;

        SensorBatch<float> batch;
        batch.has_gps = true;
        batch.gps.time_us = 1000000ULL;
        batch.gps.source = SensorSource::serial;
        batch.gps.device_id = "gps0";
        batch.gps.speed_valid = true;
        batch.gps.speed_kn = 4.0f;
        batch.gps.track_valid = true;
        batch.gps.track_deg = 90.0f;
        assert(manager.apply_batch(model, batch));

        assert(!manager.poll_timeouts(model, 8000000ULL));
        assert(model.navigation.gps.source.value == SensorSource::serial);
        assert(model.navigation.gps.speed_kn.valid);
        assert(model.navigation.gps.track_deg.valid);

        assert(manager.poll_timeouts(model, 9000001ULL));
        assert(model.navigation.gps.source.value == SensorSource::none);
        assert(!model.navigation.gps.speed_kn.valid);
        assert(!model.navigation.gps.track_deg.valid);
        assert(model.navigation.gps.last_update_us == 0);
    }

    {
        ship_data_model::DataModel<float> model;
        SensorDataModelWriter<float> writer;
        SensorSourceTimeoutManager<float> timeouts;

        WindSample<float> wind;
        wind.time_us = 1000000ULL;
        wind.source = SensorSource::serial;
        wind.device_id = "wind0";
        wind.speed_valid = true;
        wind.speed_kn = 12.0f;
        wind.direction_valid = true;
        wind.direction_deg = 45.0f;
        assert(writer.write_wind(model, wind));

        WaterSample<float> water;
        water.time_us = 1000000ULL;
        water.source = SensorSource::serial;
        water.device_id = "water0";
        water.speed_valid = true;
        water.speed_kn = 5.5f;
        water.leeway_valid = true;
        water.leeway_deg = 2.0f;
        assert(writer.write_water(model, water));

        RudderSample<float> rudder;
        rudder.time_us = 1000000ULL;
        rudder.source = SensorSource::servo;
        rudder.device_id = "servo0";
        rudder.angle_valid = true;
        rudder.angle_deg = -10.0f;
        assert(writer.write_rudder(model, rudder));

        assert(timeouts.poll(model, 9000001ULL));
        assert(model.wind.apparent.source.value == SensorSource::none);
        assert(!model.wind.apparent.speed_kn.valid);
        assert(!model.wind.apparent.direction_deg.valid);
        assert(model.water.source.value == SensorSource::none);
        assert(model.water.leeway_source.value == SensorSource::none);
        assert(!model.water.speed_kn.valid);
        assert(!model.water.leeway_deg.valid);
        assert(model.rudder.source.value == SensorSource::none);
        assert(!model.rudder.angle_deg.valid);
    }

    {
        ship_data_model::DataModel<float> model;
        SensorDataModelWriter<float> writer;
        SensorSourceTimeoutManager<float> timeouts;

        ApbSample<float> apb;
        apb.time_us = 1000000ULL;
        apb.source = SensorSource::serial;
        apb.device_id = "apb0";
        apb.track_valid = true;
        apb.track_deg = 12.0f;
        apb.xte_valid = true;
        apb.xte_nmi = 0.25f;
        assert(writer.write_apb(model, apb));
        assert(model.navigation.apb.source.value == SensorSource::serial);
        assert(model.navigation.apb.track_deg.valid);
        assert(nearf(model.navigation.apb.xte_nmi.value, 0.25f));

        assert(timeouts.poll(model, 9000001ULL));
        assert(model.navigation.apb.source.value == SensorSource::none);
        assert(model.navigation.apb.track_deg.valid);
        assert(model.navigation.apb.xte_nmi.valid);
        assert(nearf(model.navigation.apb.xte_nmi.value, 0.0f));
        assert(model.navigation.apb.last_update_us == 0);
    }

    return 0;
}
