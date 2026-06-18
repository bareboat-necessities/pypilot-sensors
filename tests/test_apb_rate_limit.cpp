#include <cassert>
#include <cmath>
#include <pypilot_sensors.hpp>
#include <pypilot_signalk_connector.hpp>

using namespace pypilot_sensors;
using pypilot_data_model::SensorSource;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    {
        pypilot_data_model::DataModel<float> model;
        SensorDataModelWriter<float> writer;

        ApbSample<float> apb;
        apb.time_us = 1000000ULL;
        apb.device_id = "apb0";
        apb.source = SensorSource::serial;
        apb.track_valid = true;
        apb.track_deg = 10.0f;
        assert(writer.write_apb(model, apb));
        assert(model.navigation.apb.source.value == SensorSource::serial);
        assert(model.navigation.apb.last_update_us == 1000000ULL);
        assert(nearf(model.navigation.apb.track_deg.value, 10.0f));

        apb.time_us = 1200000ULL;
        apb.track_deg = 20.0f;
        assert(!writer.write_apb(model, apb));
        assert(model.navigation.apb.last_update_us == 1000000ULL);
        assert(nearf(model.navigation.apb.track_deg.value, 10.0f));

        apb.time_us = 1500000ULL;
        apb.track_deg = 30.0f;
        assert(writer.write_apb(model, apb));
        assert(model.navigation.apb.last_update_us == 1500000ULL);
        assert(nearf(model.navigation.apb.track_deg.value, 30.0f));
    }

    {
        pypilot_data_model::DataModel<float> model;
        SignalKSensorAdapter<float> adapter;

        assert(adapter.apply_number(pypilot_signalk_connector::SignalKPath::apb_heading_true,
                                    1.0f,
                                    model,
                                    2000000ULL,
                                    SensorSource::signalk,
                                    "sk0"));
        const float first_track = model.navigation.apb.track_deg.value;
        assert(model.navigation.apb.source.value == SensorSource::signalk);
        assert(model.navigation.apb.last_update_us == 2000000ULL);

        assert(!adapter.apply_number(pypilot_signalk_connector::SignalKPath::apb_heading_true,
                                     2.0f,
                                     model,
                                     2200000ULL,
                                     SensorSource::signalk,
                                     "sk0"));
        assert(model.navigation.apb.last_update_us == 2000000ULL);
        assert(nearf(model.navigation.apb.track_deg.value, first_track));

        assert(adapter.apply_number(pypilot_signalk_connector::SignalKPath::apb_heading_true,
                                    2.0f,
                                    model,
                                    2500000ULL,
                                    SensorSource::signalk,
                                    "sk0"));
        assert(model.navigation.apb.last_update_us == 2500000ULL);
        assert(!nearf(model.navigation.apb.track_deg.value, first_track));
    }

    return 0;
}
