#include <cassert>
#include <pypilot_sensors.hpp>

using namespace pypilot_sensors;

int main() {
    ReplaySensorAdapter<float, 2> replay;
    SensorBatch<float> batch;
    batch.has_gps = true;
    batch.gps.time_us = 100;
    batch.gps.source = pypilot_data_model::SensorSource::serial;
    batch.gps.speed_valid = true;
    batch.gps.speed_kn = 2.5f;

    assert(replay.push(batch));
    assert(!replay.push(batch) || replay.size() <= 2);

    SensorBatch<float> out;
    assert(replay.poll(out));
    assert(out.has_gps);
    assert(out.gps.speed_kn == 2.5f);
    return 0;
}
