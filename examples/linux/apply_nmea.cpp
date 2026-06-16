#include <iostream>
#include <pypilot_sensors.hpp>

int main() {
    pypilot_data_model::DataModel<float> model;
    pypilot_sensors::Nmea0183SensorAdapter<float> adapter;
    if (adapter.parse_and_apply_line("$IIVHW,,,,,5.50,N,,*19", model, 1000)) {
        std::cout << "water speed kn=" << model.water.speed_kn.value << "\n";
        return 0;
    }
    std::cerr << adapter.last_error() << "\n";
    return 1;
}
