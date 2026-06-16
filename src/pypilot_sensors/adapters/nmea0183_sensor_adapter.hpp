#pragma once

#include <pypilot_nmea0183_connector.hpp>
#include <pypilot_data_model.hpp>

namespace pypilot_sensors {

template<typename Real = float>
class Nmea0183SensorAdapter {
public:
    Nmea0183SensorAdapter() : last_error_("") {}

    const char* last_error() const { return last_error_; }

    bool apply_sentence(const pypilot_nmea0183_connector::NmeaSentence& sentence,
                        pypilot_data_model::DataModel<Real>& model,
                        uint64_t now_us,
                        pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::serial) {
        last_error_ = "";
        if (!connector_.apply_sentence(sentence, model, now_us)) {
            last_error_ = connector_.last_error();
            return false;
        }
        tag_source(sentence, model, now_us, source);
        return true;
    }

    bool parse_and_apply_line(const char* line,
                              pypilot_data_model::DataModel<Real>& model,
                              uint64_t now_us,
                              pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::serial) {
        pypilot_nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(line, sentence)) {
            last_error_ = parser_.last_error();
            return false;
        }
        return apply_sentence(sentence, model, now_us, source);
    }

private:
    pypilot_nmea0183_connector::Nmea0183Connector<Real> connector_;
    pypilot_nmea0183_connector::Nmea0183StreamParser parser_;
    const char* last_error_;

    static void tag_source(const pypilot_nmea0183_connector::NmeaSentence& s,
                           pypilot_data_model::DataModel<Real>& model,
                           uint64_t now_us,
                           pypilot_data_model::SensorSource source) {
        using namespace pypilot_nmea0183_connector;
        if (formatter_is(s, "RMC") || formatter_is(s, "GGA") || formatter_is(s, "VTG")) {
            model.navigation.gps.source.value = source;
            model.navigation.gps.last_update_us = now_us;
        } else if (formatter_is(s, "APB") || formatter_is(s, "XTE")) {
            model.navigation.apb.source.value = source;
            model.navigation.apb.last_update_us = now_us;
        } else if (formatter_is(s, "MWV")) {
            if (s.field_count > 1 && s.field(1)[0] == 'T') {
                model.wind.truewind.source.value = source;
                model.wind.truewind.last_update_us = now_us;
            } else {
                model.wind.apparent.source.value = source;
                model.wind.apparent.last_update_us = now_us;
            }
        } else if (formatter_is(s, "VWR")) {
            model.wind.apparent.source.value = source;
            model.wind.apparent.last_update_us = now_us;
        } else if (formatter_is(s, "VWT")) {
            model.wind.truewind.source.value = source;
            model.wind.truewind.last_update_us = now_us;
        } else if (formatter_is(s, "VHW")) {
            model.water.source.value = source;
            model.water.last_update_us = now_us;
        } else if (formatter_is(s, "RSA")) {
            model.rudder.source.value = source;
            model.rudder.last_update_us = now_us;
        }
    }
};

} // namespace pypilot_sensors
