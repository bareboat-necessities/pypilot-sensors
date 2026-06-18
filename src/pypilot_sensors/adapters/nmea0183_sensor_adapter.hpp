#pragma once

#include <pypilot_nmea0183_connector.hpp>
#include <pypilot_data_model.hpp>
#include "../source_policy.hpp"

namespace pypilot_sensors {

template<typename Real = float>
class Nmea0183SensorAdapter {
public:
    Nmea0183SensorAdapter() : last_error_("") {}

    const char* last_error() const { return last_error_; }

    bool apply_sentence(const pypilot_nmea0183_connector::NmeaSentence& sentence,
                        pypilot_data_model::DataModel<Real>& model,
                        uint64_t now_us,
                        pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::serial,
                        const char* device_id = 0) {
        last_error_ = "";
        SourceArbitrationSlot slot;
        const bool has_slot = classify_sentence(sentence, slot);
        if (has_slot && !can_accept_slot(slot, model, source, device_id, now_us)) {
            last_error_ = "source rejected";
            return false;
        }
        if (!connector_.apply_sentence(sentence, model, now_us, source)) {
            last_error_ = connector_.last_error();
            return false;
        }
        normalize_sentence_output(sentence, model, now_us);
        if (has_slot) arbiter_.accepted(slot, source, device_id);
        tag_source(sentence, model, now_us, source);
        return true;
    }

    bool parse_and_apply_line(const char* line,
                              pypilot_data_model::DataModel<Real>& model,
                              uint64_t now_us,
                              pypilot_data_model::SensorSource source = pypilot_data_model::SensorSource::serial,
                              const char* device_id = 0) {
        pypilot_nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(line, sentence)) {
            last_error_ = parser_.last_error();
            return false;
        }
        return apply_sentence(sentence, model, now_us, source, device_id);
    }

    void reset_arbitration() { arbiter_.reset(); }

private:
    pypilot_nmea0183_connector::Nmea0183Connector<Real> connector_;
    pypilot_nmea0183_connector::Nmea0183StreamParser parser_;
    SourceDeviceArbitrator arbiter_;
    const char* last_error_;

    static bool classify_sentence(const pypilot_nmea0183_connector::NmeaSentence& s,
                                  SourceArbitrationSlot& slot) {
        using namespace pypilot_nmea0183_connector;
        if (formatter_is(s, "RMC") || formatter_is(s, "GGA") || formatter_is(s, "VTG")) {
            slot = SourceArbitrationSlot::gps;
            return true;
        }
        if (formatter_is(s, "APB") || formatter_is(s, "XTE")) {
            slot = SourceArbitrationSlot::apb;
            return true;
        }
        if (formatter_is(s, "MWV")) {
            slot = (s.field_count > 1 && s.field(1)[0] == 'T') ? SourceArbitrationSlot::wind_true : SourceArbitrationSlot::wind_apparent;
            return true;
        }
        if (formatter_is(s, "VWR")) {
            slot = SourceArbitrationSlot::wind_apparent;
            return true;
        }
        if (formatter_is(s, "VWT")) {
            slot = SourceArbitrationSlot::wind_true;
            return true;
        }
        if (formatter_is(s, "VHW")) {
            slot = SourceArbitrationSlot::water;
            return true;
        }
        if (formatter_is(s, "RSA")) {
            slot = SourceArbitrationSlot::rudder;
            return true;
        }
        return false;
    }

    bool can_accept_slot(SourceArbitrationSlot slot,
                         const pypilot_data_model::DataModel<Real>& model,
                         pypilot_data_model::SensorSource source,
                         const char* device_id,
                         uint64_t now_us) const {
        switch (slot) {
        case SourceArbitrationSlot::gps:
            return arbiter_.can_accept(slot, model.navigation.gps.source.value, model.navigation.gps.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::apb:
            return arbiter_.can_accept(slot, model.navigation.apb.source.value, model.navigation.apb.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::wind_apparent:
            return arbiter_.can_accept(slot, model.wind.apparent.source.value, model.wind.apparent.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::wind_true:
            return arbiter_.can_accept(slot, model.wind.truewind.source.value, model.wind.truewind.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::water:
            return arbiter_.can_accept(slot, model.water.source.value, model.water.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::rudder:
            return arbiter_.can_accept(slot, model.rudder.source.value, model.rudder.last_update_us, source, device_id, now_us);
        case SourceArbitrationSlot::count:
            return true;
        }
        return true;
    }

    static void normalize_sentence_output(const pypilot_nmea0183_connector::NmeaSentence& s,
                                          pypilot_data_model::DataModel<Real>& model,
                                          uint64_t now_us) {
        using namespace pypilot_nmea0183_connector;
        if (formatter_is(s, "RSA") &&
            model.rudder.angle_deg.valid &&
            model.rudder.angle_deg.last_update_us == now_us) {
            model.rudder.angle_deg.value = -model.rudder.angle_deg.value;
        }
    }

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
