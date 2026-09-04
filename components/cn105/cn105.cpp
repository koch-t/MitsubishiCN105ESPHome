
#include "cn105.h"
#include <cstdio>
#include <ctime>
#ifdef USE_ESP32
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace {

std::string decode_redlink_6bit_string_(const uint8_t* data, size_t byte_count, size_t character_count) {
    std::string result;
    result.reserve(character_count);
    for (size_t character = 0; character < character_count; character++) {
        const size_t start_bit = character * 6;
        if ((start_bit + 6) > (byte_count * 8)) break;

        uint8_t value = 0;
        for (size_t bit = 0; bit < 6; bit++) {
            const size_t absolute_bit = start_bit + bit;
            value = static_cast<uint8_t>((value << 1) |
                ((data[absolute_bit / 8] >> (7 - (absolute_bit % 8))) & 0x01));
        }
        if (value <= 0x1F) value = static_cast<uint8_t>(value + 0x40);
        result.push_back(static_cast<char>(value));
    }
    return result;
}

}  // namespace

using namespace esphome;

const char* esphome::driver_state_to_str(DriverState s) {
    switch (s) {
        case DriverState::BOOT:         return "BOOT";
        case DriverState::WAIT_WIFI:    return "WAIT_WIFI";
        case DriverState::WAIT_GRACE:   return "WAIT_GRACE";
        case DriverState::CONNECTING:   return "CONNECTING";
        case DriverState::CONNECTED:    return "CONNECTED";
        case DriverState::DISCONNECTED: return "DISCONNECTED";
        default:                        return "UNKNOWN";
    }
}

void CN105Climate::transition_to_(DriverState next) {
    if (state_ == next) return;
    ESP_LOGI("FSM", "State: %s -> %s", driver_state_to_str(state_), driver_state_to_str(next));
    state_ = next;
}


CN105Climate::CN105Climate(uart::UARTComponent* uart) :
    UARTDevice(uart),
    scheduler_(
        // send callback: send a packet via buildAndSendInfoPacket
        [this](uint8_t code) { this->buildAndSendInfoPacket(code); },
        // timeout_callback: uses set_timeout from component
        [this](const std::string& name, uint32_t timeout_ms, std::function<void()> callback) {
            this->set_timeout(name.c_str(), timeout_ms, std::move(callback));
        },
        // terminate_callback: completes the cycle
        [this]() { this->terminateCycle(); },
        // context_callback: Returns 'this' for the 'canSend' and 'onResponse' callbacks.
        [this]() -> CN105Climate* { return this; }
    ) {

    // Enables feature flags via the modern API (avoids deprecated setters).
    this->traits_.add_feature_flags(
        climate::CLIMATE_SUPPORTS_ACTION |
        climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE
    );
    // supports_two_point_target_temperature will be defined in setup() depending on the supported modes
    this->traits_.set_visual_min_temperature(ESPMHP_MIN_TEMPERATURE);
    this->traits_.set_visual_max_temperature(ESPMHP_MAX_TEMPERATURE);
    this->traits_.set_visual_temperature_step(ESPMHP_TEMPERATURE_STEP);


    // state_ is initialized to BOOT in the header
    this->use_temperature_encoding_b_ = false;
    this->wideVaneAdj = false;
    this->functions = heatpumpFunctions();
    this->autoUpdate = false;
    this->firstRun = true;
    this->externalUpdate = false;
    this->lastSend = 0;
    this->infoMode = 0;
    this->lastConnectRqTimeMs = 0;
    // currentStatus fields are now default-initialized via heatpumpStatus struct defaults
    this->tx_pin_ = -1;
    this->rx_pin_ = -1;

    this->horizontal_vane_select_ = nullptr;
    this->vertical_vane_select_ = nullptr;
    this->airflow_control_select_ = nullptr;
    this->compressor_frequency_sensor_ = nullptr;
    this->target_humidity_sensor_ = nullptr;
    this->redlink_thermostat_humidity_sensor_ = nullptr;
    this->redlink_thermostat_battery_sensor_ = nullptr;
    this->redlink_thermostat_model_sensor_ = nullptr;
    this->redlink_thermostat_serial_sensor_ = nullptr;
    this->redlink_thermostat_firmware_sensor_ = nullptr;
    this->redlink_thermostat_temperature_source_sensor_ = nullptr;
    this->redlink_connection_sensor_ = nullptr;
    this->redlink_packet_age_sensor_ = nullptr;
    this->redlink_rx_packet_count_sensor_ = nullptr;
    this->redlink_tx_packet_count_sensor_ = nullptr;
    this->redlink_timeout_count_sensor_ = nullptr;
    this->redlink_last_control_source_sensor_ = nullptr;
    this->input_power_sensor_ = nullptr;
    this->kwh_sensor_ = nullptr;
    this->runtime_hours_sensor_ = nullptr;

    this->air_purifier_switch_ = nullptr;
    this->night_mode_switch_ = nullptr;
    this->circulator_switch_ = nullptr;

    this->powerRequestWithoutResponses = 0;     // power request is not supported by all heatpump #112

    this->remote_temp_timeout_ = 4294967295;    // uint32_t max
    this->generateExtraComponents();
    this->loopCycle.init();
    this->wantedSettings.resetSettings();
    this->wantedRunStates.resetSettings();
#ifndef USE_ESP32
    this->wantedSettingsMutex = false;
#endif

    // Register info requests moved to setup() to ensure hardware_settings_ are populated
}

void CN105Climate::registerInfoRequests() {
    scheduler_.clear_requests();

    // 0x02 Settings
    InfoRequest r_settings("settings", "Settings", 0x02, 3, 0);
    r_settings.onResponse = [this](CN105Climate& self) { (void)self; this->getSettingsFromResponsePacket(); };
    scheduler_.register_request(r_settings);

    // 0x03 Room temperature
    InfoRequest r_room("room_temp", "Room temperature", 0x03, 3, 0);
    r_room.onResponse = [this](CN105Climate& self) { (void)self; this->getRoomTemperatureFromResponsePacket(); };
    scheduler_.register_request(r_room);

    // 0x06 Status
    InfoRequest r_status("status", "Status", 0x06, 3, 0);
    r_status.onResponse = [this](CN105Climate& self) { (void)self; this->getOperatingAndCompressorFreqFromResponsePacket(); };
    scheduler_.register_request(r_status);

    // 0x09 Standby/Power
    InfoRequest r_power("standby", "Power/Standby", 0x09, 3, 500);
    r_power.onResponse = [this](CN105Climate& self) { (void)self; this->getPowerFromResponsePacket(); };
    scheduler_.register_request(r_power);

    // 0x42 HVAC options
    InfoRequest r_hvac_opts("hvac_options", "HVAC options", 0x42, 3, 500);
    r_hvac_opts.canSend = [this](const CN105Climate& self) {
        (void)self;
        return (this->air_purifier_switch_ != nullptr || this->night_mode_switch_ != nullptr || this->circulator_switch_ != nullptr);
        };
    r_hvac_opts.onResponse = [this](CN105Climate& self) { (void)self; this->getHVACOptionsFromResponsePacket(); };
    scheduler_.register_request(r_hvac_opts);

    // Placeholders
    InfoRequest r_error_info("error_info", "Error Info", 0x04, 3, 0);
    r_error_info.onResponse = [this](CN105Climate& self) { (void)self; this->getErrorInfoFromResponsePacket(); };
    scheduler_.register_request(r_error_info);

    InfoRequest r_timers("timers", "Timers", 0x05, 1, 0);
    r_timers.disabled = true;
    scheduler_.register_request(r_timers);

    // Call to the new dedicated method.
    this->registerHardwareSettingsRequests();
}

void CN105Climate::registerHardwareSettingsRequests() {
    uint32_t interval = 0;
    bool is_enabled = false;

    if (!this->hardware_settings_.empty()) {
        ESP_LOGI(LOG_FUNCTIONS_TAG, "Registering function settings requests (0x20/0x22) with interval %" PRIu32 " ms", this->hardware_settings_interval_ms_);
        interval = this->hardware_settings_interval_ms_;
        is_enabled = true;
    }
    else {
        ESP_LOGI(LOG_FUNCTIONS_TAG, "Registering function settings requests (0x20/0x22), disabled");
    }

    // Helper Lambda: Checks for incompatibility and disables everything if necessary.
    auto check_and_disable = [](CN105Climate& self, uint8_t code) -> bool {
        if (self.data[0] != code) return false;

        bool all_zeros = true;
        // On some units (e.g. SEZ), codes may be present with a value of zero as long as the session
        // is not in installer mode. The presence of the byte (code+value) just validate the support.
        for (int i = 1; i < self.parser_.data_length(); i++) {
            if (self.data[i] != 0) {
                all_zeros = false;
                break;
            }
        }

        if (all_zeros) {
            ESP_LOGW(LOG_FUNCTIONS_TAG, "Response 0x%02X contains only zeros. Feature not supported by unit. Disabling.", code);

            // 1. Do activate the request via the scheduler.
            self.scheduler_.disable_request(code);

            // 2. Mark graphics components as failed (unavailable).
            ESP_LOGD(LOG_FUNCTIONS_TAG, "Marking Hardware Setting Selects as failed.");
            for (auto* setting : self.hardware_settings_) {
                setting->set_enabled(false);
            }

            return false;
        }

        // If no hardware settings are defined in YAML this was a manual request
        // that is expected to run once, disable future requests.
        if (self.hardware_settings_.empty()) {
            self.scheduler_.disable_request(code);
        }

        return true;
        };

    // --- Part 1 (0x20) ---
    InfoRequest r_funcs1("functions1", "Functions Part 1", 0x20, 3, 0, interval, LOG_FUNCTIONS_TAG);
    r_funcs1.onResponse = [this, check_and_disable](CN105Climate& self) {
        // Log the raw packet and decoded pairs even if the unit returns all zeros
        self.hpPacketDebug(self.data, self.parser_.data_length(), "RX 0x20");
        self.hpFunctionsDebug(self.data, self.parser_.data_length());
        if (check_and_disable(self, 0x20)) {
            self.functions.setData1(&self.data[1]);
            ESP_LOGD(LOG_FUNCTIONS_TAG, "Got functions packet 1 (via InfoRequest)");
        }
        };
    scheduler_.register_request(r_funcs1);
    if (!is_enabled) {
        scheduler_.disable_request(0x20);
    }

    // --- Part 2 (0x22) ---
    InfoRequest r_funcs2("functions2", "Functions Part 2", 0x22, 3, 0, interval, LOG_FUNCTIONS_TAG);
    r_funcs2.onResponse = [this, check_and_disable](CN105Climate& self) {
        // Log the raw packet and decoded pairs even if the unit returns all zeros
        self.hpPacketDebug(self.data, self.parser_.data_length(), "RX 0x22");
        self.hpFunctionsDebug(self.data, self.parser_.data_length());
        if (check_and_disable(self, 0x22)) {
            self.functions.setData2(&self.data[1]);
            ESP_LOGD(LOG_FUNCTIONS_TAG, "Got functions packet 2 (via InfoRequest)");
            self.functionsArrived();
        }
        };
    scheduler_.register_request(r_funcs2);
    if (!is_enabled) {
        scheduler_.disable_request(0x22);
    }

}

// The sendInfoRequest, markResponseSeenFor, sendNextAfter, and processInfoResponse methods
// have been placed in RequestScheduler to comply with the Single Responsibility Principle (SRP).



void CN105Climate::set_baud_rate(int baud) {
    this->baud_ = baud;
    ESP_LOGI(TAG, "setting baud rate to: %d", baud);
}

void CN105Climate::set_tx_rx_pins(int tx_pin, int rx_pin) {
    this->tx_pin_ = tx_pin;
    this->rx_pin_ = rx_pin;
    ESP_LOGI(TAG, "setting tx_pin: %d rx_pin: %d", tx_pin, rx_pin);

}

void CN105Climate::set_redlink_uart(uart::UARTComponent* redlink_uart) {
    this->redlink_uart_ = redlink_uart;
    this->redlink_parser_.reset();
    ESP_LOGI(LOG_CONN_TAG, "MIFH2 RedLINK bridge enabled on second UART");
}

bool CN105Climate::is_response_command_(uint8_t command) const {
    return command == 0x61 || command == 0x62 || command == 0x7A || command == 0x7B;
}

bool CN105Climate::redlink_bridge_busy_() const {
    return this->redlink_uart_ != nullptr &&
        (this->redlink_transaction_active_ || this->has_pending_redlink_frame_ ||
         this->redlink_local_transaction_active_ || this->has_pending_local_redlink_frame_ ||
         this->redlink_parser_.in_frame());
}

void CN105Climate::send_frame_(uart::UARTComponent* serial, const uint8_t* frame, int length) {
    if (serial == nullptr || frame == nullptr || length <= 0) return;
    for (int i = 0; i < length; i++) {
        serial->write_byte(frame[i]);
    }
}

void CN105Climate::send_redlink_frame_(const uint8_t* frame, int length) {
    this->send_frame_(this->redlink_uart_, frame, length);
    this->record_redlink_tx_(esphome::millis());
}

void CN105Climate::set_redlink_connection_state_(bool connected) {
    if (this->redlink_connection_state_ == connected) return;
    this->redlink_connection_state_ = connected;
    if (this->redlink_connection_sensor_ != nullptr) {
        this->redlink_connection_sensor_->publish_state(connected);
    }
}

void CN105Climate::record_redlink_control_source_(const char* source) {
    if (this->redlink_last_control_source_sensor_ != nullptr && source != nullptr) {
        this->redlink_last_control_source_sensor_->publish_state(source);
    }
}

void CN105Climate::update_redlink_diagnostics_(uint32_t now, bool force) {
    if (this->redlink_connection_state_ && this->redlink_last_packet_ms_ != 0 &&
        now - this->redlink_last_packet_ms_ > 120000) {
        this->set_redlink_connection_state_(false);
    }

    if (!force && this->redlink_last_diagnostics_ms_ != 0 &&
        now - this->redlink_last_diagnostics_ms_ < 5000) {
        return;
    }
    this->redlink_last_diagnostics_ms_ = now;

    if (this->redlink_packet_age_sensor_ != nullptr && this->redlink_last_packet_ms_ != 0) {
        this->redlink_packet_age_sensor_->publish_state(
            static_cast<float>(now - this->redlink_last_packet_ms_) / 1000.0f);
    }
    if (this->redlink_rx_packet_count_sensor_ != nullptr) {
        this->redlink_rx_packet_count_sensor_->publish_state(
            static_cast<float>(this->redlink_rx_packet_count_));
    }
    if (this->redlink_tx_packet_count_sensor_ != nullptr) {
        this->redlink_tx_packet_count_sensor_->publish_state(
            static_cast<float>(this->redlink_tx_packet_count_));
    }
    if (this->redlink_timeout_count_sensor_ != nullptr) {
        this->redlink_timeout_count_sensor_->publish_state(
            static_cast<float>(this->redlink_timeout_count_));
    }
}

void CN105Climate::record_redlink_rx_(uint32_t now) {
    this->redlink_last_packet_ms_ = now;
    this->redlink_rx_packet_count_++;
    this->set_redlink_connection_state_(true);
    this->update_redlink_diagnostics_(now, true);
}

void CN105Climate::record_redlink_tx_(uint32_t now) {
    this->redlink_tx_packet_count_++;
    this->update_redlink_diagnostics_(now, true);
}

bool CN105Climate::handle_redlink_state_query_(const uint8_t* frame, int length) {
    if (frame == nullptr || length < 6 || frame[1] != 0x42 ||
        (frame[5] != 0xA9 && frame[5] != 0xAB)) {
        return false;
    }

    // GET_RESPONSE with a 16-byte payload. The thermostat state-download
    // response carries auto mode, heat setpoint, and cool setpoint.
    uint8_t response[22] = {
        0xFC, 0x62, 0x01, 0x30, 0x10,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    response[5] = frame[5];

    if (frame[5] == 0xA9) {
        time_t now = std::time(nullptr);
        if (now < 1704067200) now = 1704067200;  // 2024-01-01 fallback
        const uint32_t timestamp = static_cast<uint32_t>(now);
        response[6] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);
        response[7] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
        response[8] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
        response[9] = static_cast<uint8_t>(timestamp & 0xFF);
        response[11] = (this->mode == climate::CLIMATE_MODE_AUTO ||
                        this->mode == climate::CLIMATE_MODE_HEAT_COOL) ? 0x01 : 0x00;

        float heat = NAN;
        float cool = NAN;
        if (this->traits_.has_feature_flags(climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE)) {
            heat = this->getTargetTemperatureLow();
            cool = this->getTargetTemperatureHigh();
        } else if (this->mode == climate::CLIMATE_MODE_COOL) {
            cool = this->getTargetTemperature();
        } else {
            heat = this->getTargetTemperature();
        }
        if (std::isfinite(heat)) response[12] = cn105_protocol::encode_temperature_b(heat);
        if (std::isfinite(cool)) response[13] = cn105_protocol::encode_temperature_b(cool);

        // Matches the enhanced thermostat response format used by MHK2.
        response[15] = 0x07;
    } else {
        // Thermostat AB query response: command 0xAB, payload byte 1 = 1.
        response[6] = 0x01;
    }

    response[21] = cn105_protocol::checksum(response, 21);
    this->send_redlink_frame_(response, sizeof(response));
    ESP_LOGD(LOG_CONN_TAG, "Answered RedLINK state query 0x%02X locally", frame[5]);
    return true;
}

void CN105Climate::queue_redlink_frame_(const uint8_t* frame, int length) {
    if (length <= 0 || length > MAX_DATA_BYTES) return;
    if (this->has_pending_redlink_frame_) {
        ESP_LOGW(LOG_CONN_TAG, "Dropping RedLINK frame: bridge queue is full");
        return;
    }
    memcpy(this->pending_redlink_frame_, frame, static_cast<size_t>(length));
    this->pending_redlink_frame_len_ = length;
    this->has_pending_redlink_frame_ = true;
}

void CN105Climate::queue_local_redlink_frame_(const uint8_t* frame, int length) {
    if (frame == nullptr || length <= 0 || length > MAX_DATA_BYTES) return;
    if (this->has_pending_local_redlink_frame_) {
        ESP_LOGW(LOG_CONN_TAG, "Dropping local RedLINK control: bridge queue is full");
        return;
    }
    memcpy(this->pending_local_redlink_frame_, frame, static_cast<size_t>(length));
    this->pending_local_redlink_frame_len_ = length;
    this->has_pending_local_redlink_frame_ = true;
}

void CN105Climate::mirror_local_control_to_redlink_(const uint8_t* frame, int length) {
    if (this->redlink_uart_ == nullptr || frame == nullptr || length < 6 ||
        frame[1] != 0x41 ||
        (frame[5] != 0x01 && frame[5] != 0x08 && frame[5] != 0x15)) {
        return;
    }

    // The same valid CN105 SET packet is understood by the RedLINK receiver
    // and keeps the MHK2's displayed settings aligned with ESPHome. Do not
    // mirror 0x07: that packet is the thermostat-to-heat-pump room sensor
    // update, not a user HVAC setting.
    if (this->redlink_local_transaction_active_ || this->redlink_transaction_active_ ||
        this->has_pending_redlink_frame_ || this->redlink_parser_.in_frame() ||
        this->local_transaction_active_) {
        this->queue_local_redlink_frame_(frame, length);
        return;
    }

    this->send_redlink_frame_(frame, length);
    this->redlink_local_transaction_active_ = true;
    this->redlink_local_transaction_started_ms_ = esphome::millis();
    ESP_LOGD(LOG_CONN_TAG, "Mirrored ESPHome control packet 0x%02X to RedLINK", frame[5]);
}

void CN105Climate::flush_local_redlink_frame_() {
    if (!this->has_pending_local_redlink_frame_ || this->redlink_uart_ == nullptr ||
        this->redlink_local_transaction_active_ || this->redlink_transaction_active_ ||
        this->has_pending_redlink_frame_ || this->local_transaction_active_ ||
        this->redlink_parser_.in_frame()) {
        return;
    }

    this->send_redlink_frame_(this->pending_local_redlink_frame_, this->pending_local_redlink_frame_len_);
    this->redlink_local_transaction_active_ = true;
    this->redlink_local_transaction_started_ms_ = esphome::millis();
    this->has_pending_local_redlink_frame_ = false;
    ESP_LOGD(LOG_CONN_TAG, "Mirrored queued ESPHome control to RedLINK");
}

void CN105Climate::capture_redlink_thermostat_temperature_(const uint8_t* frame, int length) {
    // MHK2 reports its selected room sensor as the standard CN105 remote
    // temperature SET command: 0x41 / payload 0x07. The enhanced byte is
    // preferred; older receivers may provide only the legacy byte.
    if (!this->use_redlink_thermostat_temperature_ || frame == nullptr || length < 10 ||
        frame[1] != 0x41 || frame[5] != 0x07) {
        return;
    }

    if (frame[6] == 0x00 || frame[8] == 0x80) {
        this->set_redlink_thermostat_source_active_(false);
        this->redlink_thermostat_temperature_ = NAN;
        this->redlink_thermostat_temperature_ms_ = 0;
        if (!std::isnan(this->currentStatus.roomTemperature)) {
            this->setCurrentTemperature(this->currentStatus.roomTemperature);
            this->publish_state();
        }
        ESP_LOGI(LOG_CONN_TAG, "MHK2 selected internal heat-pump temperature");
        return;
    }

    float temperature = NAN;
    if (frame[8] != 0x00) {
        temperature = (static_cast<float>(frame[8]) - 128.0f) / 2.0f;
    } else {
        temperature = (static_cast<float>(frame[7]) + 16.0f) / 2.0f;
    }

    if (std::isfinite(temperature) && temperature >= -64.0f && temperature <= 63.5f) {
        this->set_redlink_thermostat_source_active_(true);
        this->redlink_thermostat_temperature_ = temperature;
        this->redlink_thermostat_temperature_ms_ = esphome::millis();
        this->setCurrentTemperature(temperature);
        this->publish_state();
        ESP_LOGD(LOG_CONN_TAG, "MHK2 thermostat temperature: %.1f C", temperature);
    }
}

void CN105Climate::set_redlink_thermostat_source_active_(bool active) {
    if (this->redlink_thermostat_source_active_ == active) return;
    this->redlink_thermostat_source_active_ = active;
    if (this->redlink_thermostat_temperature_source_sensor_ != nullptr) {
        this->redlink_thermostat_temperature_source_sensor_->publish_state(active);
    }
}

void CN105Climate::capture_redlink_thermostat_humidity_(const uint8_t* frame, int length) {
    // MHK2 enhanced thermostat sensor status: 0x41 / payload command 0xA6.
    // The ITP packet definition places indoor RH at payload byte 5, which is
    // byte 10 in the complete CN105 frame.
    if (this->redlink_thermostat_humidity_sensor_ == nullptr || frame == nullptr || length < 22 ||
        frame[1] != 0x41 || frame[4] < 16 || frame[5] != 0xA6) {
        return;
    }

    const uint8_t humidity = frame[10];
    if (humidity <= 100) {
        this->redlink_thermostat_humidity_sensor_->publish_state(static_cast<float>(humidity));
        ESP_LOGD(LOG_CONN_TAG, "MHK2 thermostat humidity: %u%%", humidity);
    } else {
        ESP_LOGW(LOG_CONN_TAG, "Ignoring invalid MHK2 humidity: %u%%", humidity);
    }
}

void CN105Climate::capture_redlink_thermostat_status_(const uint8_t* frame, int length) {
    if (frame == nullptr || length < 22 || frame[1] != 0x41 ||
        frame[4] < 16 || frame[5] != 0xA6 || this->redlink_thermostat_battery_sensor_ == nullptr) {
        return;
    }

    const char* battery_state = nullptr;
    switch (frame[11]) {
        case 0x00: battery_state = "OK"; break;
        case 0x01: battery_state = "Low"; break;
        case 0x02: battery_state = "Critical"; break;
        case 0x03: battery_state = "Replace"; break;
        default: battery_state = "Unknown"; break;
    }
    this->redlink_thermostat_battery_sensor_->publish_state(battery_state);
}

void CN105Climate::capture_redlink_thermostat_hello_(const uint8_t* frame, int length) {
    if (frame == nullptr || length < 22 || frame[1] != 0x41 ||
        frame[4] < 16 || frame[5] != 0xA7) {
        return;
    }

    if (this->redlink_thermostat_model_sensor_ != nullptr) {
        this->redlink_thermostat_model_sensor_->publish_state(
            decode_redlink_6bit_string_(&frame[6], 4, 4));
    }
    if (this->redlink_thermostat_serial_sensor_ != nullptr) {
        this->redlink_thermostat_serial_sensor_->publish_state(
            decode_redlink_6bit_string_(&frame[9], 12, 12));
    }
    if (this->redlink_thermostat_firmware_sensor_ != nullptr) {
        char version[16];
        snprintf(version, sizeof(version), "%02u.%02u.%02u", frame[18], frame[19], frame[20]);
        this->redlink_thermostat_firmware_sensor_->publish_state(version);
    }
}

void CN105Climate::process_redlink_thermostat_state_upload_(const uint8_t* frame, int length) {
    if (frame == nullptr || length < 22 || frame[1] != 0x41 ||
        frame[4] < 16 || frame[5] != 0xA8) {
        return;
    }

    const uint8_t flags = frame[6];
    bool climate_changed = false;

    if ((flags & 0x04) != 0 && frame[12] != 0) {
        const climate::ClimateMode new_mode =
            this->traits_.has_feature_flags(climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE)
                ? climate::CLIMATE_MODE_HEAT_COOL
                : climate::CLIMATE_MODE_AUTO;
        if (this->mode != new_mode) {
            this->mode = new_mode;
            climate_changed = true;
        }
    }

    const bool has_heat = (flags & 0x08) != 0 && frame[13] != 0x00 && frame[13] != 0xFF;
    const bool has_cool = (flags & 0x10) != 0 && frame[14] != 0x00 && frame[14] != 0xFF;
    const float heat = has_heat ? (static_cast<float>(frame[13]) - 128.0f) / 2.0f : NAN;
    const float cool = has_cool ? (static_cast<float>(frame[14]) - 128.0f) / 2.0f : NAN;

    if (this->traits_.has_feature_flags(climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE)) {
        if (has_heat) {
            this->setTargetTemperatureLow(heat);
            climate_changed = true;
        }
        if (has_cool) {
            this->setTargetTemperatureHigh(cool);
            climate_changed = true;
        }
    } else if (has_heat || has_cool) {
        const float target = this->mode == climate::CLIMATE_MODE_COOL && has_cool ? cool :
            (has_heat ? heat : cool);
        this->setTargetTemperature(target);
        climate_changed = true;
    }

    if (climate_changed) {
        this->publish_state();
    }
}

bool CN105Climate::redlink_thermostat_temperature_is_fresh_() const {
    return this->use_redlink_thermostat_temperature_ &&
        !std::isnan(this->redlink_thermostat_temperature_) &&
        this->redlink_thermostat_temperature_ms_ != 0 &&
        (esphome::millis() - this->redlink_thermostat_temperature_ms_ < 120000);
}

float CN105Climate::preferred_current_temperature_() const {
    return this->redlink_thermostat_temperature_is_fresh_()
        ? this->redlink_thermostat_temperature_
        : this->currentStatus.roomTemperature;
}

void CN105Climate::flush_redlink_frame_() {
    if (!this->has_pending_redlink_frame_ || this->redlink_uart_ == nullptr ||
        this->redlink_transaction_active_ || this->redlink_local_transaction_active_ ||
        this->local_transaction_active_ ||
        this->redlink_parser_.in_frame() || !this->isUARTReady_()) {
        return;
    }

    this->send_frame_(this->parent_, this->pending_redlink_frame_, this->pending_redlink_frame_len_);
    this->lastSend = esphome::millis();
    this->redlink_transaction_active_ = true;
    this->redlink_transaction_started_ms_ = esphome::millis();
    this->has_pending_redlink_frame_ = false;
    ESP_LOGD(LOG_CONN_TAG, "Forwarded queued RedLINK frame to heat pump");
}

void CN105Climate::service_redlink_bridge_() {
    if (this->redlink_uart_ == nullptr) return;

    const uint32_t now = esphome::millis();

    // At 2400 baud a normal CN105 frame is short, but a disconnected/corrupt
    // wire must not leave the arbitrator locked forever.
    if (this->redlink_parser_.in_frame() &&
        now - this->redlink_last_byte_ms_ > 300) {
        ESP_LOGW(LOG_CONN_TAG, "Resetting incomplete RedLINK frame after timeout");
        this->redlink_parser_.reset();
        this->redlink_timeout_count_++;
        this->update_redlink_diagnostics_(now, true);
    }
    if (this->redlink_transaction_active_ &&
        now - this->redlink_transaction_started_ms_ > 2500) {
        ESP_LOGW(LOG_CONN_TAG, "RedLINK request timed out; releasing CN105 bus");
        this->redlink_transaction_active_ = false;
        this->redlink_timeout_count_++;
        this->set_redlink_connection_state_(false);
        this->update_redlink_diagnostics_(now, true);
    }
    if (this->redlink_local_transaction_active_ &&
        now - this->redlink_local_transaction_started_ms_ > 2500) {
        ESP_LOGW(LOG_CONN_TAG, "Mirrored RedLINK control timed out; releasing bridge bus");
        this->redlink_local_transaction_active_ = false;
        this->redlink_timeout_count_++;
        this->set_redlink_connection_state_(false);
        this->update_redlink_diagnostics_(now, true);
    }
    if (this->local_transaction_active_ &&
        now - this->local_transaction_started_ms_ > 2500) {
        ESP_LOGW(LOG_CONN_TAG, "Local CN105 request timed out; releasing bridge bus");
        this->local_transaction_active_ = false;
    }
    if (this->redlink_thermostat_source_active_ &&
        !this->redlink_thermostat_temperature_is_fresh_()) {
        this->set_redlink_thermostat_source_active_(false);
        this->redlink_thermostat_temperature_ = NAN;
        if (!std::isnan(this->currentStatus.roomTemperature)) {
            this->setCurrentTemperature(this->currentStatus.roomTemperature);
            this->publish_state();
        }
    }

    while (this->redlink_uart_->available()) {
        uint8_t byte = 0;
        if (!this->redlink_uart_->read_byte(&byte)) continue;
        this->redlink_last_byte_ms_ = esphome::millis();
        this->redlink_parser_.feed(byte);

        if (!this->redlink_parser_.frame_complete()) continue;

        const int length = this->redlink_parser_.frame_size();
        uint8_t frame[MAX_DATA_BYTES] = {};
        memcpy(frame, this->redlink_parser_.raw(), static_cast<size_t>(length));
        const bool valid = this->redlink_parser_.checksum_valid();
        this->redlink_parser_.reset();

        if (!valid) {
            ESP_LOGW(LOG_CONN_TAG, "Dropping invalid frame from RedLINK receiver");
            this->redlink_timeout_count_++;
            this->update_redlink_diagnostics_(esphome::millis(), true);
            continue;
        }

        this->record_redlink_rx_(esphome::millis());

        // A locally mirrored SET is acknowledged by the receiver on this
        // UART. Consume that acknowledgement here; it must not be routed to
        // the indoor unit as though it belonged to a thermostat request.
        if (this->redlink_local_transaction_active_ &&
            this->is_response_command_(frame[1])) {
            this->redlink_local_transaction_active_ = false;
            ESP_LOGD(LOG_CONN_TAG, "RedLINK acknowledged mirrored ESPHome control");
            continue;
        }

        if (frame[1] == 0x41 &&
            (frame[5] == 0x01 || frame[5] == 0x07 || frame[5] == 0x08 ||
             frame[5] == 0x15 || frame[5] == 0xA8 || frame[5] == 0xAA)) {
            this->record_redlink_control_source_("MHK2");
        }
        this->capture_redlink_thermostat_temperature_(frame, length);
        this->capture_redlink_thermostat_humidity_(frame, length);
        this->capture_redlink_thermostat_status_(frame, length);
        this->capture_redlink_thermostat_hello_(frame, length);
        this->process_redlink_thermostat_state_upload_(frame, length);

        if (this->handle_redlink_state_query_(frame, length)) {
            continue;
        }

        // Enhanced MHK2 packets are thermostat-local. Acknowledge them here;
        // forwarding them to the heat pump can leave the RedLINK side waiting
        // because older CN105 heat pumps do not understand 0xA6-0xAA.
        if (length >= 6 && frame[1] == 0x41 &&
            (frame[5] == 0xA6 || frame[5] == 0xA7 || frame[5] == 0xA8 || frame[5] == 0xAA)) {
            static const uint8_t set_response[] = {
                0xFC, 0x61, 0x01, 0x30, 0x10,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E,
            };
            this->send_redlink_frame_(set_response, sizeof(set_response));
            ESP_LOGD(LOG_CONN_TAG, "Acknowledged enhanced MHK2 thermostat packet 0x%02X", frame[5]);
            continue;
        }

        if (this->redlink_transaction_active_ || this->redlink_local_transaction_active_ ||
            this->local_transaction_active_ ||
            !this->isUARTReady_()) {
            this->queue_redlink_frame_(frame, length);
        } else {
            this->send_frame_(this->parent_, frame, length);
            this->lastSend = esphome::millis();
            this->redlink_transaction_active_ = true;
            this->redlink_transaction_started_ms_ = esphome::millis();
            ESP_LOGD(LOG_CONN_TAG, "Forwarded RedLINK frame to heat pump");
        }
    }

    this->update_redlink_diagnostics_(esphome::millis());
    this->flush_redlink_frame_();
    this->flush_local_redlink_frame_();
}

bool CN105Climate::forward_heatpump_frame_to_redlink_() {
    if (this->redlink_uart_ == nullptr || !this->parser_.checksum_valid()) return false;

    const bool is_peer_response = this->redlink_transaction_active_ &&
        this->is_response_command_(this->parser_.command());
    if (is_peer_response) {
        this->send_redlink_frame_(this->parser_.raw(), this->parser_.frame_size());
        this->redlink_transaction_active_ = false;
        ESP_LOGD(LOG_CONN_TAG, "Forwarded heat-pump response to RedLINK receiver");
        return true;
    }

    if (this->is_response_command_(this->parser_.command())) {
        this->local_transaction_active_ = false;
    }
    return false;
}

void CN105Climate::pingExternalTemperature() {
    this->set_timeout(SHEDULER_REMOTE_TEMP_TIMEOUT, this->remote_temp_timeout_, [this]() {
        ESP_LOGW(LOG_REMOTE_TEMP, "Remote temperature timeout occured, fall back to internal temperature!");
        this->stopRemoteTempKeepAlive();
        this->set_remote_temperature(0);
        });
}

void CN105Climate::set_remote_temp_timeout(uint32_t timeout) {
    this->remote_temp_timeout_ = timeout;
    if (timeout == 4294967295) {
        ESP_LOGI(LOG_REMOTE_TEMP, "set_remote_temp_timeout is set to never.");
    } else {
        //ESP_LOGI(LOG_ACTION_EVT_TAG, "set_remote_temp_timeout is set to %lu", timeout);
        log_info_uint32(LOG_REMOTE_TEMP, "set_remote_temp_timeout is set to ", timeout);

        this->pingExternalTemperature();
    }
}

void CN105Climate::set_remote_temp_keepalive_interval(uint32_t interval_ms) {
    this->remote_temp_keepalive_interval_ms_ = interval_ms;
    if (interval_ms == 0) {
        ESP_LOGI(LOG_REMOTE_TEMP, "Remote temperature keep-alive disabled.");
    } else {
        log_info_uint32(LOG_REMOTE_TEMP, "Remote temperature keep-alive interval set to ", interval_ms);
    }
}

void CN105Climate::set_remote_temperature_control_sensor(esphome::binary_sensor::BinarySensor* sensor) {
    this->remote_temp_sensor_ = sensor;
    ESP_LOGI(LOG_REMOTE_TEMP, "Remote temperature control sensor configured.");
}

void CN105Climate::set_remote_temperature_margin(float margin) {
    this->remote_temp_margin_ = margin;
    ESP_LOGI(LOG_REMOTE_TEMP, "Remote temperature margin set to %.1f", margin);
}

void CN105Climate::startRemoteTempKeepAlive() {
    // Don't start if keep-alive is disabled or already active
    if (this->remote_temp_keepalive_interval_ms_ == 0) {
        ESP_LOGD(LOG_REMOTE_TEMP, "Keep-alive disabled, not starting.");
        return;
    }
    if (this->remote_temp_keepalive_active_) {
        ESP_LOGV(LOG_REMOTE_TEMP, "Keep-alive already active.");
        return;
    }

    this->remote_temp_keepalive_active_ = true;
    log_info_uint32(LOG_REMOTE_TEMP, "Starting remote temperature keep-alive with interval ", this->remote_temp_keepalive_interval_ms_);

    this->set_interval(SCHEDULER_REMOTE_TEMP_KEEPALIVE, this->remote_temp_keepalive_interval_ms_, [this]() {
        if (this->remoteTemperature_ > 0 && this->isHeatpumpConnected()) {
            ESP_LOGD(LOG_REMOTE_TEMP, "Keep-alive: re-sending remote temperature %.1f", this->remoteTemperature_);
            // Send the temperature packet without resetting the watchdog timeout
            // (watchdog is only reset when HA sends a new value via set_remote_temperature)
            this->shouldSendExternalTemperature_ = true;
        } else {
            if (!this->isHeatpumpConnected()) {
                ESP_LOGW(LOG_REMOTE_TEMP, "Keep-alive skipped: Heatpump not connected!");
            } else {
                ESP_LOGD(LOG_REMOTE_TEMP, "Keep-alive skipped: remoteTemp %.1f <= 0", this->remoteTemperature_);
            }
        }
        });
}

void CN105Climate::stopRemoteTempKeepAlive() {
    if (!this->remote_temp_keepalive_active_) {
        return;
    }
    this->remote_temp_keepalive_active_ = false;
    this->cancel_interval(SCHEDULER_REMOTE_TEMP_KEEPALIVE);
    ESP_LOGI(LOG_REMOTE_TEMP, "Stopped remote temperature keep-alive.");
}

void CN105Climate::set_debounce_delay(uint32_t delay) {
    this->debounce_delay_ = delay;
    //ESP_LOGI(LOG_ACTION_EVT_TAG, "set_debounce_delay is set to %lu", delay);
    log_info_uint32(LOG_ACTION_EVT_TAG, "set_debounce_delay is set to ", delay);
}

float CN105Climate::get_compressor_frequency() {
    return currentStatus.compressorFrequency;
}
float CN105Climate::get_input_power() {
    return currentStatus.inputPower;
}
float CN105Climate::get_kwh() {
    return currentStatus.kWh;
}
float CN105Climate::get_runtime_hours() {
    return currentStatus.runtimeHours;
}
bool CN105Climate::is_operating() {
    return currentStatus.operating;
}
bool CN105Climate::is_air_purifier() {
    return currentRunStates.air_purifier;
}
bool CN105Climate::is_night_mode() {
    return currentRunStates.night_mode;
}
bool CN105Climate::is_circulator() {
    return currentRunStates.circulator;
}

// SERIAL_8E1
void CN105Climate::setupUART() {

    log_info_uint32(TAG, "setupUART() with baudrate ", this->parent_->get_baud_rate());
    ESP_LOGI(LOG_CONN_TAG, "setupUART(): baud=%" PRIu32 " tx=%d rx=%d (UART port=%d)", this->parent_->get_baud_rate(), this->tx_pin_, this->rx_pin_, this->uart_port_);
    this->setHeatpumpConnected(false);
    // isUARTConnected_ replaced by state_ (set to CONNECTING after successful config below)

    // just for debugging purpose, a way to use a button i, yaml to trigger a reconnect
    this->uart_setup_switch = true;

    if (this->parent_->get_data_bits() == 8 &&
        this->parent_->get_parity() == uart::UART_CONFIG_PARITY_EVEN &&
        this->parent_->get_stop_bits() == 1) {
        ESP_LOGI(LOG_CONN_TAG, "UART configured as SERIAL_8E1");
        this->transition_to_(DriverState::CONNECTING);
        this->parser_.reset();
    } else {
        ESP_LOGW(LOG_CONN_TAG, "UART is not configured as SERIAL_8E1");
    }

}

void CN105Climate::setHeatpumpConnected(bool state) {
    if (state) {
        this->transition_to_(DriverState::CONNECTED);
    } else if (state_ == DriverState::CONNECTED) {
        this->transition_to_(DriverState::DISCONNECTED);
    }
    if (this->hp_uptime_connection_sensor_ != nullptr) {
        if (state) {
            this->hp_uptime_connection_sensor_->start();
            ESP_LOGD(TAG, "starting hp_uptime_connection_sensor_ uptime chrono");
        } else {
            this->hp_uptime_connection_sensor_->stop();
            ESP_LOGD(TAG, "stopping hp_uptime_connection_sensor_ uptime chrono");
        }
    }
}
void CN105Climate::disconnectUART() {
    ESP_LOGD(TAG, "disconnectUART()");
    this->uart_setup_switch = false;
    this->setHeatpumpConnected(false);
    // Legacy booleans removed — state managed by FSM (setHeatpumpConnected / transition_to_)
    this->firstRun = true;
    this->publish_state();

}

void CN105Climate::reconnectUART() {
    ESP_LOGD(TAG, "reconnectUART()");
    this->lastReconnectTimeMs = esphome::millis();
    this->disconnectUART();
    // Disabled: Low-level UART fallback (ESP-IDF 5.4.x) can interfere with the
    // handshake/fallback tests. We let UARTComponent generate the standard reset.
    this->force_low_level_uart_reinit();
    this->setupUART();
    this->sendFirstConnectionPacket();
}


void CN105Climate::reconnectIfConnectionLost() {

    long reconnectTimeMs = esphome::millis() - this->lastReconnectTimeMs;

    if (reconnectTimeMs < this->update_interval_) {
        return;
    }

    if (!this->isHeatpumpConnectionActive()) {
        long connectTimeMs = esphome::millis() - this->lastConnectRqTimeMs;
        if (connectTimeMs > this->update_interval_) {
            long lrTimeMs = esphome::millis() - this->lastResponseMs;
            ESP_LOGW(TAG, "Heatpump has not replied for %ld s", lrTimeMs / 1000);
            ESP_LOGI(TAG, "We think Heatpump is not connected anymore..");
            this->reconnectUART();
        }
    }
}


bool CN105Climate::isHeatpumpConnectionActive() {
    long lrTimeMs = esphome::millis() - this->lastResponseMs;

    // if (lrTimeMs > MAX_DELAY_RESPONSE_FACTOR * this->update_interval_) {
    //     ESP_LOGV(TAG, "Heatpump has not replied for %ld s", lrTimeMs / 1000);
    //     ESP_LOGV(TAG, "We think Heatpump is not connected anymore..");
    //     this->disconnectUART();
    // }

    return  (lrTimeMs < MAX_DELAY_RESPONSE_FACTOR * this->update_interval_);
}

void CN105Climate::force_low_level_uart_reinit() {
#ifdef USE_ESP32
    // Low layer reset: reconfigure user control by UARTComponent
    // We use the port passed by set_uart_port (fallback UART0 if unknown)
    const uart_port_t port = (this->uart_port_ == 1) ? UART_NUM_1 :
#ifdef UART_NUM_2
    (this->uart_port_ == 2) ? UART_NUM_2 :
#endif
        UART_NUM_0;

    ESP_LOGI(TAG, "Forcing low-level UART reinit on port %d (tx=%d, rx=%d)", (int)port, this->tx_pin_, this->rx_pin_);

    // IMPORTANT: do not delete/reinstall the driver here to avoid conflict with UARTComponent
    // We reconfigure in-place and sanitize the GPIOs
    if (this->tx_pin_ >= 0) gpio_reset_pin((gpio_num_t)this->tx_pin_);
    if (this->rx_pin_ >= 0) gpio_reset_pin((gpio_num_t)this->rx_pin_);
    esphome::delay(2);

    // Settings SERIAL_8E1 @ 2400 bauds (values ​​from the UARTComponent config)
    uart_config_t cfg = {};
    cfg.baud_rate = this->parent_ ? (int)this->parent_->get_baud_rate() : 2400;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_EVEN;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.rx_flow_ctrl_thresh = 0;

    uart_param_config(port, &cfg);

    // Reconfigure the pins if known; otherwise GPIO1/2 (Atom S3 yaml)
    int tx = (this->tx_pin_ >= 0) ? this->tx_pin_ : 1;
    int rx = (this->rx_pin_ >= 0) ? this->rx_pin_ : 2;
    esp_err_t pin_err = uart_set_pin(port, (gpio_num_t)tx, (gpio_num_t)rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (pin_err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(pin_err));
    }

    // RX idle high: ensure a pull-up (useful at low bitrates/8E1)
    if (this->rx_pin_ >= 0) {
        gpio_set_pull_mode((gpio_num_t)this->rx_pin_, GPIO_PULLUP_ONLY);
    }

    // Ensure classic UART mode
    uart_set_mode(port, UART_MODE_UART);

    // Wait for any TX in progress to finish (if driver already installed)
    uart_wait_tx_done(port, pdMS_TO_TICKS(20));

    // Fix UART clock source (lower bits may be sensitive)
#if defined(UART_SCLK_XTAL)
    uart_set_sclk(port, UART_SCLK_XTAL);
#elif defined(UART_SCLK_APB)
    uart_set_sclk(port, UART_SCLK_APB);
#endif
    // Explicitly re-force baud after sclk
    uart_set_baudrate(port, cfg.baud_rate);

    // Disable inversion/flow control
    uart_set_line_inverse(port, UART_SIGNAL_INV_DISABLE);
    uart_set_hw_flow_ctrl(port, UART_HW_FLOWCTRL_DISABLE, 0);

    // Short RX timeout for quick emptying
    uart_set_rx_timeout(port, 2);

    // Purge buffers to avoid residue
    uart_flush_input(port);
    esphome::delay(2);

    // Diagnostics
    uint32_t eff_baud = 0;
    uart_get_baudrate(port, &eff_baud);
    ESP_LOGD(TAG, "UART effective baud=%lu tx_pin=%d rx_pin=%d", (unsigned long)eff_baud, this->tx_pin_, this->rx_pin_);
#else
    // No ESP32: nothing to do
#endif
}
