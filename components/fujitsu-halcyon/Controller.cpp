#include "Controller.h"

#include <algorithm>
#include <cmath>
#include "esp_timer.h"

//#include <esp_log.h>
// Log through esphome instead of standard esp logging
#include <esphome/core/log.h>
using esphome::esp_log_printf_;

namespace fujitsu_general::airstage::h {

static const char* TAG = "fujitsu_general::airstage::h::Controller";

void Controller::process_uart_data() {
    // Check received packets
    auto buffer_len = this->uart_available_bytes();
    if (buffer_len >= Packet::FrameSize) {
        Packet::Buffer buffer;

        // Discard partial frame
        if (auto discard = buffer_len % buffer.size()) {
            this->uart_read_bytes(buffer.data(), discard);
            ESP_LOGW(TAG, "Discarded %d bytes", discard);
        }

        // For each frame
        while (buffer_len) {
            this->uart_read_bytes(buffer.data(), buffer.size());
            buffer_len = this->uart_available_bytes();
            this->process_packet(buffer);
        }
    }
    // See if we've got any packets to transmit
    if (this->lastPacketToMe) {
        uint32_t now = esp_timer_get_time() / 1000ULL;
        if ((now - this->rx_time) >= this->tx_delay_) {
            this->transmit_packet();
        }
    }
    // Have now (hopefully) transmitted on time so call pending callback                                                
    if (this->deferred_callback) {                                                                  
        this->deferred_callback();                                                                                                                      
    }     

}

size_t Controller::uart_available_bytes() {
    return this->callbacks.AvailableBytes ? callbacks.AvailableBytes() : 0;
}

void Controller::uart_read_bytes(uint8_t *buf, size_t length) {
    if (this->callbacks.ReadBytes)
        callbacks.ReadBytes(buf, length);
}

void Controller::uart_write_bytes(const uint8_t *buf, size_t length) {
    // Check if transmitting
    if (!this->transmit_) {
        ESP_LOGW(TAG, "TX disabled, Packet skipped");
        return;
    }
    if (this->callbacks.WriteBytes)
        callbacks.WriteBytes(buf, length);
}

void Controller::set_initialization_stage(const InitializationStageEnum stage) {
    // See if we need to skip 
    if (this->skip_find_next_controller && stage == InitializationStageEnum::FindNextControllerTx) {
        this->initialization_stage = InitializationStageEnum::FindNextControllerRx;
        ESP_LOGW(TAG, "skipping find next controller, changing to state %d", this->initialization_stage);
    } else {
        this->initialization_stage = stage;
    }
    if (this->callbacks.InitializationStage)
        callbacks.InitializationStage(this->initialization_stage);
}

void Controller::update_rx_stats(PacketTypeEnum packetType) {
    // increment packet stats
    this->rx_time = esp_timer_get_time() / 1000ULL;
    this->rx_total++;
    this->lastPacketType = packetType; // Save last packet type
    int idx = static_cast<int>(packetType);
    if (idx < sizeof(this->rx_stats)) this->rx_stats[idx]++;
    else {
        ESP_LOGE(TAG, "invalid packet type received: %d", idx);
        this->rx_invalid_type++;
    }
    if (this->callbacks.RxPktStats)
        callbacks.RxPktStats(this->rx_total, this->rx_invalid_type, this->rx_stats);
}

void Controller::update_to_me_stats(PacketTypeEnum packetType) {
    // increment packet stats
    this->to_me_total++;
    int idx = static_cast<int>(packetType);
    if (idx < sizeof(this->to_me_stats)) this->to_me_stats[idx]++;
    else {
        ESP_LOGE(TAG, "invalid packet type received: %d", idx);
        this->rx_invalid_type++;
    }
    if (this->callbacks.ToMePktStats)
        callbacks.ToMePktStats(this->to_me_total, this->to_me_invalid_type, this->to_me_stats);
}

void Controller::update_tx_stats(PacketTypeEnum packetType) {
    // increment packet stats
    this->tx_total++;
    int idx = static_cast<int>(packetType);
    if (idx < sizeof(this->tx_stats)) this->tx_stats[idx]++;
    else {
        ESP_LOGE(TAG, "invalid packet type sent: %d", idx);
        this->tx_invalid_type++;
    }
    // publish stats
    if (this->callbacks.TxPktStats)
        callbacks.TxPktStats(this->tx_total, this->tx_invalid_type, this->tx_stats, this->last_tx_delay);
}

void Controller::process_packet(const Packet::Buffer& buffer) {

    // Parse buffer
    Packet packet(buffer);

    // increment packet stats
    this->update_rx_stats(packet.Type);

    // Was this a packet for us ?
    if (packet.TokenDestinationType == AddressTypeEnum::Controller && packet.TokenDestinationAddress == this->controller_address) {
        this->lastPacketToMe = true;
        this->update_to_me_stats(packet.Type);
    }

    // Check for poll to us to complete initialisation
    if (this->initialization_stage == InitializationStageEnum::WaitForPoll
        && this->lastPacketToMe && packet.Type == PacketTypeEnum::Config) {
        // then transition to complete
        this->set_initialization_stage(InitializationStageEnum::Complete);
    }

    // Check for next controller
    if (this->initialization_stage == InitializationStageEnum::FindNextControllerRx) {
        // Controller with address > configured did not transmit
        if (packet.SourceType != AddressTypeEnum::Controller)
            this->next_token_destination_type = AddressTypeEnum::IndoorUnit;

        // Fujitsu RC1 checks for next controller twice (in case of slow booting controller?), but we are only checking once
        this->set_initialization_stage(InitializationStageEnum::WaitForPoll);
    }

    // Process packets from Indoor Units
    if (packet.SourceType == AddressTypeEnum::IndoorUnit) {
        switch (packet.Type) {
            [[likely]] case PacketTypeEnum::Config:
                if (this->initialization_stage == InitializationStageEnum::DetectFeatureSupport) {
                    // Advance to FindNextControllerTx (skip feature negotiation entirely) if:
                    //  - autoconf is disabled (use the configured features directly), or
                    //  - the IU's UnknownFlags == 2 (no feature negotiation support).
                    // Otherwise, transition to FeatureRequestTx to send a FeatureRequest packet
                    // when our turn with the token comes around. The actual transmission and
                    // the subsequent transition to FeatureRequestRx happen later in this function.
                    // Note: this->features is already initialized to DefaultFeatures (or to a
                    // user-supplied override via set_features()), so no assignment is needed here.
                    if (!this->autoconf ||
                        packet.Config.IndoorUnit.UnknownFlags == 2) {
                        this->set_initialization_stage(InitializationStageEnum::FindNextControllerTx);
                    } else
                        this->set_initialization_stage(InitializationStageEnum::FeatureRequestTx);
                }
                else if (this->initialization_stage == InitializationStageEnum::FeatureRequestRx) {
                    // We already transmitted a FeatureRequest and the IU replied with another
                    // Config instead of a Features packet -> the IU does not support feature
                    // negotiation. Fall back to the in-code (or user-supplied) defaults already
                    // present in this->features and proceed.
                    this->set_initialization_stage(InitializationStageEnum::FindNextControllerTx);
                }

                if (this->last_error_flag != packet.Config.IndoorUnit.Error)
                    this->error_flag_changed = true;

                this->last_error_flag = packet.Config.IndoorUnit.Error;
                this->current_configuration = packet.Config;

                // Include the state of these fields not returned from the Indoor Unit in the callback data
                this->current_configuration.Controller.Temperature = this->changed_configuration.Controller.Temperature;
                this->current_configuration.Controller.UseControllerSensor = this->changed_configuration.Controller.UseControllerSensor;

                if (this->callbacks.Config)
                    this->deferred_callback = [this](){ this->callbacks.Config(this->current_configuration); };
                break;

            case PacketTypeEnum::Error:
                if (this->callbacks.Error)
                    this->deferred_callback = [&](){ this->callbacks.Error(packet); };
                break;

            case PacketTypeEnum::Features:
                this->features = packet.Features;
                this->set_initialization_stage(InitializationStageEnum::FindNextControllerTx);
                break;

            case PacketTypeEnum::Function:
                if (this->callbacks.Function)
                    this->deferred_callback = [&](){ this->callbacks.Function(packet.Function); };
                break;
            case PacketTypeEnum::Status:
                break;
        }
    } else {
        switch (packet.Type) {
            // Config packet from another controller
            [[likely]] case PacketTypeEnum::Config:
                this->rx_config_from_ctlr++;
                if (this->callbacks.ControllerConfig)
                    this->deferred_callback = [&](){ this->callbacks.ControllerConfig(packet.SourceAddress, packet.Config); };
                break;
            default:
                break;
        }
    }

}

void Controller::transmit_packet() {

    Packet tx_packet;
    tx_packet.SourceType = AddressTypeEnum::Controller;
    tx_packet.SourceAddress = this->controller_address;

    if (this->initialization_stage == InitializationStageEnum::FindNextControllerTx) {
        this->next_token_destination_type = AddressTypeEnum::Controller;
        this->set_initialization_stage(InitializationStageEnum::FindNextControllerRx);
    }

    tx_packet.TokenDestinationType = this->next_token_destination_type;
    tx_packet.TokenDestinationAddress = this->next_token_destination_type == AddressTypeEnum::Controller ? this->controller_address + 1 : 1;

    if (this->initialization_stage == InitializationStageEnum::FeatureRequestTx) {
        tx_packet.Type = PacketTypeEnum::Features;
        // Advance only after the request is actually transmitted, mirroring
        // the FindNextControllerTx -> FindNextControllerRx transition above.
        this->set_initialization_stage(InitializationStageEnum::FeatureRequestRx);
    }
    else if ((error_flag_changed && this->is_primary_controller()) ||
             (this->lastPacketType == PacketTypeEnum::Error && !this->is_primary_controller()))
        tx_packet.Type = PacketTypeEnum::Error;
    else if (!this->function_queue.empty()) {
        tx_packet.Type = PacketTypeEnum::Function;
        tx_packet.Function = this->function_queue.front();
        this->function_queue.pop();
    }
    else {
        // First CONFIG packet sent from Fujitsu controller has write flag set, but we do not restore state at this time
        tx_packet.Type = PacketTypeEnum::Config;
        tx_packet.Config = this->current_configuration;
        tx_packet.Config.Controller.Temperature = this->changed_configuration.Controller.Temperature;
        tx_packet.Config.Controller.UseControllerSensor = this->changed_configuration.Controller.UseControllerSensor;

        if (this->configuration_changes.any()) {
            tx_packet.Config.Controller.Write = true;

            // Overwrite fields received from Indoor Unit
            if (this->configuration_changes[SettableFields::Enabled])
                tx_packet.Config.Enabled = this->changed_configuration.Enabled;

            if (this->configuration_changes[SettableFields::Economy])
                tx_packet.Config.Economy = this->changed_configuration.Economy;

            if (this->configuration_changes[SettableFields::Setpoint])
                tx_packet.Config.Setpoint = this->changed_configuration.Setpoint;

            if (this->configuration_changes[SettableFields::TestRun])
                tx_packet.Config.TestRun = this->changed_configuration.TestRun;

            if (this->configuration_changes[SettableFields::Mode])
                tx_packet.Config.Mode = this->changed_configuration.Mode;

            if (this->configuration_changes[SettableFields::FanSpeed])
                tx_packet.Config.FanSpeed = this->changed_configuration.FanSpeed;

            if (this->configuration_changes[SettableFields::SwingVertical])
                tx_packet.Config.SwingVertical = this->changed_configuration.SwingVertical;

            if (this->configuration_changes[SettableFields::SwingHorizontal])
                tx_packet.Config.SwingHorizontal = this->changed_configuration.SwingHorizontal;

            // Set fields not returned from Indoor Unit
            if (this->configuration_changes[SettableFields::AdvanceVerticalLouver])
                tx_packet.Config.Controller.AdvanceVerticalLouver = this->changed_configuration.Controller.AdvanceVerticalLouver;

            if (this->configuration_changes[SettableFields::AdvanceHorizontalLouver])
                tx_packet.Config.Controller.AdvanceHorizontalLouver = this->changed_configuration.Controller.AdvanceHorizontalLouver;

            if (this->configuration_changes[SettableFields::ResetFilterTimer])
                tx_packet.Config.Controller.ResetFilterTimer = this->changed_configuration.Controller.ResetFilterTimer;

            if (this->configuration_changes[SettableFields::Maintenance])
                tx_packet.Config.Controller.Maintenance = this->changed_configuration.Controller.Maintenance;
        }

        this->configuration_changes.reset();

        // Some fields need to be written clear in next tx packet
        if (tx_packet.Config.Controller.ResetFilterTimer) {
            this->changed_configuration.Controller.ResetFilterTimer = false;
            this->configuration_changes[SettableFields::ResetFilterTimer] = true;
        }

        if (tx_packet.Config.Controller.Maintenance) {
            this->changed_configuration.Controller.Maintenance = false;
            this->configuration_changes[SettableFields::Maintenance] = true;
        }
    }

    // Calc the tx delay
    this->tx_time = esp_timer_get_time() / 1000ULL;
    this->last_tx_delay = (this->tx_time - this->rx_time);

    // Update stats
    this->update_tx_stats(tx_packet.Type);
    if (tx_packet.TokenDestinationType == AddressTypeEnum::Controller) this->tx_to_ctlr++;

    // Transmit packet
    Packet::Buffer b = tx_packet.to_buffer();
    this->uart_write_bytes(b.data(), b.size());

    // We've processed this packet
    this->lastPacketToMe = false;
}

void Controller::set_current_temperature(float temperature) {
    this->changed_configuration.Controller.Temperature = std::clamp(std::isfinite(temperature) ? temperature : 0, MinTemperature, MaxTemperature);
    // Do not set configuration_changed flag - does not require write bit set
}

bool Controller::set_enabled(bool enabled, bool ignore_lock) {
    if (!ignore_lock && (this->current_configuration.IndoorUnit.Lock.All || this->current_configuration.IndoorUnit.Lock.Enabled))
        return false;

    this->changed_configuration.Enabled = enabled;
    this->configuration_changes[SettableFields::Enabled] = true;
    return true;
}

bool Controller::set_economy(bool economy, bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    this->changed_configuration.Economy = economy;
    this->configuration_changes[SettableFields::Economy] = true;
    return true;
}

bool Controller::set_test_run(bool test_run, bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    this->changed_configuration.TestRun = test_run;
    this->configuration_changes[SettableFields::TestRun] = true;
    return true;
}

bool Controller::set_setpoint(uint8_t temperature, bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    if (temperature < MinSetpoint || temperature > MaxSetpoint)
        return false;

    this->changed_configuration.Setpoint = temperature;
    this->configuration_changes[SettableFields::Setpoint] = true;
    return true;
}

bool Controller::set_mode(ModeEnum mode, bool ignore_lock) {
    if (!ignore_lock && (this->current_configuration.IndoorUnit.Lock.All || this->current_configuration.IndoorUnit.Lock.Mode))
        return false;

    switch (mode) {
        case ModeEnum::Fan:
            if (!this->features.Mode.Fan)
                return false;
            break;

        case ModeEnum::Dry:
            if (!this->features.Mode.Dry)
                return false;
            break;

        case ModeEnum::Cool:
            if (!this->features.Mode.Cool)
                return false;
            break;

        case ModeEnum::Heat:
            if (!this->features.Mode.Heat)
                return false;
            break;

        case ModeEnum::Auto:
            if (!this->features.Mode.Auto)
                return false;
            break;
    }

    this->changed_configuration.Mode = mode;
    this->configuration_changes[SettableFields::Mode] = true;
    return true;
}

bool Controller::set_fan_speed(FanSpeedEnum fan_speed, bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    switch (fan_speed) {
        case FanSpeedEnum::Auto:
            if (!this->features.FanSpeed.Auto)
                return false;
            break;

        case FanSpeedEnum::Quiet:
            if (!this->features.FanSpeed.Quiet)
                return false;
            break;

        case FanSpeedEnum::Low:
            if (!this->features.FanSpeed.Low)
                return false;
            break;

        case FanSpeedEnum::Medium:
            if (!this->features.FanSpeed.Medium)
                return false;
            break;

        case FanSpeedEnum::High:
            if (!this->features.FanSpeed.High)
                return false;
            break;
    }

    this->changed_configuration.FanSpeed = fan_speed;
    this->configuration_changes[SettableFields::FanSpeed] = true;
    return true;
}

bool Controller::set_vertical_swing(bool swing_vertical, bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    if (!this->features.VerticalLouvers)
        return false;

    this->changed_configuration.SwingVertical = swing_vertical;
    this->configuration_changes[SettableFields::SwingVertical] = true;
    return true;
}

bool Controller::set_horizontal_swing(bool swing_horizontal, bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    if (!this->features.HorizontalLouvers)
        return false;

    this->changed_configuration.SwingHorizontal = swing_horizontal;
    this->configuration_changes[SettableFields::SwingHorizontal] = true;
    return true;
}

bool Controller::advance_vertical_louver(bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    if (!this->features.VerticalLouvers)
        return false;

    this->changed_configuration.Controller.AdvanceVerticalLouver = true;
    this->configuration_changes[SettableFields::AdvanceVerticalLouver] = true;
    return true;
}

bool Controller::advance_horizontal_louver(bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    if (!this->features.HorizontalLouvers)
        return false;

    this->changed_configuration.Controller.AdvanceHorizontalLouver = true;
    this->configuration_changes[SettableFields::AdvanceHorizontalLouver] = true;
    return true;
}

bool Controller::use_sensor(bool use_sensor, bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    if (!this->features.SensorSwitching)
        return false;

    this->changed_configuration.Controller.UseControllerSensor = use_sensor;
    // Do not set configuration_changed flag - does not require write bit set
    return true;
}

bool Controller::transmit(bool transmit) {

    this->transmit_ = transmit;
    return true;
}

bool Controller::set_tx_delay(uint8_t tx_delay) {

    this->tx_delay_ = tx_delay;
    return true;
}

bool Controller::set_tx_test(bool tx_test) {

    this->tx_test_ = tx_test;
    return true;
}

bool Controller::reset_filter(bool ignore_lock) {
    if (!ignore_lock && (this->current_configuration.IndoorUnit.Lock.All || this->current_configuration.IndoorUnit.Lock.ResetFilterTimer))
        return false;

    if (!this->features.FilterTimer)
        return false;

    this->changed_configuration.Controller.ResetFilterTimer = true;
    this->configuration_changes[SettableFields::ResetFilterTimer] = true;
    return true;
}

bool Controller::maintenance(bool ignore_lock) {
    if (!ignore_lock && this->current_configuration.IndoorUnit.Lock.All)
        return false;

    if (!this->features.Maintenance)
        return false;

    this->changed_configuration.Controller.Maintenance = true;
    this->configuration_changes[SettableFields::Maintenance] = true;
    return true;
}

}
