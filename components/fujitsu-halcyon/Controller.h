#pragma once

#include <bitset>
#include <functional>
#include <queue>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/uart.h>

#include "Packet.h"

namespace fujitsu_general::airstage::h {

constexpr uart_config_t UARTConfig = {
        .baud_rate = 500,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
};

constexpr uint8_t UARTInterPacketSymbolSpacing = 2;
constexpr uint8_t DefaultTXDelay = 50; // in ms

// Temperatures are in Celcius
// min set point should be 18
// constexpr uint8_t MinSetpoint = 16;
constexpr uint8_t MinSetpoint = 18;
constexpr uint8_t MaxSetpoint = 30;
constexpr float MinTemperature = 0.0;
constexpr float MaxTemperature = 60.0;

constexpr Features DefaultFeatures = {
        .Mode = {
            .Auto = true,
            .Heat = true,
            .Fan = true,
            .Dry = true,
            .Cool = true,
        },

        .FanSpeed = {
            .Quiet = false,
            .Low = true,
            .Medium = true,
            .High = true,
            .Auto = true,
        },

        .FilterTimer = false,
        .SensorSwitching = false,
        .Maintenance = false,
        .EconomyMode = true,
        .HorizontalLouvers = false,
        .VerticalLouvers = false,
};

enum class InitializationStageEnum : uint8_t {
    DetectFeatureSupport,
    FeatureRequest,
    FindNextControllerTx,
    FindNextControllerRx,
    WaitForPoll,
    Complete
};

namespace SettableFields {
    enum {
        Enabled,
        Economy,
        TestRun,
        Setpoint,
        Mode,
        FanSpeed,
        SwingVertical,
        SwingHorizontal,
        AdvanceVerticalLouver,
        AdvanceHorizontalLouver,
        ResetFilterTimer,
        Maintenance,
        MAX
    };
};

class Controller {
    using ConfigCallback = std::function<void(const Config&)>;
    using ErrorCallback  = std::function<void(const Packet&)>;
    using FunctionCallback = std::function<void(const Function&)>;
    using ControllerConfigCallback = std::function<void(const uint8_t address, const Config&)>;
    using InitializationStageCallback = std::function<void(const InitializationStageEnum stage)>;
    using RxPktStatsCallback = std::function<void(const uint16_t total, const uint16_t invalid_type, const uint16_t* stats)>;
    using ToMePktStatsCallback = std::function<void(const uint16_t total, const uint16_t invalid_type, const uint16_t* stats)>;
    using TxPktStatsCallback = std::function<void(const uint16_t total, const uint16_t invalid_type, const uint16_t* stats, const uint32_t tx_delay)>;
    using ReadBytesCallback  = std::function<void(uint8_t *data, size_t len)>;
    using WriteBytesCallback = std::function<void(const uint8_t *data, size_t len)>;

    struct Callbacks {
        ConfigCallback Config;
        ErrorCallback Error;
        FunctionCallback Function;
        ControllerConfigCallback ControllerConfig;
        InitializationStageCallback InitializationStage;
        RxPktStatsCallback RxPktStats;
        ToMePktStatsCallback ToMePktStats;
        TxPktStatsCallback TxPktStats;
        ReadBytesCallback ReadBytes;
        WriteBytesCallback WriteBytes;
    };

    public:
        Controller(uint8_t uart_num, uint8_t controller_address, bool transmit, uint8_t tx_delay, bool tx_test,
                   const Callbacks& callbacks, QueueHandle_t uart_event_queue = nullptr)
            : uart_num(static_cast<uart_port_t>(uart_num)), controller_address(controller_address), transmit_(transmit),
              tx_delay_(tx_delay), tx_test_(tx_test), uart_event_queue(uart_event_queue), callbacks(callbacks) {
            this->set_initialization_stage(InitializationStageEnum::DetectFeatureSupport);
            // Init the diag stats
            if (this->callbacks.RxPktStats)            
                callbacks.RxPktStats(this->rx_total, this->rx_invalid_type, this->rx_stats);
            if (this->callbacks.ToMePktStats)            
                callbacks.ToMePktStats(this->to_me_total, this->to_me_invalid_type, this->to_me_stats);
            if (this->callbacks.TxPktStats)            
                callbacks.TxPktStats(this->tx_total, this->tx_invalid_type, this->tx_stats, this->last_tx_delay);

        }

        bool start();
        bool is_initialized() const { return this->initialization_stage == InitializationStageEnum::Complete; }
        void reinitialize() { this->set_initialization_stage(InitializationStageEnum::DetectFeatureSupport); }
        InitializationStageEnum get_initialization_stage() const { return this->initialization_stage; }
        const struct Features& get_features() const { return this->features; }

        void set_current_temperature(float temperature);
        bool set_enabled(bool enabled, bool ignore_lock = false);
        bool set_economy(bool economy, bool ignore_lock = false);
        bool set_test_run(bool test_run, bool ignore_lock = false);
        bool set_setpoint(uint8_t temperature, bool ignore_lock = false);
        bool set_mode(ModeEnum mode, bool ignore_lock = false);
        bool set_fan_speed(FanSpeedEnum fan_speed, bool ignore_lock = false);
        bool set_vertical_swing(bool swing_vertical, bool ignore_lock = false);
        bool set_horizontal_swing(bool swing_horizontal, bool ignore_lock = false);
        bool advance_vertical_louver(bool ignore_lock = false);
        bool advance_horizontal_louver(bool ignore_lock = false);
        bool use_sensor(bool use_sensor, bool ignore_lock = false);
        bool transmit(bool transmit);
        bool set_tx_delay(uint8_t tx_delay);
        bool set_tx_test(bool tx_test);
        bool reset_filter(bool ignore_lock = false);
        bool maintenance(bool ignore_lock = false);

        void get_function(uint8_t function, uint8_t unit) { this->function_queue.push({ .Function = function, .Unit = unit }); }
        void set_function(uint8_t function, uint8_t value, uint8_t unit) { this->function_queue.push({ true, function, value, unit }); }

    protected:
        InitializationStageEnum initialization_stage;
        AddressTypeEnum next_token_destination_type = AddressTypeEnum::IndoorUnit;

        bool is_primary_controller() const { return this->controller_address == PrimaryAddress; }
        void set_initialization_stage(const InitializationStageEnum stage);
        void update_rx_stats(const PacketTypeEnum packetType);
        void update_to_me_stats(const PacketTypeEnum packetType);
        void update_tx_stats(const PacketTypeEnum packetType);
        void process_packet(const Packet::Buffer& buffer);
        void transmit_packet();

    private:
        uart_port_t uart_num;
        uint8_t controller_address;
        bool lastPacketToMe = false;
        PacketTypeEnum lastPacketType;
        bool skip_find_next_controller = false;
        bool transmit_ = true;
        uint8_t tx_delay_ = DefaultTXDelay;
        bool tx_test_ = false;
        // TX Delay measurement
        uint32_t rx_time = 0;
        uint32_t tx_time = 0;
        uint32_t last_tx_delay = 0;
        // RX packet  stats
        uint16_t rx_total = 0;
        uint16_t rx_invalid_type = 0;
        uint16_t rx_stats[5] = {}; // needs to align with PacketTypeEnum
        uint16_t rx_config_from_ctlr = 0;
        // Packets "to me" stats
        uint16_t to_me_total = 0;
        uint16_t to_me_invalid_type = 0;
        uint16_t to_me_stats[5] = {}; // needs to align with PacketTypeEnum
        // TX packet status
        uint16_t tx_total = 0;
        uint16_t tx_invalid_type = 0;
        uint16_t tx_stats[5] = {}; // needs to align with PacketTypeEnum
        uint16_t tx_to_ctlr = 0;
        QueueHandle_t uart_event_queue;
        Callbacks callbacks;
        std::function<void()> deferred_callback{};
        bool error_flag_changed = false;   

        // original
        //struct Features features = {};
        struct Features features = DefaultFeatures;
        struct Config current_configuration = {};
        struct Config changed_configuration = {};
        std::bitset<SettableFields::MAX> configuration_changes;
        std::queue<struct Function> function_queue;
        bool last_error_flag = false; // TODO handle errors for multiple indoor units...multiple errors per IU?

        [[noreturn]] void uart_event_task();
        void uart_read_bytes(uint8_t *buf, size_t length);
        void uart_write_bytes(const uint8_t *buf, size_t length);
};

}
