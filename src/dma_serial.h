#pragma once

// A class that is compatible with MIDI.h SerialMIDI interface
// but it uses ESP32 DMA to send and receive data
// much faster

const int RX2_pin = 16;
const int TX2_pin = 17;
// we work at 1 ms, so 32 samples per ms at 31250 bauds
// 1024 must be more than enough
const int UART_BUFFER_SIZE = 1024;
uint8_t uart_buffer[UART_BUFFER_SIZE];

class DmaSerial {
  public:
  DmaSerial() {}

  void begin() {
    uart_config_t uart_config = {
        .baud_rate = 31250,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(
        uart_driver_install(UART_NUM_2, UART_BUFFER_SIZE * 2, UART_BUFFER_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(
        uart_set_pin(UART_NUM_2, TX2_pin, RX2_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  }

  template<typename T>
  bool beginTransmission(const T&) {
    return true;
  }

  void endTransmission() {}

  bool available() {
    size_t len;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, &len));
    return len > 0;
  }

  void write(uint8_t data) { uart_write_bytes(UART_NUM_2, (const char*)&data, 1); }

  uint8_t read() {
    uint8_t data;
    int len = uart_read_bytes(UART_NUM_2, &data, 1, 0);
    if (len == 1) {
      /// if (thruActivated) {
      //  uart_write_bytes(UART_NUM_2, (const char*)&data, 1);
      //}
      // uart_flush_input(UART_NUM_2);
      return data;
    }
    return 0;
  }

  bool thruActivated{false};
};
