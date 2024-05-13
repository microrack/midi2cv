#pragma once
// analogRead is very slow, so the header tries to overcome this
// based on
// https://gist.github.com/gresan-gits/49f1c5dd860a940f4f72e1d29f9edf6d
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_system.h"

// Multisampling
// Even multisampling 1 is kind of slow
#define NO_OF_SAMPLES 1
#define REF_VOLTAGE   1100

#define LIN_COEFF_A_SCALE 65536
#define LIN_COEFF_A_ROUND (LIN_COEFF_A_SCALE / 2)
#define ADC_12_BIT_RES    4096

std::array<uint8_t, 3> adcPins = {ADC_0, ADC_1, ADC_2};
std::array<esp_adc_cal_characteristics_t, 3> adc_chars;

uint8_t adcPinToCharsIndex(uint8_t pin) {
  switch (pin) {
    case ADC_0:
      return 0;
    case ADC_1:
      return 1;
    case ADC_2:
      return 2;
  }
  return 0;
}

adc1_channel_t get_adc1_channel(uint8_t pin) {
  adc1_channel_t chan;
  switch (pin) {
    case 32:
      chan = ADC1_CHANNEL_4;
      break;
    case 33:
      chan = ADC1_CHANNEL_5;
      break;
    case 34:
      chan = ADC1_CHANNEL_6;
      break;
    case 35:
      chan = ADC1_CHANNEL_7;
      break;
    case 36:
      chan = ADC1_CHANNEL_0;
      break;
    case 37:
      chan = ADC1_CHANNEL_1;
      break;
    case 38:
      chan = ADC1_CHANNEL_2;
      break;
    case 39:
      chan = ADC1_CHANNEL_3;
      break;
  }
  return chan;
}

float read_adc_idf(uint8_t pin) {
  uint32_t adc_reading = 0;

  // Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw((adc1_channel_t)get_adc1_channel(pin));
  }
  adc_reading /= NO_OF_SAMPLES;

  return adc_reading;
}

float read_adc_idf_voltage(uint8_t pin) {
  uint32_t adc_reading = 0;

  // Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    adc_reading += adc1_get_raw((adc1_channel_t)get_adc1_channel(pin));
  }
  adc_reading /= NO_OF_SAMPLES;
  // Convert adc_reading to voltage in mV
  // This doesn't work, check esp_adc_cal_characterize call below
  float voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars[adcPinToCharsIndex(pin)]);

  return voltage;
}

void check_efuse(void) {
  // Check TP is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    Serial.print("eFuse Two Point: Supported\n");
  } else {
    Serial.print("eFuse Two Point: NOT supported\n");
  }

  // Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    Serial.print("eFuse Vref: Supported\n");
  } else {
    Serial.print("eFuse Vref: NOT supported\n");
  }
}
void print_char_val_type(esp_adc_cal_value_t val_type) {
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.print("Characterized using Two Point Value\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.print("Characterized using eFuse Vref\n");
  } else {
    Serial.print("Characterized using Default Vref\n");
  }
}
void adc_setup() {
  auto bits = ADC_WIDTH_12Bit;
  auto atten = ADC_ATTEN_11db;
  adc1_config_width(bits);
  for (int c = 0; c < adcPins.size(); c++) {
    adc1_config_channel_atten(get_adc1_channel(adcPins[c]), atten);
    esp_adc_cal_value_t val_type
        = esp_adc_cal_characterize(ADC_UNIT_1, atten, bits, REF_VOLTAGE, &adc_chars[c]);
    print_char_val_type(val_type);
  }
}
