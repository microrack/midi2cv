#pragma once

// just some drafts. OSC will be used

enum class ClockInputSignal {
  Internal,
  Midi,   // midi is Ppqn24
  Ppqn4,  // standard step per 16th note, should be default
  Ppqn2,  // korg volca default
  Ppqn24,
  Ppqn48
};

enum class CvOutputMode { Low, High, First, Last, ArpUp, ArpDown };

// Parameters that can be controlled from UI and web
// We might want to store them periodically
struct Parameters {
  float bpm{120};
  int midiChannel{-1};
  ClockInputSignal clockSource{ClockInputSignal::Internal};
  CvOutputMode cvOutputMode{CvOutputMode::Low};

  // works only if Ppqb4 is selected
  bool swing;

  float pitchAdjustment;
  float glide{1};

  // If we don't have midi connected, we can output gate based on clock
  // this is in percentage of the clock period
  float gateLength;

  std::array<int, Dsp::numModInputChannels> inModChannelNumber{33, 71};
  std::array<float, Dsp::numModInputChannels> inModChannelLowVoltage;
  std::array<float, Dsp::numModInputChannels> inModChannelHighVoltage;

  bool midiThru;
};

#include <ArduinoJson.h>

// Convert Parameters struct to JSON string
String parametersToJson(const Parameters& params) {
  JsonDocument doc;

  doc["bpm"] = params.bpm;
  doc["midiChannel"] = params.midiChannel;
  doc["clockSource"] = static_cast<int>(params.clockSource);
  doc["cvOutputMode"] = static_cast<int>(params.cvOutputMode);
  doc["swing"] = params.swing;
  doc["pitchAdjustment"] = params.pitchAdjustment;
  doc["glide"] = params.glide;
  doc["gateLength"] = params.gateLength;
  for (size_t i = 0; i < params.inModChannelNumber.size(); ++i) {
    doc["inModChannelNumber"][i] = params.inModChannelNumber[i];
    doc["inModChannelLowVoltage"][i] = params.inModChannelLowVoltage[i];
    doc["inModChannelHighVoltage"][i] = params.inModChannelHighVoltage[i];
  }
  doc["midiThru"] = params.midiThru;

  String output;
  serializeJson(doc, output);
  return output;
}

// Update Parameters struct with values from JSON string
void updateParametersWithJson(Parameters& params, const String& json) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  params.bpm = doc["bpm"] | params.bpm;
  params.midiChannel = doc["midiChannel"] | params.midiChannel;
  params.clockSource = static_cast<ClockInputSignal>(doc["clockSource"].as<int>());
  params.cvOutputMode = static_cast<CvOutputMode>(doc["cvOutputMode"].as<int>());
  params.swing = doc["swing"] | params.swing;
  params.pitchAdjustment = doc["pitchAdjustment"] | params.pitchAdjustment;
  params.glide = doc["glide"] | params.glide;
  params.gateLength = doc["gateLength"] | params.gateLength;
  for (size_t i = 0; i < params.inModChannelNumber.size(); ++i) {
    params.inModChannelNumber[i] = doc["inModChannelNumber"][i] | params.inModChannelNumber[i];
    params.inModChannelLowVoltage[i]
        = doc["inModChannelLowVoltage"][i] | params.inModChannelLowVoltage[i];
    params.inModChannelHighVoltage[i]
        = doc["inModChannelHighVoltage"][i] | params.inModChannelHighVoltage[i];
  }
  params.midiThru = doc["midiThru"] | params.midiThru;
}
