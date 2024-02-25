// define so that arduino-style IDEs ignore this file
#ifdef PYBIND11_TESTING_MODULE
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

unsigned int micros() {
  return 0;
}

#include "../dsp.h"

namespace py = pybind11;

// Helper function to convert Python dict to Dsp::Input
Dsp::Input convertDictToInput(py::dict inputDict) {
  Dsp::Input input;

  // For simple types
  input.tempo = inputDict["tempo"].cast<float>();
  input.syncIn = inputDict["syncIn"].cast<float>();
  input.midiClockTrigger = inputDict["midiClockTrigger"].cast<int>();
  input.glide = inputDict["glide"].cast<float>();

  // For std::array fields, assuming numCvGateInputChannels and other constants are correctly
  // defined
  if (inputDict.contains("midiNoteCv")) {
    auto midiNoteCvList = inputDict["midiNoteCv"].cast<std::vector<float>>();
    std::copy(midiNoteCvList.begin(), midiNoteCvList.end(), input.midiNoteCv.begin());
  }
  if (inputDict.contains("noteOnTrigger")) {
    auto noteOnTriggerList = inputDict["noteOnTrigger"].cast<std::vector<int>>();
    std::copy(noteOnTriggerList.begin(), noteOnTriggerList.end(), input.noteOnTrigger.begin());
  }
  if (inputDict.contains("noteOffTrigger")) {
    auto noteOffTriggerList = inputDict["noteOffTrigger"].cast<std::vector<int>>();
    std::copy(noteOffTriggerList.begin(), noteOffTriggerList.end(), input.noteOffTrigger.begin());
  }
  if (inputDict.contains("pinCv")) {
    auto pinCvList = inputDict["pinCv"].cast<std::vector<float>>();
    std::copy(pinCvList.begin(), pinCvList.end(), input.pinCv.begin());
  }
  if (inputDict.contains("pinGate")) {
    auto pinGateList = inputDict["pinGate"].cast<std::vector<float>>();
    std::copy(pinGateList.begin(), pinGateList.end(), input.pinGate.begin());
  }
  if (inputDict.contains("pinMod")) {
    auto pinModList = inputDict["pinMod"].cast<std::vector<float>>();
    std::copy(pinModList.begin(), pinModList.end(), input.pinMod.begin());
  }

  return input;
}

// Helper function to convert Dsp::Output to Python dict
py::dict convertOutputToDict(Dsp::Output output) {
  py::dict outputDict;

  // For simple types
  outputDict["triggerSync"] = output.triggerSync;
  outputDict["pulseSync"] = output.pulseSync;
  outputDict["midiClockTrigger"] = output.midiClockTrigger;

  // For std::array fields, converting to Python lists
  outputDict["cv"] = py::cast(output.cv);
  outputDict["gate"] = py::cast(output.gate);
  outputDict["cvForMidi"] = py::cast(output.cvForMidi);
  outputDict["cvToMidiNoteOnTrigger"] = py::cast(output.cvToMidiNoteOnTrigger);
  outputDict["cvToMidiNoteOffTrigger"] = py::cast(output.cvToMidiNoteOffTrigger);
  outputDict["modValueFromPin"] = py::cast(output.modValueFromPin);

  return outputDict;
}

PYBIND11_MODULE(dsp, m) {
  py::enum_<Dsp::ClockSource>(m, "ClockSource")
      .value("Internal", Dsp::ClockSource::Internal)
      .value("SyncIn", Dsp::ClockSource::SyncIn)
      .value("MidiIn", Dsp::ClockSource::MidiIn)
      .export_values();

  py::class_<Dsp>(m, "Dsp")
      .def(py::init<>())
      .def("tick",
           [](Dsp& self, py::dict inputDict) -> py::dict {
             // Convert inputDict to Dsp::Input
             Dsp::Input input = convertDictToInput(inputDict);

             // Call the tick function
             Dsp::Output output = self.tick(input);

             // Convert output to a Python dictionary
             py::dict outputDict = convertOutputToDict(output);
             return outputDict;
           })
      .def("set_clock_source",
           [](Dsp& self, Dsp::ClockSource source) { self.clockSource = source; });
}

#endif
