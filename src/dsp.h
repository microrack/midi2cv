#pragma once
#include <MIDI.h>

#include <unordered_map>

constexpr int sampleRate = 1000;

// Most components expect the updates to be smooth,
// So always put OnePoleLowpass smoothers after the "analog" inputs
// RisingEdgeDetectors smooth by itself

// Input analog values are measured in volts, not ADC values
// The mapping is handled internally by voltsTo... functions

// unipolar for now

// 0 volt is 0%
// 5 volts is 100%
float voltsToGatePercentage(float volts) {
  return volts / 5.0f;
}

int voltsToMidiCcValue(float volts) {
  return int(127.0f * volts / 5.0f);
}

// just in case, not needed yet
float codecValueToVolts(int codecValue,
                        int codecBits,
                        float minCodecValueVolts,
                        float maxCodecValueVolts) {
  return minCodecValueVolts
         + (maxCodecValueVolts - maxCodecValueVolts) * codecValue / (1 << codecBits);
}

float midiNoteToCv(int midiNote) {
  // MIDI note numbers are in the range 0-127
  // Middle C (C4) is MIDI note number 60
  // Each semitone is one MIDI note number apart
  // CV (Control Voltage) for MIDI is typically 1V/octave
  // To convert MIDI note numbers to CV, we calculate the voltage as follows:
  // Voltage = (MIDI note number - 69) / 12.0
  // This formula gives us a voltage of 0V for MIDI note 69, and increases/decreases by 1V per
  // octave
  return (midiNote - 69) / 12.0f;
}

int cvToMidiNote(float cv) {
  // Inverse of the above
  return 69 + round(12.0f * cv);
}

struct LinearMapper {
  float minInput;
  float maxInput;
  float minOutput;
  float maxOutput;

  LinearMapper(float minIn, float maxIn, float minOut, float maxOut)
      : minInput(minIn), maxInput(maxIn), minOutput(minOut), maxOutput(maxOut) {}

  float map(float input) {
    return minOutput + (maxOutput - minOutput) * (input - minInput) / (maxInput - minInput);
  }
};

LinearMapper adcToVoltsDevboard{1947.96f, 3179.1f, 0.0f, 3.0f};
LinearMapper adcToVolts{0.0f, 4095.0f, -5.0f, 5.0f};
LinearMapper digitalInToAnalogIn{0.0f, 1.0f, 0.0f, 5.0f};
LinearMapper modCvToMidiCc{0.0f, 5.0f, 0.0f, 127.0f};
LinearMapper voltsToDac{0.0f, 3.2f, 0.0f, 4095.0f};

// A simple lock free (for the producer) ring buffer for visualization purposes
struct RingBuffer {
  static constexpr int bufferSize = 32;
  std::array<float, bufferSize> buffer = {0.0f};
  std::array<float, bufferSize> copy = {0.0f};
  int bufferIndex = 0;

  unsigned long changeEnd{0};

  void put(float value) {
    buffer[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % bufferSize;
    changeEnd = micros();
  }

  void prepareForVisualization() {
    // anti-tearing machinery. If we made a teared copy, try again
    unsigned long changeEndSnapshot;
    do {
      changeEndSnapshot = changeEnd;
      for (int i = bufferSize - 1; i >= 0; --i) {
        copy[bufferSize - i] = buffer[(bufferIndex + i) % bufferSize];
      }
    } while (changeEndSnapshot != changeEnd);
  }

  float get(int index) {
    if (index < bufferSize) {
      return copy[index];
    } else {
      return 0.0f;
    }
  }
};

struct DigitalTriggerToPulse {
  float gateTime{5.0f};     // Duration of the gate in milliseconds
  float elapsedTime{0.0f};  // Time elapsed since the gate was triggered
  int gateState{0};

  DigitalTriggerToPulse(float initialGateTime) : gateTime(initialGateTime) {}

  // Update the gate time
  void updateGateTime(float newGateTime) { gateTime = newGateTime; }

  int tick(int triggerInput) {
    if (triggerInput) {
      gateState = true;
      elapsedTime = 0.0f;  // Reset the elapsed time
    } else if (gateState) {
      elapsedTime += 1000.0f / sampleRate;
      if (elapsedTime >= gateTime) {
        gateState = false;
        elapsedTime = 0.0f;
      }
    }
    return gateState;
  }
};

struct RisingEdgeDetector {
  // 3 sample latency, but a more robust detection
  static constexpr int bufferSize = 3;
  std::array<float, bufferSize> buffer = {0.0f};
  int bufferIndex = 0;
  float prevSignal = 0.0f;
  float threshold = 2.5f;

  RisingEdgeDetector(float threshold_) : threshold(threshold_) {}

  float boxFilterTick(float signal) {
    buffer[bufferIndex] = signal;
    bufferIndex = (bufferIndex + 1) % bufferSize;
    float sum = 0.0f;
    for (int i = 0; i < bufferSize; ++i) {
      sum += buffer[i];
    }
    return sum / bufferSize;
  }

  int tick(float signal) {
    float smoothedSignal = boxFilterTick(signal);
    int risingEdge = (smoothedSignal > threshold) && (prevSignal <= threshold);
    prevSignal = smoothedSignal;
    return risingEdge;
  }
};

// edge detectors are different just with the threshold level
// digital signals are 0 .. 1, analog signals are 0 .. 5
struct RisingAnalogEdgeDetector : public RisingEdgeDetector {
  RisingAnalogEdgeDetector() : RisingEdgeDetector(2.5f) {}
};

struct RisingDigitalEdgeDetector : public RisingEdgeDetector {
  RisingDigitalEdgeDetector() : RisingEdgeDetector(0.5f) {}
};

template<int delay>
struct RisingAnalogEdgeDetectorWithDelay : public RisingAnalogEdgeDetector {
  std::array<int, delay> buffer;
  int bufferIndex = 0;

  RisingAnalogEdgeDetectorWithDelay() {}

  int tick(float signal) {
    int edge = RisingAnalogEdgeDetector::tick(signal);
    buffer[bufferIndex] = edge;
    bufferIndex = (bufferIndex + 1) % delay;
    return buffer[bufferIndex];
  }
};

struct Clock {
  int counter{999999999};
  float counter{999999999.0f};

  int tick(float period) {
    counter += 1.0f;
    // use while or fmod?
    if (counter >= period) {
      if (counter == 999999999.0f) {
        counter = 0.0f;
        break;
      } else {
        counter = counter - period;
      }
      return 1;
    }
    return 0;
  }
};


struct NoteOnOffToGate {
  int noteOn{0};

  int tick(int noteOnTrigger, int noteOffTrigger) {
    if (noteOnTrigger) {
      noteOn = 1;
    }
    if (noteOffTrigger) {
      noteOn = 0;
    }

    return noteOn;
  }
};

// ClockDivider is triggered only by rising edge
struct ClockDivider {
  int divisor;
  int counter{9999999};
  int prevSignal{0};
  int outputSignal{0};

  ClockDivider(int div) : divisor(div) {}

  int tick(int signal) {
    if (signal == 1 && prevSignal == 0) {
      counter++;
      if (counter >= divisor) {
        counter = 0;
        prevSignal = signal;
        return 1;
      }
    }
    prevSignal = signal;
    return 0;
  }
};

// ClockMultiplier is actually a clock with some internal tempo feel
// If it misses, it slowly adjusts the rate and the phase until it locks
// An additional clock outputs the multiplied pulses
struct ClockMultiplier {
  static constexpr bool instantJump = false;
  int prevSignal{0};
  // assume 120bpm ppqn24 value, 20ms per pulse (60/120/24)
  int clockPeriodEstimate{20};
  // when turned on, we wait for the first pulse before making any assumptions
  bool afterFirstPulse{false};
  int timeBetweenExternalPulses{0};
  // internal clock that tries to synchronize with the input, both frequency and phase
  int phaseCounter{0};
  int mult{1};
  int triggered_{0};
  int prevResult{0};

  ClockMultiplier(int multiplier) : mult(multiplier) {}

  int tick(int signal) {
    bool triggered = signal == 1 && prevSignal == 0;

    prevSignal = signal;

    // initially we just wait for the first pulse
    // and when we get it, immediately output
    if (!afterFirstPulse) {
      if (!triggered) {
        afterFirstPulse = true;
        return 1;
      }
      return 0;
    }

    ++timeBetweenExternalPulses;

    // when triggered, we adjust our internal clock frequency and phase
    if (triggered) {
      if (instantJump) {
        clockPeriodEstimate = timeBetweenExternalPulses;
        phaseCounter = 0;
      } else {
        clockPeriodEstimate = 0.5 * clockPeriodEstimate + 0.5 * timeBetweenExternalPulses;
        if (abs(clockPeriodEstimate - timeBetweenExternalPulses) < 3) {
          clockPeriodEstimate = timeBetweenExternalPulses;
        }
        // triggered_ = clockPeriodEstimate - timeBetweenExternalPulses;
        //   when we triggered, we want phase to be 0, so push us forward
        if (phaseCounter != 0) {
          phaseCounter -= 1;
        }
      }
      timeBetweenExternalPulses = 0;
    }

    // when the input is silent for a while, we turn off ourselves
    if (timeBetweenExternalPulses > clockPeriodEstimate * 4) {
      afterFirstPulse = false;
      timeBetweenExternalPulses = 0;
      phaseCounter = 0;
      return 0;
    }

    float result;

    if (phaseCounter <= mult) {
      result = 1;
    } else {
      result = 0;
    }

    phaseCounter += mult;

    if (phaseCounter >= clockPeriodEstimate) {
      phaseCounter = phaseCounter - clockPeriodEstimate;
    }

    float edgeResult = (result == 1 && prevResult == 0) ? 1 : 0;
    prevResult = result;
    return edgeResult;
  }
};

struct ClockMultiplierDivider {
  ClockMultiplier multiplier;
  ClockDivider divider;

  ClockMultiplierDivider(int mult, int div) : multiplier(mult), divider(div) {}

  int tick(int signal) { return multiplier.tick(signal); }
};

struct OnePoleLowpass {
  float coefficient{1.0f};
  float state{0.0f};

  OnePoleLowpass() : OnePoleLowpass(10.0f) {}

  OnePoleLowpass(float freq) { updateCoefficients(freq); }

  void updateCoefficients(float freq) { coefficient = exp(-2.0 * M_PI * freq / sampleRate); }
  float tick(float input) {
    state = coefficient * state + (1.0 - coefficient) * input;
    return state;
  }
};

struct DoubleLowpass {
  OnePoleLowpass lowpass1{15.0f};
  OnePoleLowpass lowpass2{15.0f};
  OnePoleLowpass lowpass3{15.0f};
  OnePoleLowpass lowpass4{15.0f};

  DoubleLowpass() = default;

  DoubleLowpass(float freq) {
    lowpass1.updateCoefficients(freq);
    lowpass2.updateCoefficients(freq);
    lowpass3.updateCoefficients(freq);
    lowpass4.updateCoefficients(freq);
  }

  float tick(float input) {
    float state = lowpass1.tick(input);
    state = lowpass2.tick(state);
    state = lowpass3.tick(state);
    state = lowpass4.tick(state);
    return state;
  }
};

template<int bufferSize = 15>
struct MedianFilter {
  std::array<float, bufferSize> buffer = {0.0f};
  int bufferIndex = 0;

  float tick(float input) {
    buffer[bufferIndex] = input;
    bufferIndex = (bufferIndex + 1) % bufferSize;
    std::sort(buffer.begin(), buffer.end());
    return buffer[bufferSize / 2];
  }
};

struct OnePoleVariableLowpass {
  float coefficient{1.0f};
  float state{0.0f};
  float oldFreq{-1.0f};

  void updateCoefficients(float freq) { coefficient = exp(-2.0 * M_PI * freq / sampleRate); }

  float tick(float input, float newFreq = -1) {
    if (newFreq != -1) {
      if (newFreq != oldFreq) {
        updateCoefficients(newFreq);
        oldFreq = newFreq;
      }
    }
    state = coefficient * state + (1.0 - coefficient) * input;
    return state;
  }
};

struct Dsp {
  static constexpr int numCvGateInputChannels = 1;
  static constexpr int numModInputChannels = 2;

  static constexpr int numCvGateOutputChannels = 1;
  static constexpr int numModOutputChannels = 1;

  enum class ClockSource { Internal, SyncIn, MidiIn };

  ClockSource prevClockSource{ClockSource::Internal};
  ClockSource clockSource{ClockSource::Internal};

  float gateTime;

  struct Processors {
    // clock part
    //
    RisingDigitalEdgeDetector syncInDetector;
    // 5 ms pulses for analog sync output
    DigitalTriggerToPulse clockTriggerToPulse{5.0};
    OnePoleLowpass smoothClockPeriod{10.0};

    // ppqn24
    Clock highestResolutionClock;
    // ppqn4
    ClockDivider internalClock{6};
    ClockMultiplier syncInToPpqn24{6};

    // midi-to-cv-gate part
    //
    OnePoleLowpass smoothGlide{10.0};
    std::array<OnePoleVariableLowpass, numCvGateOutputChannels> CVOutWithGlide;
    std::array<NoteOnOffToGate, numCvGateOutputChannels> noteOnOffToGate;

    // pins-to-midi part
    //
    // 25 ms delay to deal with crappy ESP32 ADC
    std::array<MedianFilter<20>, numCvGateOutputChannels> smoothCvForMidi;
    std::array<RisingAnalogEdgeDetectorWithDelay<25>, numModInputChannels> gateRisingDetector;
    std::array<RisingAnalogEdgeDetectorWithDelay<25>, numModInputChannels> gateFallingDetector;
  };

  Processors p;

  struct Input {
    // clock part
    //
    float tempo{120.0};
    float syncIn{0.0};

    int midiClockTrigger{0};

    // midi-to-cv-gate part
    //
    std::array<float, numCvGateInputChannels> midiNoteCv;
    std::array<int, numCvGateInputChannels> noteOnTrigger;
    std::array<int, numCvGateInputChannels> noteOffTrigger;
    float glide{10};

    // pins-to-midi part
    //
    std::array<float, numCvGateInputChannels> pinCv;
    std::array<float, numCvGateOutputChannels> pinGate;
    std::array<float, numModOutputChannels> pinMod;
  };

  struct Output {
    // clock part
    //
    int triggerHighestRes{0};
    int triggerSync{0};
    int pulseSync{0};
    int midiClockTrigger{0};

    // midi-to-cv-gate part
    std::array<float, numCvGateInputChannels> cv;
    std::array<int, numCvGateInputChannels> gate;

    // pins-to-midi part
    std::array<float, numCvGateOutputChannels> cvForMidi;
    std::array<int, numCvGateOutputChannels> cvToMidiNoteOnTrigger;
    std::array<int, numCvGateOutputChannels> cvToMidiNoteOffTrigger;

    std::array<int, numModOutputChannels> modValueFromPin;
    ;
  };

  void reset() {
    p = Processors();
    p.smoothClockPeriod.state = float(sampleRate * 60 / 120 / 4);
  }

  std::array<float, numCvGateInputChannels> lastCvMidiNote;

  std::array<int, numCvGateOutputChannels> soundingNoteForPin;
  std::array<int, numModOutputChannels> lastModValueFromPin;

  std::unordered_map<int, int> currentlySoundingNotes;

  Output tick(Input i) {
    Output o{};

    // reset when global settings changed
    if (clockSource != prevClockSource) {
      reset();
    }

    //
    // clock part
    //

    // the idea is to minimize the amount of clock conversions
    // if we have internal clock, we can generate ppqn24 for midi out right away
    // if we have sync in, we need a single clock multiplier from ppqnX to ppqn24 (the hardest case)
    // if we have midi in, we need only clock division for analog clocking and pass through the midi
    // to the output
    switch (clockSource) {
      case ClockSource::Internal: {
        float bpm = i.tempo;
        // 16th notes is ppqn4
        float period = 60.0f / bpm / 4 * sampleRate;

        float smoothedInternalClockPeriod = p.smoothClockPeriod.tick(period);
        float smoothedPeriodPpqn24 = smoothedInternalClockPeriod / 6.0f;

        o.triggerHighestRes = p.highestResolutionClock.tick(smoothedPeriodPpqn24);

        o.triggerSync = p.internalClock.tick(o.triggerHighestRes);
        break;
      }
      case ClockSource::SyncIn: {
        o.triggerSync = p.syncInDetector.tick(i.syncIn);
        o.triggerHighestRes = p.syncInToPpqn24.tick(o.triggerSync);
        break;
      }
      case ClockSource::MidiIn: {
        o.triggerHighestRes = i.midiClockTrigger;
        o.triggerSync = p.internalClock.tick(o.triggerHighestRes);
        break;
      }
    }
    prevClockSource = clockSource;
    o.pulseSync = p.clockTriggerToPulse.tick(o.triggerSync);

    o.midiClockTrigger = o.triggerHighestRes;

    //
    // midi-to-cv-gate part
    // most of logic here is outside of the dsp
    // as the most interesting part is how we assign the midi events to different
    // cv-gate channels
    //

    float smoothedGlide = p.smoothGlide.tick(i.glide);
    for (int c = 0; c < numCvGateOutputChannels; ++c) {
      // let's not support glide for now
      // o.cv[c] = p.CVOutWithGlide[c].tick(i.midiNoteCv[c], 0.2 /*smoothedGlide*/);
      o.cv[c] = i.midiNoteCv[c];
      o.gate[c] = p.noteOnOffToGate[c].tick(i.noteOnTrigger[c], i.noteOffTrigger[c]);
    }

    //
    // pins-to-midi part
    //

    for (int c = 0; c < numCvGateOutputChannels; ++c) {
      o.cvForMidi[c] = p.smoothCvForMidi[c].tick(i.pinCv[c]);
      o.cvToMidiNoteOnTrigger[c] = p.gateRisingDetector[c].tick(i.pinGate[c]);
      o.cvToMidiNoteOffTrigger[c] = p.gateFallingDetector[c].tick(5.0 - i.pinGate[c]);

      // filter out short pulses
      if (o.cvToMidiNoteOnTrigger[c] == 1 && o.cvToMidiNoteOffTrigger[c] == 1) {
        o.cvToMidiNoteOnTrigger[c] = 0;
        o.cvToMidiNoteOffTrigger[c] = 0;
      }
    }

    for (int c = 0; c < numModOutputChannels; ++c) {
      o.modValueFromPin[c] = voltsToMidiCcValue(i.pinMod[c]);
    }

    return o;
  }

  template<typename Midi>
  Output processWithMidi(Input& input, Midi& midi) {
    while (midi.read()) {
      switch (midi.getType()) {
        case midi::NoteOn: {
          currentlySoundingNotes[midi.getData1()] = midi.getData2();
          lastCvMidiNote[0] = midi.getData1();
          break;
        }
        case midi::NoteOff: {
          input.noteOffTrigger[0] = 1;
          currentlySoundingNotes.erase(midi.getData1());
          break;
        }
        case midi::Clock: {
          input.midiClockTrigger = 1;
          break;
        }
        case midi::ControlChange: {
          break;
        }
        case midi::PitchBend: {
          break;
        }
        case midi::AfterTouchPoly: {
          break;
        }
        case midi::AfterTouchChannel: {
          break;
        }
        case midi::ProgramChange: {
          break;
        }
        case midi::SystemExclusive: {
          break;
        }
        case midi::TimeCodeQuarterFrame: {
          break;
        }
        case midi::SongPosition: {
          break;
        }
        case midi::SongSelect: {
          break;
        }
        case midi::TuneRequest: {
          break;
        }
        case midi::Start: {
          break;
        }
        case midi::Continue: {
          break;
        }
        case midi::Stop: {
          break;
        }
        case midi::ActiveSensing: {
          break;
        }
        case midi::SystemReset: {
          break;
        }
        default: {
          break;
        }
      }
    }

    input.midiNoteCv[0] = midiNoteToCv(lastCvMidiNote[0]);
    Dsp::Output o = tick(input);

    if (o.midiClockTrigger) {
      midi.sendClock();
    }

    // pins-to-midi part
    for (int c = 0; c < numCvGateOutputChannels; ++c) {
      auto currentNote = cvToMidiNote(o.cvForMidi[c]);
      bool newNote = false;
      if (o.cvToMidiNoteOnTrigger[c]) {
        newNote = true;
      }
      // this highly depends on low noise cvForMidi
      // if it's noisy, a log of extra notes will appear
      if (currentNote != soundingNoteForPin[c]) {
        newNote = true;
      }
      if (newNote) {
        if (soundingNoteForPin[c] != -1) {
          // this shouldn't happen, unless filtering logic in RisingAnalogEdgeDetector is glitching
          midi.sendNoteOff(soundingNoteForPin[c], 127, 1);
        }

        /*if (currentNote != soundingNoteForPin[c]) {
          midi.sendNoteOff(soundingNoteForPin[c], 127, 1);
        }*/
        soundingNoteForPin[c] = currentNote;
        midi.sendNoteOn(soundingNoteForPin[c], 127, 1);
      }
      if (o.cvToMidiNoteOffTrigger[c]) {
        midi.sendNoteOff(soundingNoteForPin[c], 127, 1);
        soundingNoteForPin[c] = -1;
      }
    }

    for (int c = 0; c < numModOutputChannels; ++c) {
      int mapped = modCvToMidiCc.map(o.modValueFromPin[c]);
      // if control changes, i.e. we send 1 message per 1 ms,
      // we are at 150 us per audio tick
      // thus,
      if (lastModValueFromPin[c] != mapped) {
        lastModValueFromPin[c] = mapped;
        midi.sendControlChange(c, mapped, 1);
      }
    }

    return o;
  }
};

void setupDsp() {}
