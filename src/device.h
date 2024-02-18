#pragma once
#include "dsp.h"
#include "parameters.h"
#include "tap_tempo.h"

struct Ui {
  struct Idle {
    std::string wifiString;
    std::string ipString;
    int dspTickTime;
    bool fine;

    mutable RingBuffer inputMidiClock;
    mutable RingBuffer outputMidiClock;
    mutable RingBuffer inputSyncClock;
    mutable RingBuffer outputSyncClock;

    bool debug;
    float debug1;
    float debug2;
    float debug3;
    float debug4;
  };

  struct Tap {
    int bpmToDisplay{-1};
  };

  struct MenuItem {
    enum class MenuType { Channel, ModA, ModB };
    MenuType menuType;
    std::string title;
    std::string helpText;
  };

  struct Menu {
    int itemsVerticalShift{0};
    int previousItem{0};
    int currentItem{0};
    std::array<MenuItem, 3> items = {
        MenuItem{MenuItem::MenuType::Channel, "MCH", "MIDI channel"},
        MenuItem{MenuItem::MenuType::ModA, "CCA", "Modulation input A"},
        MenuItem{MenuItem::MenuType::ModB, "CCB", "Modulation input B"},
    };
  };

  enum class Mode { Idle, TapTempo, Menu, MenuTransitionAnimation };
  Mode mode;

  Idle idle;
  Tap tap;
  Menu menu;
};

struct MidiToCvDevice {
  Ui ui;
  Dsp dsp;
  Parameters parameters;

  TapTempo tapTempo;
  int encoder_delta;
};

MidiToCvDevice device;