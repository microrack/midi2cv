#include <Fonts/Picopixel.h>

#include <string>
#include <vector>

#include "Arduino.h"
#include "dsp.h"
#include "hardware_config.h"
#include "parameters.h"
uint8_t display_buffer[32][128];
#include "server.h"

TaskHandle_t TaskUpdateDebouncerHandle = NULL;
TaskHandle_t TaskWebServerHandle = NULL;
TaskHandle_t TaskUIHandle = NULL;
TaskHandle_t TaskMusicHandle = NULL;
void TaskUpdateDebouncer(void* pvParameters);
void TaskUI(void* pvParameters);
void TaskMusicProcessing(void* pvParameters);

void setupWifi();

void setup() {
  setupHardware();
  // setupWifi();
  setupDsp();

  // the first core is dedicated to UI/WiFI and other non-realtime tasks
  // the second core is dedicated to music

  xTaskCreatePinnedToCore(TaskUI, "TaskUI",
                          10000,  // Stack size in words
                          NULL,   // Task input parameter
                          1,      // Priority of the task
                          &TaskUIHandle,
                          0);  // Core where the task should run

  xTaskCreatePinnedToCore(TaskUpdateDebouncer, "TaskDebouncer", 10000, NULL, 0,
                          &TaskUpdateDebouncerHandle, 0);

  // xTaskCreatePinnedToCore(TaskWebServer, "TaskWebServer", 10000, NULL, 1, &TaskWebServerHandle, 0);

  xTaskCreatePinnedToCore(TaskMusicProcessing, "TaskMusic", 50000, NULL, 1, &TaskMusicHandle, 1);
}

#include "view.h"

void setIpString(const char* sessid, const char* ip) {
  device.ui.idle.wifiString = sessid;
  device.ui.idle.ipString = ip;
}

void handleActions(MidiToCvDevice& device) {
  int encoder_delta = encoder.getCount();
  encoder.clearCount();

  switch (device.ui.mode) {
    case Ui::Mode::Idle: {
      if (buttonA.isPressedNow()) {
        device.parameters.bpm += encoder_delta * 0.1;
        device.ui.idle.fine = true;
      } else {
        device.parameters.bpm += encoder_delta * 0.25;
        device.ui.idle.fine = false;
      }
      if (buttonB.isNewPress()) {
        device.tapTempo.reset();
        device.tapTempo.addTap();
        device.ui.tap.bpmToDisplay = -1;
        device.ui.mode = Ui::Mode::TapTempo;
      }
      if (encoderButton.isNewPress()) {
        device.ui.mode = Ui::Mode::Menu;
      }
      break;
    }
    case Ui::Mode::TapTempo: {
      if (buttonB.isNewPress()) {
        float estimatedBpm = device.tapTempo.addTap(esp_timer_get_time());
        device.ui.tap.bpmToDisplay = -1;
        if (estimatedBpm > 0) {
          // don't we need to smooth it even more?
          device.ui.tap.bpmToDisplay = estimatedBpm;
          device.parameters.bpm = estimatedBpm;
        }
      }
      // if time since last button press is more than 2 second, return to Idle
      if (device.tapTempo.lastTapTime + 2000000 < esp_timer_get_time()) {
        device.ui.mode = Ui::Mode::Idle;
      }

      break;
    }
    case Ui::Mode::Menu: {
      if (buttonA.isNewPress()) {
        device.ui.menu.previousItem = device.ui.menu.currentItem;
        device.ui.menu.currentItem = (device.ui.menu.currentItem - 1) % device.ui.menu.items.size();
        device.ui.mode = Ui::Mode::MenuTransitionAnimation;
      }

      if (buttonB.isNewPress()) {
        device.ui.menu.previousItem = device.ui.menu.currentItem;
        device.ui.menu.currentItem = (device.ui.menu.currentItem + 1) % device.ui.menu.items.size();
        device.ui.mode = Ui::Mode::MenuTransitionAnimation;
      }

      // Encoder changes current item
      auto& currentItem = device.ui.menu.items[device.ui.menu.currentItem];
      switch (currentItem.menuType) {
        case Ui::MenuItem::MenuType::Channel: {
          int newMidiChannel = device.parameters.midiChannel + encoder_delta / 2;
          if (newMidiChannel < -1) {
            newMidiChannel = 15;
          }
          if (newMidiChannel > 15) {
            newMidiChannel = -1;
          }
          device.parameters.midiChannel = newMidiChannel;
          break;
        }
        case Ui::MenuItem::MenuType::Sync: {
          int delta = encoder_delta / 2;
          int newValue = static_cast<int>(device.parameters.clockSource) + delta;
          if (newValue <= 0) {
            newValue = 0;
          }
          if (newValue > 2) {
            newValue = 2;
          }
          device.parameters.clockSource = static_cast<ClockInputSignal>(newValue);
          break;
        }
        case Ui::MenuItem::MenuType::ModA: {
          int newModA = device.parameters.inModChannelNumber[0] + encoder_delta / 2;
          if (newModA < 0) {
            newModA = 127;
          }
          if (newModA > 127) {
            newModA = 0;
          }
          device.parameters.inModChannelNumber[0] = newModA;
          break;
        }
        case Ui::MenuItem::MenuType::ModB: {
          int newModB = device.parameters.inModChannelNumber[1] + encoder_delta / 2;
          if (newModB < 0) {
            newModB = 127;
          }
          if (newModB > 127) {
            newModB = 0;
          }
          device.parameters.inModChannelNumber[1] = newModB;
          break;
        }
      }

      if (encoderButton.isNewPress()) {
        device.ui.mode = Ui::Mode::Idle;
      }
      break;
    }
    case Ui::Mode::MenuTransitionAnimation: {
      device.ui.mode = Ui::Mode::Menu;
    }
  }
}

void TaskUpdateDebouncer(void* pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(1);
  for (;;) {
    buttonA.update();
    buttonB.update();
    encoderButton.update();
    vTaskDelay(xDelay);
  }
}

void TaskUI(void* pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(20);

  for (;;) {
    drawUI(device.ui);
    handleActions(device);
    vTaskDelay(xDelay);
  }
}

int16_t buffer[4096];
int bufferIndex = 0;

void TaskMusicProcessing(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  static bool firstRun = true;
  if (firstRun) {
    firstRun = false;
    MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.turnThruOff();
  }

  for (;;) {
    // timings: midi arduino ~ 140 us with spikes up to 1000-2000
    // midi DMA ~ 10 us with spikes up to 50 ms (probably when the clock is sent)
    // ditialRead/write ~ 0 us
    // dsp ~ 10 us
    // analogRead 500 us :E
    // + ledcWrite 2000 us :EEE
    // but ledc alone is 50 us. Hmm....
    // switching to a more native esp32 analog read gives 250 in total

    // if we multisample, better doing it with some delay
    auto prev_value = read_adc_idf(ADC_0);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    unsigned long startTick = micros();

    Dsp::Input input{};
    input.tempo = device.parameters.bpm;
    input.syncIn = digitalRead(SYNC_IN);

    input.glide = device.parameters.glide;

    auto value = read_adc_idf(ADC_0);
    auto mean_value = (prev_value + value) / 2.0f;

    input.pinCv[0] = adcToVoltsDevboard.map(mean_value);
    input.pinGate[0] = adcToVoltsDevboard.map(read_adc_idf(ADC_1));
    input.pinMod[0] = adcToVoltsDevboard.map(read_adc_idf(ADC_2));
    // input.pinMod[1] = adcToVoltsDevboard.map(read_adc_idf(ADC_3));

    switch (device.parameters.clockSource) {
      case ClockInputSignal::Internal: {
        device.dsp.clockSource = Dsp::ClockSource::Internal;
        break;
      }
      case ClockInputSignal::Midi: {
        device.dsp.clockSource = Dsp::ClockSource::MidiIn;
        break;
      }
      case ClockInputSignal::Ppqn4: {
        device.dsp.clockSource = Dsp::ClockSource::SyncIn;
        break;
      }
    }

    Dsp::Output output = device.dsp.processWithMidi(input, MIDI);

    if (input.midiClockTrigger) {
      device.ui.idle.debug1 = 1;
    } else {
      device.ui.idle.debug1 = 0;
    }
    device.ui.idle.debug2 = output.midiClockTrigger;

    // PWM output
    ledcWrite(0, voltsToDac.map(output.cv[0]));

    digitalWrite(SYNC_OUT, output.pulseSync);

    unsigned long endTick = micros();
    device.ui.idle.dspTickTime = endTick - startTick;
    // uncomment to get a debug layout in UI
    device.ui.idle.debug = true;
    device.ui.idle.debug1 = output.cv[0]; // input.pinCv[0];
    device.ui.idle.debug2 = input.pinGate[0];
    device.ui.idle.debug3 = output.gate[0];
    device.ui.idle.debug4 = round(output.cv[0] * 100.0f) / 100.0f;
  }
}

void loop() {}
