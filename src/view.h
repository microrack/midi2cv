#pragma once

void drawIdle(const Ui::Idle& idle) {
  display.clearDisplay();
  display.setRotation(1);
  display.setFont(&Picopixel);
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.printf("BPM\n");
  display.printf("%d", int(device.parameters.bpm));

  display.setRotation(0);
  display.setFont();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("WiFi:%s\n", idle.wifiString.substr(0, 5).c_str());
  display.printf("IP%s\n", idle.ipString.c_str());
  display.printf("Audio_t:%03dus\n", idle.dspTickTime);
  if (idle.fine) {
    display.printf("Fine\n");
  }

  display.setRotation(1);

  if (idle.debug) {
    display.clearDisplay();
    display.setFont(&Picopixel);
    display.setCursor(0, 6);
    display.setTextSize(1);
    display.printf(":%d\n", int(device.parameters.bpm));
    display.printf(":%.03dus\n", idle.dspTickTime);
    display.printf(":%.2f\n", idle.debug1);
    display.printf(":%.2f\n", idle.debug2);
    display.printf(":%.2f\n", idle.debug3);
    display.printf(":%.2f\n", idle.debug4);
  }
// this stuff is funny, but it will take a ton of time to make this right
  /*
idle.inputMidiClock.prepareForVisualization();
for (int i = 0; i < idle.inputMidiClock.bufferSize; i++) {
  display.drawPixel(i, 40 + idle.inputMidiClock.get(i), SSD1306_WHITE);
}
idle.outputMidiClock.prepareForVisualization();
for (int i = 0; i < idle.outputMidiClock.bufferSize; i++) {
  display.drawPixel(i, 50 + idle.outputMidiClock.get(i), SSD1306_WHITE);
}
idle.inputSyncClock.prepareForVisualization();
for (int i = 0; i < idle.inputSyncClock.bufferSize; i++) {
  display.drawPixel(i, 60 + idle.inputSyncClock.get(i), SSD1306_WHITE);
}
idle.outputSyncClock.prepareForVisualization();
for (int i = 0; i < idle.outputSyncClock.bufferSize; i++) {
  display.drawPixel(i, 70 + idle.outputSyncClock.get(i), SSD1306_WHITE);
}*/
}

void drawMenu(const Ui::Menu& menu) {
  display.clearDisplay();
  display.setRotation(1);
  display.setCursor(0, 0);
  display.setFont();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("MENU\n");
  display.printf("----\n");
  for (int i = 0; i < menu.items.size(); i++) {
    display.printf("%s", menu.items[i].title.c_str());
    if (i == menu.currentItem) {
      display.printf("<");
    }
    display.printf("\n");
  }

  display.printf("\n");

  switch (menu.items[menu.currentItem].menuType) {
    case Ui::MenuItem::MenuType::Channel: {
      if (device.parameters.midiChannel == -1) {
        display.printf("MCH\nAll\n");
      } else {
        display.printf("MCH:\n%d\n", device.parameters.midiChannel);
      }
      break;
    }
    case Ui::MenuItem::MenuType::ModA: {
      display.printf("A:\n%d\n", device.parameters.inModChannelNumber[0]);
      break;
    }
    case Ui::MenuItem::MenuType::ModB: {
      display.printf("B:\n%d\n", device.parameters.inModChannelNumber[1]);
      break;
    }
  }
}

void drawTapTempo(const Ui::Tap& tap) {
  display.clearDisplay();
  display.setRotation(1);
  display.fillRect(0, 0, display.width(), display.height(), SSD1306_BLACK);
  display.setFont(&Picopixel);
  display.setTextSize(2);
  display.setCursor(0, 40);
  display.printf("TAP\n");
  if (tap.bpmToDisplay == -1) {
    display.printf("...");
  } else {
    display.printf("%d", tap.bpmToDisplay);
  }
}

void drawUI(const Ui& ui) {
  display.clearDisplay();
  display.setRotation(1);
  switch (ui.mode) {
    case Ui::Mode::Idle:
      drawIdle(ui.idle);
      break;
    case Ui::Mode::TapTempo:
      drawTapTempo(ui.tap);
      break;
    case Ui::Mode::MenuTransitionAnimation:
      drawMenu(ui.menu);
      break;
    case Ui::Mode::Menu:
      drawMenu(ui.menu);
      break;
  }
  for (int row = 0; row < 32; ++row) {
    for (int col = 0; col < 128; ++col) {
      display_buffer[row][col] = display.getPixel(row, col);
    }
  }

  display.display();
}
