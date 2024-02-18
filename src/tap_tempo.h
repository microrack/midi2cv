#pragma once
#include <array>
#include <cstdint>

struct TapTempo {
  int tapCount{0};
  int tapIndex{0};
  std::array<int, 6> taps;

  int64_t lastTapTime{0};

  void reset() {
    tapCount = 0;
    tapIndex = 0;
    lastTapTime = 0;
  }

  float addTap() { return addTap(esp_timer_get_time()); }

  float addTap(int64_t timeInMicros) {
    if (lastTapTime == 0) {
      lastTapTime = timeInMicros;
      return 0.0f;
    }
    taps[tapIndex] = timeInMicros - lastTapTime;
    lastTapTime = timeInMicros;
    tapIndex = (tapIndex + 1) % taps.size();
    ++tapCount;
    if (tapCount > taps.size()) {
      tapCount = taps.size();
    }

    // 1. calculate average
    // 2. remove outliers
    // 3. calculate average again
    // 4. convert micros to bpm
    // 5. return bpm

    // Calculate average time between taps
    int64_t sum = 0;
    for (int i = 0; i < tapCount && i < taps.size(); ++i) {
      sum += taps[i];
    }

    float average = static_cast<float>(sum) / (tapCount - 1);

    if (tapCount == 2) {
      return 60000000.0f / average;
    }

    int64_t sumValid = 0;
    int validTaps = 0;
    for (int i = 1; i < tapCount && i < taps.size(); ++i) {
      float diff = float(taps[i]);
      if (0.8 * average < diff && diff < 1.2 * average) {
        sumValid += diff;
        validTaps++;
      }
    }

    float averageValid = static_cast<float>(sumValid) / validTaps;
    return 60000000.0f / averageValid;
  }
};