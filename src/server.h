#pragma once
#include "WebServer.h"
#include "device.h"
#include "hardware_config.h"

const char* htmlClientApp =
#include "client.html"
    ;
void setIpString(const char* sessid, const char* ip);

WebServer server(80);

void setupWifi() {
#if __has_include("secret_password")
#include "secret_password"
#else
  WiFi.begin("sessid", "password");
#endif
  WiFi.waitForConnectResult();
  setIpString("TEAM-TELECOM-ARMENIA", WiFi.localIP().toString().c_str());
  server.begin();
}

void TaskWebServer(void* pvParameters) {
  static bool firstRun = true;
  if (firstRun) {
    firstRun = false;
    server.on("/", HTTP_GET, []() { server.send(200, "text/html", htmlClientApp); });
    server.on("/parameters", HTTP_GET,
              []() { server.send(200, "application/json", parametersToJson(device.parameters)); });
    server.on("/parameters", HTTP_POST, []() {
      String json = server.arg("plain");
      updateParametersWithJson(device.parameters, json);
      server.send(200, "application/json", parametersToJson(device.parameters));
    });
    server.on("/screen_data", HTTP_GET, []() {
      String jsonResponse;
      JsonDocument doc;
      doc["width"] = 128;
      doc["height"] = 32;
      String pixels;
      for (int j = 0; j < 128; j++) {
        for (int i = 0; i < 32; i++) {
          bool isPixelOn = display_buffer[i][j];
          pixels += (isPixelOn ? "X" : ".");
        }
        pixels += "\n";
      }
      doc["pixels"] = pixels;
      serializeJson(doc, jsonResponse);
      server.send(200, "application/json", jsonResponse);
    });
    server.on("/click", HTTP_POST, []() {
      if (server.hasArg("button")) {
        String button = server.arg("button");
        if (button == "A") {
          buttonA.serverClick();
        } else if (button == "B") {
          buttonB.serverClick();
        } else if (button == "encoder") {
          encoderButton.serverClick();
        } else if (button == "encoder_increase") {
          int currentCount = encoder.getCount();
          encoder.setCount(currentCount + 4);
        } else if (button == "encoder_decrease") {
          int currentCount = encoder.getCount();
          encoder.setCount(currentCount - 4);
        }
        server.send(200, "text/plain", "Button clicked");
      } else {
        server.send(400, "text/plain", "Missing button argument");
      }
    });
  }  // if firstRun

  while (true) {
    server.handleClient();
    yield();
  };
}