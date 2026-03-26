#include "WebInterface.h"

#include <WebServer.h>
#include <WiFi.h>
#include <math.h>
#include <string.h>

#include "AppShared.h"
#include "LightControlConfig.h"

namespace {

WebServer webServer(80);
bool webServerStarted = false;
bool fallbackApActive = false;

String htmlEscape(const char *text) {
  if (text == nullptr) {
    return String();
  }

  String escaped;
  for (const char *cursor = text; *cursor != '\0'; ++cursor) {
    switch (*cursor) {
      case '&':
        escaped += F("&amp;");
        break;
      case '<':
        escaped += F("&lt;");
        break;
      case '>':
        escaped += F("&gt;");
        break;
      case '"':
        escaped += F("&quot;");
        break;
      case '\'':
        escaped += F("&#39;");
        break;
      default:
        escaped += *cursor;
        break;
    }
  }

  return escaped;
}

String checkedAttribute(bool enabled) {
  return enabled ? F(" checked") : String();
}

void buildFallbackApSsid(char *buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%s-%s", LightControlConfig::FALLBACK_AP_SSID_PREFIX, LightControlConfig::MQTT_DEVICE_ID);
}

void redirectToRoot() {
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

String renderManualForm(const char *label, const char *motorToken, float speedPercent) {
  String html;
  html += F("<form method='post' action='/manual' class='row'>");
  html += F("<input type='hidden' name='motor' value='");
  html += htmlEscape(motorToken);
  html += F("'>");
  html += F("<strong>");
  html += htmlEscape(label);
  html += F("</strong>");
  html += F("<label>Speed % <input type='number' name='speed' min='0' max='100' step='1' value='");
  html += String(speedPercent, 0);
  html += F("'></label>");
  html += F("<button type='submit' name='action' value='cw'>Run CW</button>");
  html += F("<button type='submit' name='action' value='ccw'>Run CCW</button>");
  html += F("<button type='submit' name='action' value='stop'>Stop</button>");
  html += F("</form>");
  return html;
}

String renderRootPage() {
  SharedState stateSnapshot;
  copySharedState(stateSnapshot);

  RuntimeSettings settingsSnapshot;
  copyRuntimeSettings(settingsSnapshot);

  const IPAddress stationIp = WiFi.localIP();
  const IPAddress apIp = WiFi.softAPIP();

  String page;
  page.reserve(9000);

  page += F("<!doctype html><html><head><meta charset='utf-8'>");
  page += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>ESP32 Mirror Ball Control</title>");
  page += F("<style>");
  page += F("body{font-family:Arial,sans-serif;background:#f4f6f8;color:#111;margin:0;padding:20px;}");
  page += F("h1,h2{margin:0 0 10px 0;}section{background:#fff;border-radius:12px;padding:16px;margin:0 0 16px 0;box-shadow:0 2px 8px rgba(0,0,0,.08);}");
  page += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;}");
  page += F(".row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:8px 0;}");
  page += F("label{display:flex;flex-direction:column;font-size:14px;gap:4px;}");
  page += F("input[type=text],input[type=password],input[type=number]{padding:8px;border:1px solid #c9d1d9;border-radius:8px;min-width:180px;}");
  page += F("button{padding:10px 14px;border:none;border-radius:8px;background:#0a7cff;color:#fff;cursor:pointer;}");
  page += F("button:hover{background:#0667d0;}code{background:#eef2f7;padding:2px 6px;border-radius:6px;}");
  page += F(".msg{padding:10px 12px;background:#ecfdf3;border:1px solid #9de3b0;border-radius:8px;margin-bottom:16px;}");
  page += F(".status{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px;}");
  page += F(".card{background:#f8fafc;border-radius:10px;padding:10px;}");
  page += F("</style></head><body>");

  page += F("<h1>ESP32 Mirror Ball Motor Controller</h1>");
  page += F("<p>Configure networking, MQTT and motor direction, then trigger motors directly from this page.</p>");
  page += F("<div class='msg'>");
  page += htmlEscape(getUiMessage());
  page += F("</div>");

  page += F("<section><h2>Status</h2><div class='status'>");
  page += F("<div class='card'><strong>WiFi STA</strong><br>");
  page += stateSnapshot.wifiConnected ? F("connected") : F("disconnected");
  page += F("<br><code>");
  page += ipAddressToString(stationIp);
  page += F("</code></div>");

  page += F("<div class='card'><strong>Fallback AP</strong><br>");
  page += fallbackApActive ? F("active") : F("inactive");
  page += F("<br><code>");
  page += ipAddressToString(apIp);
  page += F("</code></div>");

  page += F("<div class='card'><strong>MQTT</strong><br>");
  page += stateSnapshot.mqttConnected ? F("connected") : F("disconnected");
  page += F("<br><code>");
  page += htmlEscape(settingsSnapshot.mqttBrokerUri);
  page += F("</code></div>");

  page += F("<div class='card'><strong>VIN</strong><br>");
  page += String(stateSnapshot.vin, 2);
  page += F(" V</div>");

  page += F("<div class='card'><strong>Protection</strong><br>");
  page += settingsSnapshot.underVoltageProtectionEnabled ? F("undervoltage check on") : F("undervoltage check off");
  page += F("<br>Motor limit: ");
  page += String(settingsSnapshot.motorVoltageLimit, 2);
  page += F(" V</div>");
  page += F("</div></section>");

  page += F("<div class='grid'>");
  page += F("<section><h2>Network</h2>");
  page += F("<p>WiFi credentials stay in firmware. IP mode is configurable here.</p>");
  page += F("<p><strong>SSID:</strong> <code>");
  page += htmlEscape(LightControlConfig::WIFI_SSID);
  page += F("</code></p>");
  page += F("<form method='post' action='/save/network'>");
  page += F("<div class='row'><label><input type='checkbox' name='use_dhcp'");
  page += checkedAttribute(settingsSnapshot.useDhcp);
  page += F("> Use DHCP</label></div>");
  page += F("<div class='row'>");
  page += F("<label>Static IP<input type='text' name='static_ip' value='");
  page += ipAddressToString(settingsSnapshot.staticIp);
  page += F("'></label>");
  page += F("<label>Gateway<input type='text' name='gateway' value='");
  page += ipAddressToString(settingsSnapshot.gateway);
  page += F("'></label></div><div class='row'>");
  page += F("<label>Subnet<input type='text' name='subnet' value='");
  page += ipAddressToString(settingsSnapshot.subnet);
  page += F("'></label>");
  page += F("<label>DNS<input type='text' name='dns' value='");
  page += ipAddressToString(settingsSnapshot.dns);
  page += F("'></label></div><div class='row'><button type='submit'>Save Network</button></div></form></section>");

  page += F("<section><h2>MQTT</h2><form method='post' action='/save/mqtt'>");
  page += F("<div class='row'><label>Broker URI<input type='text' name='mqtt_uri' value='");
  page += htmlEscape(settingsSnapshot.mqttBrokerUri);
  page += F("'></label></div><div class='row'>");
  page += F("<label>Username<input type='text' name='mqtt_user' value='");
  page += htmlEscape(settingsSnapshot.mqttUsername);
  page += F("'></label>");
  page += F("<label>Password<input type='password' name='mqtt_pass' value='");
  page += htmlEscape(settingsSnapshot.mqttPassword);
  page += F("'></label></div><div class='row'><button type='submit'>Save MQTT</button></div></form></section>");
  page += F("</div>");

  page += F("<div class='grid'>");
  page += F("<section><h2>Motor Settings</h2><form method='post' action='/save/motor'>");
  page += F("<div class='row'><label><input type='checkbox' name='invert1'");
  page += checkedAttribute(settingsSnapshot.invertDirection[0]);
  page += F("> Invert Motor 1</label></div>");
  page += F("<div class='row'><label><input type='checkbox' name='invert2'");
  page += checkedAttribute(settingsSnapshot.invertDirection[1]);
  page += F("> Invert Motor 2</label></div>");
  page += F("<div class='row'><label><input type='checkbox' name='uv_check'");
  page += checkedAttribute(settingsSnapshot.underVoltageProtectionEnabled);
  page += F("> Enable undervoltage protection</label></div>");
  page += F("<div class='row'><label>Motor voltage limit (V)<input type='number' name='motor_voltage_limit' min='0' max='");
  page += String(LightControlConfig::MAX_MOTOR_VOLTAGE_LIMIT_V, 1);
  page += F("' step='0.1' value='");
  page += String(settingsSnapshot.motorVoltageLimit, 2);
  page += F("'></label></div>");
  page += F("<div class='row'><button type='submit'>Save Motor Settings</button></div></form></section>");

  page += F("<section><h2>Manual Control</h2>");
  for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
    float speedPercent = (LightControlConfig::MAX_COMMAND_SPEED_RAD_S > 0.0f)
                           ? (stateSnapshot.command[motorIndex].speedRadPerSec / LightControlConfig::MAX_COMMAND_SPEED_RAD_S) * 100.0f
                           : 0.0f;
    if (speedPercent <= 0.0f) {
      speedPercent = DEFAULT_MANUAL_SPEED_PERCENT;
    }
    page += renderManualForm(MOTOR_TOPICS[motorIndex].displayName, MOTOR_TOPICS[motorIndex].aliases[0], speedPercent);
  }
  page += renderManualForm("Both Motors", "all", DEFAULT_MANUAL_SPEED_PERCENT);
  page += F("</section></div>");

  page += F("<section><h2>Motor State</h2><div class='status'>");
  for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
    const int direction = effectiveMotorDirection(stateSnapshot.command[motorIndex], settingsSnapshot.invertDirection[motorIndex]);
    page += F("<div class='card'><strong>");
    page += htmlEscape(MOTOR_TOPICS[motorIndex].displayName);
    page += F("</strong><br>Enabled: ");
    page += stateSnapshot.command[motorIndex].enabled ? F("yes") : F("no");
    page += F("<br>Direction: ");
    page += (direction >= 0) ? F("cw") : F("ccw");
    page += F("<br>Target Speed: ");
    page += String(stateSnapshot.command[motorIndex].speedRadPerSec, 2);
    page += F(" rad/s<br>Current: ");
    page += String(stateSnapshot.currentVelocity[motorIndex], 2);
    page += F(" rad/s</div>");
  }
  page += F("</div></section>");

  page += F("<section><h2>MQTT Topics</h2><p>Commands stay under <code>");
  page += htmlEscape(LightControlConfig::MQTT_COMMAND_ROOT);
  page += F("</code>, for example:</p><p><code>");
  page += htmlEscape(LightControlConfig::MQTT_COMMAND_ROOT);
  page += F("/Motor 1/enable</code><br><code>");
  page += htmlEscape(LightControlConfig::MQTT_COMMAND_ROOT);
  page += F("/Motor 1/direction</code><br><code>");
  page += htmlEscape(LightControlConfig::MQTT_COMMAND_ROOT);
  page += F("/Motor 1/speed</code></p></section>");

  page += F("</body></html>");
  return page;
}

void handleRoot() {
  webServer.send(200, "text/html", renderRootPage());
}

void handleSaveNetwork() {
  RuntimeSettings settings;
  copyRuntimeSettings(settings);

  settings.useDhcp = webServer.hasArg("use_dhcp");
  if (!settings.useDhcp) {
    IPAddress parsedIp;
    IPAddress parsedGateway;
    IPAddress parsedSubnet;
    IPAddress parsedDns;

    if (!parseIpAddress(webServer.arg("static_ip").c_str(), parsedIp) ||
        !parseIpAddress(webServer.arg("gateway").c_str(), parsedGateway) ||
        !parseIpAddress(webServer.arg("subnet").c_str(), parsedSubnet) ||
        !parseIpAddress(webServer.arg("dns").c_str(), parsedDns)) {
      setUiMessage("Invalid static IP settings. Nothing was changed.");
      redirectToRoot();
      return;
    }

    settings.staticIp = parsedIp;
    settings.gateway = parsedGateway;
    settings.subnet = parsedSubnet;
    settings.dns = parsedDns;
  }

  saveRuntimeSettings(settings);
  applyRuntimeSettings(settings);
  setUiMessage("Network settings saved. Reconnecting WiFi.");
  requestWifiReconfigure();
  requestMqttReconnect();
  redirectToRoot();
}

void handleSaveMqtt() {
  RuntimeSettings settings;
  copyRuntimeSettings(settings);

  copyString(settings.mqttBrokerUri, sizeof(settings.mqttBrokerUri), webServer.arg("mqtt_uri").c_str());
  copyString(settings.mqttUsername, sizeof(settings.mqttUsername), webServer.arg("mqtt_user").c_str());
  copyString(settings.mqttPassword, sizeof(settings.mqttPassword), webServer.arg("mqtt_pass").c_str());

  saveRuntimeSettings(settings);
  applyRuntimeSettings(settings);
  setUiMessage("MQTT settings saved. Reconnecting MQTT.");
  requestMqttReconnect();
  redirectToRoot();
}

void handleSaveMotorSettings() {
  RuntimeSettings settings;
  copyRuntimeSettings(settings);

  settings.invertDirection[0] = webServer.hasArg("invert1");
  settings.invertDirection[1] = webServer.hasArg("invert2");
  settings.underVoltageProtectionEnabled = webServer.hasArg("uv_check");

  float motorVoltageLimit = settings.motorVoltageLimit;
  if (!parseFloatStrict(webServer.arg("motor_voltage_limit").c_str(), motorVoltageLimit) || motorVoltageLimit < 0.0f) {
    setUiMessage("Invalid motor voltage limit. Nothing was changed.");
    redirectToRoot();
    return;
  }
  settings.motorVoltageLimit = motorVoltageLimit;

  saveRuntimeSettings(settings);
  applyRuntimeSettings(settings);
  setUiMessage("Motor settings saved.");
  redirectToRoot();
}

void handleManualControl() {
  const String motorToken = webServer.arg("motor");
  const String action = webServer.arg("action");

  const int motorIndex = findMotorIndex(motorToken.c_str());
  if (motorIndex == -2) {
    setUiMessage("Manual control failed: unknown motor target.");
    redirectToRoot();
    return;
  }

  float speedPercent = DEFAULT_MANUAL_SPEED_PERCENT;
  if (webServer.hasArg("speed")) {
    float parsedSpeedPercent = DEFAULT_MANUAL_SPEED_PERCENT;
    if (parseFloatStrict(webServer.arg("speed").c_str(), parsedSpeedPercent)) {
      speedPercent = parsedSpeedPercent;
    }
  }
  speedPercent = fminf(fmaxf(speedPercent, 0.0f), 100.0f);

  if (action == "stop") {
    applyToTargets(motorIndex, applySpeedToMotor, 0.0f);
    applyToTargets(motorIndex, applyEnableToMotor, false);
    setUiMessage("Manual stop command sent.");
    Serial.printf("[web] manual stop: target=%s\n", motorToken.c_str());
    redirectToRoot();
    return;
  }

  if (action == "cw" || action == "ccw") {
    bool usedFallbackSpeed = false;
    if (speedPercent <= 0.0f) {
      speedPercent = DEFAULT_MANUAL_SPEED_PERCENT;
      usedFallbackSpeed = true;
    }

    const int direction = (action == "cw") ? 1 : -1;
    const float speedRadPerSec = percentToRadPerSec(speedPercent);

    applyToTargets(motorIndex, applyDirectionToMotor, direction);
    applyToTargets(motorIndex, applySpeedToMotor, speedRadPerSec);
    applyToTargets(motorIndex, applyEnableToMotor, true);
    if (usedFallbackSpeed) {
      setUiMessage("Manual run command sent. Speed was 0, so 50% was used.");
    } else {
      setUiMessage("Manual run command sent.");
    }
    Serial.printf("[web] manual run: target=%s action=%s speed=%.1f%% (%.2f rad/s)\n",
                  motorToken.c_str(),
                  action.c_str(),
                  speedPercent,
                  speedRadPerSec);
    redirectToRoot();
    return;
  }

  setUiMessage("Manual control failed: unsupported action.");
  redirectToRoot();
}

void ensureWebServerStarted() {
  if (webServerStarted) {
    return;
  }

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/save/network", HTTP_POST, handleSaveNetwork);
  webServer.on("/save/mqtt", HTTP_POST, handleSaveMqtt);
  webServer.on("/save/motor", HTTP_POST, handleSaveMotorSettings);
  webServer.on("/manual", HTTP_POST, handleManualControl);
  webServer.on("/favicon.ico", HTTP_GET, []() { webServer.send(204); });
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "/");
    webServer.send(302);
  });

  webServer.begin();
  webServerStarted = true;
  Serial.println("[web] server started on port 80");
}

}  // namespace

bool isFallbackApActive() {
  return fallbackApActive;
}

void startFallbackAp() {
  if (fallbackApActive) {
    return;
  }

  ensureWebServerStarted();

  char ssid[64];
  buildFallbackApSsid(ssid, sizeof(ssid));

  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  bool started = false;
  if (strlen(LightControlConfig::FALLBACK_AP_PASSWORD) >= 8) {
    started = WiFi.softAP(ssid, LightControlConfig::FALLBACK_AP_PASSWORD);
  } else {
    started = WiFi.softAP(ssid);
  }

  if (started) {
    fallbackApActive = true;
    Serial.printf("[web] fallback AP started: %s @ %s\n", ssid, WiFi.softAPIP().toString().c_str());
    setUiMessage("Fallback AP started because STA is not connected.");
  }
}

void stopFallbackAp() {
  if (!fallbackApActive) {
    return;
  }

  WiFi.softAPdisconnect(true);
  fallbackApActive = false;
  Serial.println("[web] fallback AP stopped");
}

void serviceWebInterface() {
  ensureWebServerStarted();
  webServer.handleClient();
}

void applyWifiSettings() {
  RuntimeSettings settings;
  copyRuntimeSettings(settings);

  const bool restoreFallbackAp = fallbackApActive;
  stopFallbackAp();

  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(LightControlConfig::WIFI_HOSTNAME);

  ensureWebServerStarted();

  if (restoreFallbackAp) {
    startFallbackAp();
  }

  if (settings.useDhcp) {
    IPAddress zero(0, 0, 0, 0);
    WiFi.config(zero, zero, zero, zero);
  } else if (!WiFi.config(settings.staticIp, settings.gateway, settings.subnet, settings.dns)) {
    Serial.println("[wifi] failed to apply static IP config");
  }

  wifiConnectStartedMs = millis();
  if (LightControlConfig::WIFI_SSID[0] != '\0') {
    WiFi.begin(LightControlConfig::WIFI_SSID, LightControlConfig::WIFI_PASSWORD);
    Serial.printf("[wifi] connecting to %s using %s\n",
                  LightControlConfig::WIFI_SSID,
                  settings.useDhcp ? "DHCP" : "static IP");
  } else {
    Serial.println("[wifi] WIFI_SSID is empty in include/LightControlConfig.h");
    startFallbackAp();
  }
}

void printWebHelp() {
  char ssid[64];
  buildFallbackApSsid(ssid, sizeof(ssid));
  Serial.println("Web UI:");
  Serial.println("  STA URL: http://<device-ip>/");
  Serial.printf("  Fallback AP SSID: %s\n", ssid);
}
