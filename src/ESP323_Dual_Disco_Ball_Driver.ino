#include <Arduino.h>
#include <Preferences.h>
#include <SimpleFOC.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ctype.h>
#include <math.h>
#include <mqtt_client.h>
#include <string.h>

#include "LightControlConfig.h"

namespace Pins {
static constexpr int MOTOR_1_U = 32;
static constexpr int MOTOR_1_V = 33;
static constexpr int MOTOR_1_W = 25;

static constexpr int MOTOR_2_U = 26;
static constexpr int MOTOR_2_V = 27;
static constexpr int MOTOR_2_W = 14;

static constexpr int DRIVER_ENABLE = 12;
static constexpr int VIN_SENSE = 13;
}  // namespace Pins

static constexpr size_t MOTOR_COUNT = 2;
static constexpr uint32_t MOTOR_TASK_IDLE_DELAY_MS = 1;
static constexpr uint32_t MOTOR_TASK_BUSY_YIELD_INTERVAL = 128;
static constexpr uint32_t BOARD_CHECK_INTERVAL_MS = 1000;
static constexpr size_t MAX_TOPIC_LENGTH = 192;
static constexpr size_t MAX_PAYLOAD_LENGTH = 256;
static constexpr size_t MAX_URI_LENGTH = 128;
static constexpr size_t MAX_USERNAME_LENGTH = 64;
static constexpr size_t MAX_PASSWORD_LENGTH = 64;
static constexpr size_t MAX_UI_MESSAGE_LENGTH = 160;
static constexpr int MQTT_QOS = 1;
static constexpr float DEFAULT_MANUAL_SPEED_PERCENT = 50.0f;

struct MotorTopicInfo {
  const char *displayName;
  const char *aliases[4];
};

static const MotorTopicInfo MOTOR_TOPICS[MOTOR_COUNT] = {
  {
    LightControlConfig::MOTOR_1_NAME,
    {"motor1", "Motor 1", "1", nullptr},
  },
  {
    LightControlConfig::MOTOR_2_NAME,
    {"motor2", "Motor 2", "2", nullptr},
  },
};

static const char *GLOBAL_ALIASES[] = {"all", "both", "motors", nullptr};

struct MotorCommand {
  bool enabled;
  int direction;
  float speedRadPerSec;
  uint32_t lastUpdateMs;
};

struct SharedState {
  MotorCommand command[MOTOR_COUNT];
  float currentVelocity[MOTOR_COUNT];
  float vin;
  bool underVoltage;
  bool mqttConnected;
  bool wifiConnected;
};

struct RuntimeSettings {
  bool useDhcp;
  IPAddress staticIp;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;
  char mqttBrokerUri[MAX_URI_LENGTH];
  char mqttUsername[MAX_USERNAME_LENGTH];
  char mqttPassword[MAX_PASSWORD_LENGTH];
  bool invertDirection[MOTOR_COUNT];
};

BLDCMotor motorA = BLDCMotor(7);
BLDCDriver3PWM driverA = BLDCDriver3PWM(Pins::MOTOR_1_U, Pins::MOTOR_1_V, Pins::MOTOR_1_W, Pins::DRIVER_ENABLE);

BLDCMotor motorB = BLDCMotor(7);
BLDCDriver3PWM driverB = BLDCDriver3PWM(Pins::MOTOR_2_U, Pins::MOTOR_2_V, Pins::MOTOR_2_W, Pins::DRIVER_ENABLE);

BLDCMotor *const MOTORS[MOTOR_COUNT] = {&motorA, &motorB};
BLDCDriver3PWM *const DRIVERS[MOTOR_COUNT] = {&driverA, &driverB};

TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t networkTaskHandle = nullptr;
esp_mqtt_client_handle_t mqttClient = nullptr;
WebServer webServer(80);
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE settingsMux = portMUX_INITIALIZER_UNLOCKED;

SharedState sharedState = {
  {
    {false, 1, 0.0f, 0},
    {false, 1, 0.0f, 0},
  },
  {0.0f, 0.0f},
  0.0f,
  false,
  false,
  false,
};

RuntimeSettings runtimeSettings = {};

bool driverOutputsEnabled = false;
bool mqttClientStarted = false;
bool webServerStarted = false;
bool fallbackApActive = false;
bool vinMonitoringAvailable = true;
volatile bool stateDirty = true;
volatile bool wifiReconfigureRequested = false;
volatile bool mqttReconnectRequested = false;
uint32_t wifiConnectStartedMs = 0;
char uiMessage[MAX_UI_MESSAGE_LENGTH] = "Web UI ready.";

bool equalsIgnoreCase(const char *lhs, const char *rhs) {
  if (lhs == nullptr || rhs == nullptr) {
    return false;
  }

  while (*lhs != '\0' && *rhs != '\0') {
    if (tolower(static_cast<unsigned char>(*lhs)) != tolower(static_cast<unsigned char>(*rhs))) {
      return false;
    }
    ++lhs;
    ++rhs;
  }

  return *lhs == '\0' && *rhs == '\0';
}

void copyString(char *destination, size_t destinationSize, const char *source) {
  if (destination == nullptr || destinationSize == 0) {
    return;
  }

  if (source == nullptr) {
    destination[0] = '\0';
    return;
  }

  strncpy(destination, source, destinationSize - 1);
  destination[destinationSize - 1] = '\0';
}

void trimInPlace(char *text) {
  if (text == nullptr) {
    return;
  }

  size_t length = strlen(text);
  size_t start = 0;
  while (start < length && isspace(static_cast<unsigned char>(text[start]))) {
    ++start;
  }

  size_t end = length;
  while (end > start && isspace(static_cast<unsigned char>(text[end - 1]))) {
    --end;
  }

  if (start > 0 && end > start) {
    memmove(text, text + start, end - start);
  } else if (start > 0 && end == start) {
    text[0] = '\0';
    return;
  }

  text[end - start] = '\0';
}

bool parseFloatStrict(const char *text, float &value) {
  if (text == nullptr) {
    return false;
  }

  char *endPtr = nullptr;
  value = strtof(text, &endPtr);
  if (endPtr == text) {
    return false;
  }

  while (*endPtr != '\0' && isspace(static_cast<unsigned char>(*endPtr))) {
    ++endPtr;
  }

  return *endPtr == '\0' && isfinite(value);
}

bool parseIpAddress(const char *text, IPAddress &address) {
  if (text == nullptr) {
    return false;
  }
  return address.fromString(text);
}

void ipAddressToCString(const IPAddress &address, char *buffer, size_t bufferSize) {
  if (buffer == nullptr || bufferSize == 0) {
    return;
  }
  snprintf(buffer, bufferSize, "%u.%u.%u.%u", address[0], address[1], address[2], address[3]);
}

String ipAddressToString(const IPAddress &address) {
  char buffer[20];
  ipAddressToCString(address, buffer, sizeof(buffer));
  return String(buffer);
}

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

bool parseBoolValue(const char *text, bool &value) {
  if (text == nullptr) {
    return false;
  }

  if (equalsIgnoreCase(text, "1") || equalsIgnoreCase(text, "true") || equalsIgnoreCase(text, "on") ||
      equalsIgnoreCase(text, "enable") || equalsIgnoreCase(text, "enabled") || equalsIgnoreCase(text, "yes")) {
    value = true;
    return true;
  }

  if (equalsIgnoreCase(text, "0") || equalsIgnoreCase(text, "false") || equalsIgnoreCase(text, "off") ||
      equalsIgnoreCase(text, "disable") || equalsIgnoreCase(text, "disabled") || equalsIgnoreCase(text, "no")) {
    value = false;
    return true;
  }

  float numeric = 0.0f;
  if (parseFloatStrict(text, numeric)) {
    value = fabsf(numeric) > 0.5f;
    return true;
  }

  return false;
}

bool parseDirectionValue(const char *text, int &direction) {
  if (text == nullptr) {
    return false;
  }

  if (equalsIgnoreCase(text, "cw") || equalsIgnoreCase(text, "forward") || equalsIgnoreCase(text, "fwd")) {
    direction = 1;
    return true;
  }

  if (equalsIgnoreCase(text, "ccw") || equalsIgnoreCase(text, "reverse") || equalsIgnoreCase(text, "rev")) {
    direction = -1;
    return true;
  }

  float numeric = 0.0f;
  if (!parseFloatStrict(text, numeric)) {
    return false;
  }

  if (numeric < 0.0f) {
    direction = -1;
    return true;
  }

  if (numeric > 100.0f) {
    direction = (numeric >= 127.5f) ? -1 : 1;
  } else {
    direction = (numeric >= 50.0f) ? -1 : 1;
  }

  return true;
}

float percentToRadPerSec(float percentOrDmx) {
  const float maxSpeed = LightControlConfig::MAX_COMMAND_SPEED_RAD_S;
  const float magnitude = fabsf(percentOrDmx);

  if (magnitude <= 1.0f) {
    return magnitude * maxSpeed;
  }

  if (magnitude <= 100.0f) {
    return (magnitude / 100.0f) * maxSpeed;
  }

  if (magnitude <= 255.0f) {
    return (magnitude / 255.0f) * maxSpeed;
  }

  return fminf(magnitude, maxSpeed);
}

bool parseSpeedValue(const char *text, bool absoluteMode, float &speedRadPerSec) {
  if (text == nullptr) {
    return false;
  }

  float numeric = 0.0f;
  if (!parseFloatStrict(text, numeric)) {
    return false;
  }

  speedRadPerSec = absoluteMode ? fabsf(numeric) : percentToRadPerSec(numeric);
  speedRadPerSec = fminf(speedRadPerSec, LightControlConfig::MOTOR_VELOCITY_LIMIT_RAD_S);
  return true;
}

const char *emptyToNull(const char *value) {
  if (value == nullptr || value[0] == '\0') {
    return nullptr;
  }
  return value;
}

void markStateDirty() {
  stateDirty = true;
}

void setUiMessage(const char *message) {
  copyString(uiMessage, sizeof(uiMessage), message);
}

bool tryGetVinVolt(float &vin) {
  if (!vinMonitoringAvailable) {
    SharedState snapshot;
    copySharedState(snapshot);
    vin = snapshot.vin;
    return false;
  }

  const uint32_t milliVolts = analogReadMilliVolts(Pins::VIN_SENSE);
  if (milliVolts == 0) {
    vinMonitoringAvailable = false;
    SharedState snapshot;
    copySharedState(snapshot);
    vin = snapshot.vin;
    Serial.println("[power] VIN monitoring disabled: GPIO13 is ADC2 and is unavailable while WiFi is active");
    return false;
  }

  vin = milliVolts * 8.5f / 1000.0f;
  return true;
}

float getVinVolt() {
  float vin = 0.0f;
  tryGetVinVolt(vin);
  return vin;
}

void copySharedState(SharedState &snapshot) {
  portENTER_CRITICAL(&stateMux);
  snapshot = sharedState;
  portEXIT_CRITICAL(&stateMux);
}

void updateSharedStateVin(float vin) {
  portENTER_CRITICAL(&stateMux);
  sharedState.vin = vin;
  portEXIT_CRITICAL(&stateMux);
}

void updateSharedCurrentVelocity(size_t motorIndex, float velocity) {
  portENTER_CRITICAL(&stateMux);
  sharedState.currentVelocity[motorIndex] = velocity;
  portEXIT_CRITICAL(&stateMux);
}

void updateWifiState(bool connected) {
  bool changed = false;
  portENTER_CRITICAL(&stateMux);
  if (sharedState.wifiConnected != connected) {
    sharedState.wifiConnected = connected;
    changed = true;
  }
  portEXIT_CRITICAL(&stateMux);
  if (changed) {
    markStateDirty();
  }
}

void updateMqttState(bool connected) {
  bool changed = false;
  portENTER_CRITICAL(&stateMux);
  if (sharedState.mqttConnected != connected) {
    sharedState.mqttConnected = connected;
    changed = true;
  }
  portEXIT_CRITICAL(&stateMux);
  if (changed) {
    markStateDirty();
  }
}

void updateUnderVoltageState(bool underVoltage) {
  bool changed = false;
  portENTER_CRITICAL(&stateMux);
  if (sharedState.underVoltage != underVoltage) {
    sharedState.underVoltage = underVoltage;
    changed = true;
  }
  portEXIT_CRITICAL(&stateMux);
  if (changed) {
    markStateDirty();
  }
}

void setDefaultRuntimeSettings(RuntimeSettings &settings) {
  settings.useDhcp = LightControlConfig::DEFAULT_USE_DHCP;
  parseIpAddress(LightControlConfig::DEFAULT_STATIC_IP, settings.staticIp);
  parseIpAddress(LightControlConfig::DEFAULT_STATIC_GATEWAY, settings.gateway);
  parseIpAddress(LightControlConfig::DEFAULT_STATIC_SUBNET, settings.subnet);
  parseIpAddress(LightControlConfig::DEFAULT_STATIC_DNS, settings.dns);
  copyString(settings.mqttBrokerUri, sizeof(settings.mqttBrokerUri), LightControlConfig::MQTT_BROKER_URI);
  copyString(settings.mqttUsername, sizeof(settings.mqttUsername), LightControlConfig::MQTT_USERNAME);
  copyString(settings.mqttPassword, sizeof(settings.mqttPassword), LightControlConfig::MQTT_PASSWORD);
  for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
    settings.invertDirection[motorIndex] = false;
  }
}

void copyRuntimeSettings(RuntimeSettings &snapshot) {
  portENTER_CRITICAL(&settingsMux);
  snapshot = runtimeSettings;
  portEXIT_CRITICAL(&settingsMux);
}

void applyRuntimeSettings(const RuntimeSettings &settings) {
  portENTER_CRITICAL(&settingsMux);
  runtimeSettings = settings;
  portEXIT_CRITICAL(&settingsMux);
  markStateDirty();
}

void saveRuntimeSettings(const RuntimeSettings &settings) {
  Preferences preferences;
  preferences.begin("motorcfg", false);
  preferences.putBool("dhcp", settings.useDhcp);

  char buffer[20];
  ipAddressToCString(settings.staticIp, buffer, sizeof(buffer));
  preferences.putString("ip", buffer);
  ipAddressToCString(settings.gateway, buffer, sizeof(buffer));
  preferences.putString("gw", buffer);
  ipAddressToCString(settings.subnet, buffer, sizeof(buffer));
  preferences.putString("mask", buffer);
  ipAddressToCString(settings.dns, buffer, sizeof(buffer));
  preferences.putString("dns", buffer);

  preferences.putString("mqtt_uri", settings.mqttBrokerUri);
  preferences.putString("mqtt_user", settings.mqttUsername);
  preferences.putString("mqtt_pass", settings.mqttPassword);
  preferences.putBool("inv1", settings.invertDirection[0]);
  preferences.putBool("inv2", settings.invertDirection[1]);
  preferences.end();
}

void loadRuntimeSettings() {
  RuntimeSettings loadedSettings;
  setDefaultRuntimeSettings(loadedSettings);

  Preferences preferences;
  preferences.begin("motorcfg", false);
  loadedSettings.useDhcp = preferences.getBool("dhcp", loadedSettings.useDhcp);

  String value = preferences.getString("ip", LightControlConfig::DEFAULT_STATIC_IP);
  parseIpAddress(value.c_str(), loadedSettings.staticIp);
  value = preferences.getString("gw", LightControlConfig::DEFAULT_STATIC_GATEWAY);
  parseIpAddress(value.c_str(), loadedSettings.gateway);
  value = preferences.getString("mask", LightControlConfig::DEFAULT_STATIC_SUBNET);
  parseIpAddress(value.c_str(), loadedSettings.subnet);
  value = preferences.getString("dns", LightControlConfig::DEFAULT_STATIC_DNS);
  parseIpAddress(value.c_str(), loadedSettings.dns);

  value = preferences.getString("mqtt_uri", LightControlConfig::MQTT_BROKER_URI);
  copyString(loadedSettings.mqttBrokerUri, sizeof(loadedSettings.mqttBrokerUri), value.c_str());
  value = preferences.getString("mqtt_user", LightControlConfig::MQTT_USERNAME);
  copyString(loadedSettings.mqttUsername, sizeof(loadedSettings.mqttUsername), value.c_str());
  value = preferences.getString("mqtt_pass", LightControlConfig::MQTT_PASSWORD);
  copyString(loadedSettings.mqttPassword, sizeof(loadedSettings.mqttPassword), value.c_str());

  loadedSettings.invertDirection[0] = preferences.getBool("inv1", false);
  loadedSettings.invertDirection[1] = preferences.getBool("inv2", false);
  preferences.end();

  applyRuntimeSettings(loadedSettings);
}

void requestWifiReconfigure() {
  wifiReconfigureRequested = true;
}

void requestMqttReconnect() {
  mqttReconnectRequested = true;
}

void applyEnableToMotor(size_t motorIndex, bool enabled) {
  portENTER_CRITICAL(&stateMux);
  sharedState.command[motorIndex].enabled = enabled;
  sharedState.command[motorIndex].lastUpdateMs = millis();
  portEXIT_CRITICAL(&stateMux);
  markStateDirty();
}

void applyDirectionToMotor(size_t motorIndex, int direction) {
  portENTER_CRITICAL(&stateMux);
  sharedState.command[motorIndex].direction = (direction >= 0) ? 1 : -1;
  sharedState.command[motorIndex].lastUpdateMs = millis();
  portEXIT_CRITICAL(&stateMux);
  markStateDirty();
}

void applySpeedToMotor(size_t motorIndex, float speedRadPerSec) {
  portENTER_CRITICAL(&stateMux);
  sharedState.command[motorIndex].speedRadPerSec = fminf(fabsf(speedRadPerSec), LightControlConfig::MOTOR_VELOCITY_LIMIT_RAD_S);
  sharedState.command[motorIndex].lastUpdateMs = millis();
  portEXIT_CRITICAL(&stateMux);
  markStateDirty();
}

void applyToTargets(int motorIndex, void (*fn)(size_t, bool), bool value) {
  if (motorIndex < 0) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      fn(i, value);
    }
    return;
  }
  fn(static_cast<size_t>(motorIndex), value);
}

void applyToTargets(int motorIndex, void (*fn)(size_t, int), int value) {
  if (motorIndex < 0) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      fn(i, value);
    }
    return;
  }
  fn(static_cast<size_t>(motorIndex), value);
}

void applyToTargets(int motorIndex, void (*fn)(size_t, float), float value) {
  if (motorIndex < 0) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      fn(i, value);
    }
    return;
  }
  fn(static_cast<size_t>(motorIndex), value);
}

int findMotorIndex(const char *token) {
  if (token == nullptr) {
    return -2;
  }

  for (const char *const *alias = GLOBAL_ALIASES; *alias != nullptr; ++alias) {
    if (equalsIgnoreCase(token, *alias)) {
      return -1;
    }
  }

  for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
    for (const char *const *alias = MOTOR_TOPICS[motorIndex].aliases; *alias != nullptr; ++alias) {
      if (equalsIgnoreCase(token, *alias)) {
        return static_cast<int>(motorIndex);
      }
    }
    if (equalsIgnoreCase(token, MOTOR_TOPICS[motorIndex].displayName)) {
      return static_cast<int>(motorIndex);
    }
  }

  return -2;
}

int effectiveMotorDirection(const MotorCommand &command, bool inverted) {
  int direction = (command.direction >= 0) ? 1 : -1;
  if (inverted) {
    direction *= -1;
  }
  return direction;
}

void publishAvailability(bool online);
void publishState();

void buildCommandTopic(char *buffer, size_t bufferSize, const char *target, const char *field) {
  snprintf(buffer, bufferSize, "%s/%s/%s", LightControlConfig::MQTT_COMMAND_ROOT, target, field);
}

void buildStatusTopic(char *buffer, size_t bufferSize, const char *suffix) {
  snprintf(buffer, bufferSize, "%s/status/%s/%s", LightControlConfig::MQTT_COMMAND_ROOT, LightControlConfig::MQTT_DEVICE_ID, suffix);
}

void subscribeTopic(const char *target, const char *field) {
  if (mqttClient == nullptr) {
    return;
  }

  char topic[MAX_TOPIC_LENGTH];
  buildCommandTopic(topic, sizeof(topic), target, field);
  esp_mqtt_client_subscribe(mqttClient, topic, MQTT_QOS);
}

void subscribeAllCommandTopics() {
  const char *fields[] = {
    "enable",
    "ENABLE",
    "direction",
    "DIRECTION",
    "dir",
    "speed",
    "SPEED",
    "speed_pct",
    "speed_percent",
    "speed_rad_s",
    "velocity",
    "set",
    "command",
    nullptr,
  };

  for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
    for (const char *const *alias = MOTOR_TOPICS[motorIndex].aliases; *alias != nullptr; ++alias) {
      for (const char **field = fields; *field != nullptr; ++field) {
        subscribeTopic(*alias, *field);
      }
    }

    for (const char **field = fields; *field != nullptr; ++field) {
      subscribeTopic(MOTOR_TOPICS[motorIndex].displayName, *field);
    }
  }

  for (const char *const *alias = GLOBAL_ALIASES; *alias != nullptr; ++alias) {
    for (const char **field = fields; *field != nullptr; ++field) {
      subscribeTopic(*alias, *field);
    }
  }
}

bool handleSingleFieldCommand(int motorIndex, const char *field, const char *payload) {
  if (field == nullptr || payload == nullptr) {
    return false;
  }

  if (equalsIgnoreCase(field, "enable")) {
    bool enabled = false;
    if (!parseBoolValue(payload, enabled)) {
      return false;
    }
    applyToTargets(motorIndex, applyEnableToMotor, enabled);
    return true;
  }

  if (equalsIgnoreCase(field, "direction") || equalsIgnoreCase(field, "dir")) {
    int direction = 1;
    if (!parseDirectionValue(payload, direction)) {
      return false;
    }
    applyToTargets(motorIndex, applyDirectionToMotor, direction);
    return true;
  }

  if (equalsIgnoreCase(field, "speed") || equalsIgnoreCase(field, "speed_pct") || equalsIgnoreCase(field, "speed_percent")) {
    float speedRadPerSec = 0.0f;
    if (!parseSpeedValue(payload, false, speedRadPerSec)) {
      return false;
    }
    applyToTargets(motorIndex, applySpeedToMotor, speedRadPerSec);
    return true;
  }

  if (equalsIgnoreCase(field, "speed_rad_s") || equalsIgnoreCase(field, "velocity")) {
    float speedRadPerSec = 0.0f;
    if (!parseSpeedValue(payload, true, speedRadPerSec)) {
      return false;
    }
    applyToTargets(motorIndex, applySpeedToMotor, speedRadPerSec);
    return true;
  }

  return false;
}

void normalizeCompositePayload(char *payload) {
  for (char *cursor = payload; *cursor != '\0'; ++cursor) {
    if (*cursor == '{' || *cursor == '}' || *cursor == '"' || *cursor == '\'') {
      *cursor = ' ';
    }
  }
}

bool handleCompositeCommand(int motorIndex, char *payload) {
  normalizeCompositePayload(payload);

  bool handled = false;
  char *savePtr = nullptr;
  for (char *token = strtok_r(payload, ",;", &savePtr); token != nullptr; token = strtok_r(nullptr, ",;", &savePtr)) {
    trimInPlace(token);
    if (*token == '\0') {
      continue;
    }

    char *separator = strchr(token, '=');
    if (separator == nullptr) {
      separator = strchr(token, ':');
    }

    if (separator == nullptr) {
      handled = handleSingleFieldCommand(motorIndex, "speed", token) || handled;
      continue;
    }

    *separator = '\0';
    char *key = token;
    char *value = separator + 1;
    trimInPlace(key);
    trimInPlace(value);

    if (handleSingleFieldCommand(motorIndex, key, value)) {
      handled = true;
    }
  }

  return handled;
}

void handleIncomingMqttMessage(const char *topic, const char *payload) {
  if (topic == nullptr || payload == nullptr) {
    return;
  }

  const size_t rootLength = strlen(LightControlConfig::MQTT_COMMAND_ROOT);
  if (strncmp(topic, LightControlConfig::MQTT_COMMAND_ROOT, rootLength) != 0) {
    return;
  }

  const char *subTopic = topic + rootLength;
  if (*subTopic == '/') {
    ++subTopic;
  }
  if (*subTopic == '\0') {
    return;
  }

  char workingTopic[MAX_TOPIC_LENGTH];
  strncpy(workingTopic, subTopic, sizeof(workingTopic) - 1);
  workingTopic[sizeof(workingTopic) - 1] = '\0';

  char *savePtr = nullptr;
  char *targetToken = strtok_r(workingTopic, "/", &savePtr);
  char *fieldToken = strtok_r(nullptr, "/", &savePtr);

  if (targetToken == nullptr || fieldToken == nullptr) {
    return;
  }

  const int motorIndex = findMotorIndex(targetToken);
  if (motorIndex == -2) {
    return;
  }

  char workingPayload[MAX_PAYLOAD_LENGTH];
  strncpy(workingPayload, payload, sizeof(workingPayload) - 1);
  workingPayload[sizeof(workingPayload) - 1] = '\0';
  trimInPlace(workingPayload);

  if (*workingPayload == '\0') {
    return;
  }

  if (equalsIgnoreCase(fieldToken, "set") || equalsIgnoreCase(fieldToken, "command")) {
    if (handleCompositeCommand(motorIndex, workingPayload)) {
      Serial.printf("[mqtt] %s <= %s\n", topic, payload);
    }
    return;
  }

  if (handleSingleFieldCommand(motorIndex, fieldToken, workingPayload)) {
    Serial.printf("[mqtt] %s <= %s\n", topic, payload);
  }
}

void mqttEventHandler(void *handlerArgs, esp_event_base_t base, int32_t eventId, void *eventData) {
  (void)handlerArgs;
  (void)base;

  auto *event = static_cast<esp_mqtt_event_handle_t>(eventData);
  switch (static_cast<esp_mqtt_event_id_t>(eventId)) {
    case MQTT_EVENT_CONNECTED:
      Serial.println("[mqtt] connected");
      updateMqttState(true);
      subscribeAllCommandTopics();
      publishAvailability(true);
      publishState();
      break;

    case MQTT_EVENT_DISCONNECTED:
      Serial.println("[mqtt] disconnected");
      updateMqttState(false);
      break;

    case MQTT_EVENT_DATA: {
      if (event->current_data_offset != 0 || event->total_data_len >= static_cast<int>(MAX_PAYLOAD_LENGTH) ||
          event->topic_len >= static_cast<int>(MAX_TOPIC_LENGTH)) {
        Serial.println("[mqtt] skipped fragmented or oversized message");
        return;
      }

      char topicBuffer[MAX_TOPIC_LENGTH];
      char payloadBuffer[MAX_PAYLOAD_LENGTH];

      memcpy(topicBuffer, event->topic, event->topic_len);
      topicBuffer[event->topic_len] = '\0';

      memcpy(payloadBuffer, event->data, event->data_len);
      payloadBuffer[event->data_len] = '\0';

      handleIncomingMqttMessage(topicBuffer, payloadBuffer);
      break;
    }

    default:
      break;
  }
}

void destroyMqttClient() {
  if (mqttClient == nullptr) {
    mqttClientStarted = false;
    updateMqttState(false);
    return;
  }

  if (mqttClientStarted) {
    esp_mqtt_client_stop(mqttClient);
  }

  esp_mqtt_client_destroy(mqttClient);
  mqttClient = nullptr;
  mqttClientStarted = false;
  updateMqttState(false);
}

void createMqttClient() {
  if (mqttClient != nullptr) {
    return;
  }

  RuntimeSettings settings;
  copyRuntimeSettings(settings);
  if (settings.mqttBrokerUri[0] == '\0') {
    Serial.println("[mqtt] broker URI is empty");
    return;
  }

  static char availabilityTopic[MAX_TOPIC_LENGTH];
  buildStatusTopic(availabilityTopic, sizeof(availabilityTopic), "availability");

  esp_mqtt_client_config_t mqttConfig = {};
  mqttConfig.broker.address.uri = settings.mqttBrokerUri;
  mqttConfig.credentials.username = emptyToNull(settings.mqttUsername);
  mqttConfig.credentials.client_id = LightControlConfig::MQTT_DEVICE_ID;
  mqttConfig.credentials.authentication.password = emptyToNull(settings.mqttPassword);
  mqttConfig.session.keepalive = 30;
  mqttConfig.session.last_will.topic = availabilityTopic;
  mqttConfig.session.last_will.msg = "offline";
  mqttConfig.session.last_will.msg_len = 7;
  mqttConfig.session.last_will.qos = MQTT_QOS;
  mqttConfig.session.last_will.retain = 1;
  mqttConfig.network.reconnect_timeout_ms = 5000;
  mqttConfig.network.timeout_ms = 5000;
  mqttConfig.task.priority = 3;
  mqttConfig.task.stack_size = 6144;
  mqttConfig.buffer.size = 1024;
  mqttConfig.buffer.out_size = 1024;

  mqttClient = esp_mqtt_client_init(&mqttConfig);
  if (mqttClient == nullptr) {
    Serial.println("[mqtt] init failed");
    return;
  }

  esp_mqtt_client_register_event(mqttClient, MQTT_EVENT_ANY, mqttEventHandler, nullptr);
}

void restartMqttClient() {
  destroyMqttClient();
  createMqttClient();
  if (mqttClient != nullptr) {
    esp_mqtt_client_start(mqttClient);
    mqttClientStarted = true;
  }
}

void publishAvailability(bool online) {
  if (mqttClient == nullptr || !sharedState.mqttConnected) {
    return;
  }

  char topic[MAX_TOPIC_LENGTH];
  buildStatusTopic(topic, sizeof(topic), "availability");
  esp_mqtt_client_publish(mqttClient, topic, online ? "online" : "offline", 0, MQTT_QOS, true);
}

void publishState() {
  if (mqttClient == nullptr) {
    return;
  }

  SharedState snapshot;
  copySharedState(snapshot);
  if (!snapshot.mqttConnected) {
    return;
  }

  RuntimeSettings settings;
  copyRuntimeSettings(settings);

  char topic[MAX_TOPIC_LENGTH];
  char payload[320];

  IPAddress stationIp = WiFi.localIP();
  IPAddress apIp = WiFi.softAPIP();
  snprintf(payload, sizeof(payload),
           "{\"wifi\":%s,\"mqtt\":%s,\"undervoltage\":%s,\"vin\":%.2f,"
           "\"dhcp\":%s,\"station_ip\":\"%u.%u.%u.%u\",\"ap_ip\":\"%u.%u.%u.%u\"}",
           snapshot.wifiConnected ? "true" : "false",
           snapshot.mqttConnected ? "true" : "false",
           snapshot.underVoltage ? "true" : "false",
           snapshot.vin,
           settings.useDhcp ? "true" : "false",
           stationIp[0], stationIp[1], stationIp[2], stationIp[3],
           apIp[0], apIp[1], apIp[2], apIp[3]);
  buildStatusTopic(topic, sizeof(topic), "device");
  esp_mqtt_client_publish(mqttClient, topic, payload, 0, MQTT_QOS, true);

  for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
    const int direction = effectiveMotorDirection(snapshot.command[motorIndex], settings.invertDirection[motorIndex]);
    const float signedTarget = snapshot.command[motorIndex].enabled ? direction * snapshot.command[motorIndex].speedRadPerSec : 0.0f;
    const float speedPercent = (LightControlConfig::MAX_COMMAND_SPEED_RAD_S > 0.0f)
                                 ? (snapshot.command[motorIndex].speedRadPerSec / LightControlConfig::MAX_COMMAND_SPEED_RAD_S) * 100.0f
                                 : 0.0f;

    snprintf(payload, sizeof(payload),
             "{\"name\":\"%s\",\"enabled\":%s,\"direction\":\"%s\","
             "\"inverted\":%s,\"speed_pct\":%.1f,\"target_rad_s\":%.3f,\"current_rad_s\":%.3f,"
             "\"age_ms\":%lu}",
             MOTOR_TOPICS[motorIndex].displayName,
             snapshot.command[motorIndex].enabled ? "true" : "false",
             (direction >= 0) ? "cw" : "ccw",
             settings.invertDirection[motorIndex] ? "true" : "false",
             speedPercent,
             signedTarget,
             snapshot.currentVelocity[motorIndex],
             static_cast<unsigned long>(millis() - snapshot.command[motorIndex].lastUpdateMs));

    snprintf(topic, sizeof(topic), "%s/status/%s/motor%u",
             LightControlConfig::MQTT_COMMAND_ROOT,
             LightControlConfig::MQTT_DEVICE_ID,
             static_cast<unsigned>(motorIndex + 1));
    esp_mqtt_client_publish(mqttClient, topic, payload, 0, MQTT_QOS, true);
  }
}

void buildFallbackApSsid(char *buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "%s-%s", LightControlConfig::FALLBACK_AP_SSID_PREFIX, LightControlConfig::MQTT_DEVICE_ID);
}

void startFallbackAp() {
  if (fallbackApActive) {
    return;
  }

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

String renderRootPage();

void redirectToRoot() {
  webServer.sendHeader("Location", "/");
  webServer.send(303);
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

void handleSaveInvert() {
  RuntimeSettings settings;
  copyRuntimeSettings(settings);

  settings.invertDirection[0] = webServer.hasArg("invert1");
  settings.invertDirection[1] = webServer.hasArg("invert2");

  saveRuntimeSettings(settings);
  applyRuntimeSettings(settings);
  setUiMessage("Direction inversion saved.");
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
  webServer.on("/save/invert", HTTP_POST, handleSaveInvert);
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
  } else {
    if (!WiFi.config(settings.staticIp, settings.gateway, settings.subnet, settings.dns)) {
      Serial.println("[wifi] failed to apply static IP config");
    }
  }

  if (LightControlConfig::WIFI_SSID[0] != '\0') {
    WiFi.begin(LightControlConfig::WIFI_SSID, LightControlConfig::WIFI_PASSWORD);
    wifiConnectStartedMs = millis();
    Serial.printf("[wifi] connecting to %s using %s\n",
                  LightControlConfig::WIFI_SSID,
                  settings.useDhcp ? "DHCP" : "static IP");
  } else {
    wifiConnectStartedMs = millis();
    Serial.println("[wifi] WIFI_SSID is empty in include/LightControlConfig.h");
    startFallbackAp();
  }
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
  page += htmlEscape(uiMessage);
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
  page += F("<section><h2>Direction Inversion</h2><form method='post' action='/save/invert'>");
  page += F("<div class='row'><label><input type='checkbox' name='invert1'");
  page += checkedAttribute(settingsSnapshot.invertDirection[0]);
  page += F("> Invert Motor 1</label></div>");
  page += F("<div class='row'><label><input type='checkbox' name='invert2'");
  page += checkedAttribute(settingsSnapshot.invertDirection[1]);
  page += F("> Invert Motor 2</label></div>");
  page += F("<div class='row'><button type='submit'>Save Inversion</button></div></form></section>");

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

void boardInit() {
  pinMode(Pins::MOTOR_1_U, INPUT_PULLUP);
  pinMode(Pins::MOTOR_1_V, INPUT_PULLUP);
  pinMode(Pins::MOTOR_1_W, INPUT_PULLUP);
  pinMode(Pins::MOTOR_2_U, INPUT_PULLUP);
  pinMode(Pins::MOTOR_2_V, INPUT_PULLUP);
  pinMode(Pins::MOTOR_2_W, INPUT_PULLUP);

  analogReadResolution(12);

  float vin = getVinVolt();
  updateSharedStateVin(vin);
  while (vin <= LightControlConfig::UNDERVOLTAGE_THRESHOLD_V) {
    Serial.printf("Waiting for power, VIN=%.2fV\n", vin);
    delay(500);
    vin = getVinVolt();
    updateSharedStateVin(vin);
  }

  Serial.printf("Power ready, VIN=%.2fV\n", vin);
}

void initMotors() {
  const float supplyVoltage = getVinVolt();
  updateSharedStateVin(supplyVoltage);

  for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
    DRIVERS[motorIndex]->voltage_power_supply = supplyVoltage;
    DRIVERS[motorIndex]->init();

    MOTORS[motorIndex]->linkDriver(DRIVERS[motorIndex]);
    MOTORS[motorIndex]->voltage_limit = LightControlConfig::MOTOR_VOLTAGE_LIMIT_V;
    MOTORS[motorIndex]->velocity_limit = LightControlConfig::MOTOR_VELOCITY_LIMIT_RAD_S;
    MOTORS[motorIndex]->controller = MotionControlType::velocity_openloop;
    MOTORS[motorIndex]->init();
    MOTORS[motorIndex]->disable();
  }
}

void updateDriverEnableState(bool shouldEnable) {
  if (shouldEnable == driverOutputsEnabled) {
    return;
  }

  if (shouldEnable) {
    for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
      MOTORS[motorIndex]->enable();
    }
    Serial.println("[motor] outputs enabled");
  } else {
    for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
      MOTORS[motorIndex]->disable();
    }
    Serial.println("[motor] outputs disabled");
  }

  driverOutputsEnabled = shouldEnable;
}

void boardCheck() {
  static uint32_t lastCheckMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastCheckMs < BOARD_CHECK_INTERVAL_MS) {
    return;
  }
  lastCheckMs = nowMs;

  float vin = 0.0f;
  if (!tryGetVinVolt(vin)) {
    return;
  }
  updateSharedStateVin(vin);

  const bool undervoltage = vin < LightControlConfig::UNDERVOLTAGE_THRESHOLD_V;
  updateUnderVoltageState(undervoltage);
  if (undervoltage) {
    Serial.printf("[power] undervoltage: %.2fV\n", vin);
  }
}

void motorTask(void *parameter) {
  (void)parameter;
  uint32_t busyLoopCounter = 0;

  for (;;) {
    boardCheck();

    SharedState snapshot;
    RuntimeSettings settings;
    float targetVelocity[MOTOR_COUNT] = {0.0f, 0.0f};
    copySharedState(snapshot);
    copyRuntimeSettings(settings);

    bool shouldEnableOutputs = false;
    for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
      const MotorCommand &command = snapshot.command[motorIndex];

      bool commandTimedOut = false;
      if (LightControlConfig::COMMAND_TIMEOUT_MS > 0 && command.lastUpdateMs > 0) {
        commandTimedOut = (millis() - command.lastUpdateMs) > LightControlConfig::COMMAND_TIMEOUT_MS;
      }

      const bool communicationHealthy = snapshot.mqttConnected || !LightControlConfig::STOP_MOTORS_WHEN_MQTT_DISCONNECTS;
      if (!snapshot.underVoltage && communicationHealthy && !commandTimedOut && command.enabled) {
        const float direction = static_cast<float>(effectiveMotorDirection(command, settings.invertDirection[motorIndex]));
        targetVelocity[motorIndex] = direction * command.speedRadPerSec;
        shouldEnableOutputs = true;
      }

      if (fabsf(targetVelocity[motorIndex]) > 0.001f) {
        shouldEnableOutputs = true;
      }
    }

    updateDriverEnableState(shouldEnableOutputs);

    for (size_t motorIndex = 0; motorIndex < MOTOR_COUNT; ++motorIndex) {
      updateSharedCurrentVelocity(motorIndex, targetVelocity[motorIndex]);
      MOTORS[motorIndex]->move(targetVelocity[motorIndex]);
    }

    if (!shouldEnableOutputs) {
      busyLoopCounter = 0;
      vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_IDLE_DELAY_MS));
      continue;
    }

    ++busyLoopCounter;
    if (busyLoopCounter >= MOTOR_TASK_BUSY_YIELD_INTERVAL) {
      busyLoopCounter = 0;
      vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_IDLE_DELAY_MS));
    }
  }
}

void networkTask(void *parameter) {
  (void)parameter;

  applyWifiSettings();
  restartMqttClient();

  uint32_t lastWifiRetryMs = 0;
  uint32_t lastStatusPublishMs = 0;

  for (;;) {
    webServer.handleClient();

    if (wifiReconfigureRequested) {
      wifiReconfigureRequested = false;
      applyWifiSettings();
      lastWifiRetryMs = millis();
    }

    if (mqttReconnectRequested) {
      mqttReconnectRequested = false;
      restartMqttClient();
    }

    const bool wifiConnected = WiFi.status() == WL_CONNECTED;
    updateWifiState(wifiConnected);

    if (wifiConnected) {
      wifiConnectStartedMs = millis();
      if (fallbackApActive) {
        stopFallbackAp();
      }
    } else {
      const uint32_t nowMs = millis();
      if (nowMs - lastWifiRetryMs >= LightControlConfig::WIFI_RETRY_INTERVAL_MS && LightControlConfig::WIFI_SSID[0] != '\0') {
        lastWifiRetryMs = nowMs;
        WiFi.reconnect();
        Serial.println("[wifi] reconnect requested");
      }

      if (!fallbackApActive && (nowMs - wifiConnectStartedMs >= LightControlConfig::FALLBACK_AP_DELAY_MS)) {
        startFallbackAp();
      }
    }

    const uint32_t nowMs = millis();
    if (stateDirty || (nowMs - lastStatusPublishMs >= LightControlConfig::STATUS_PUBLISH_INTERVAL_MS)) {
      publishState();
      stateDirty = false;
      lastStatusPublishMs = nowMs;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void printTopicHelp() {
  Serial.println("MQTT command examples:");
  Serial.printf("  %s/%s/enable -> on\n", LightControlConfig::MQTT_COMMAND_ROOT, MOTOR_TOPICS[0].displayName);
  Serial.printf("  %s/%s/direction -> cw | ccw\n", LightControlConfig::MQTT_COMMAND_ROOT, MOTOR_TOPICS[0].displayName);
  Serial.printf("  %s/%s/speed -> 0..100\n", LightControlConfig::MQTT_COMMAND_ROOT, MOTOR_TOPICS[0].displayName);
  Serial.printf("  %s/%s/speed_rad_s -> 0..10\n", LightControlConfig::MQTT_COMMAND_ROOT, MOTOR_TOPICS[0].displayName);
  Serial.printf("  %s/all/set -> {\"enable\":true,\"direction\":\"cw\",\"speed\":35}\n", LightControlConfig::MQTT_COMMAND_ROOT);
}

void printWebHelp() {
  char ssid[64];
  buildFallbackApSsid(ssid, sizeof(ssid));
  Serial.println("Web UI:");
  Serial.println("  STA URL: http://<device-ip>/");
  Serial.printf("  Fallback AP SSID: %s\n", ssid);
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("ESP32 dual motor MQTT controller");
  Serial.printf("Arduino loop core: %d\n", ARDUINO_RUNNING_CORE);

  loadRuntimeSettings();
  boardInit();
  initMotors();
  printTopicHelp();
  printWebHelp();

  xTaskCreatePinnedToCore(motorTask, "motorTask", 8192, nullptr, 3, &motorTaskHandle, 1);
  xTaskCreatePinnedToCore(networkTask, "networkTask", 12288, nullptr, 2, &networkTaskHandle, 0);

  Serial.println("Tasks started");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
