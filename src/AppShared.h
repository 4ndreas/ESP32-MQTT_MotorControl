#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <stddef.h>
#include <stdint.h>

static constexpr size_t MOTOR_COUNT = 2;
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
  bool underVoltageProtectionEnabled;
  float motorVoltageLimit;
};

extern const MotorTopicInfo MOTOR_TOPICS[MOTOR_COUNT];
extern const char *const GLOBAL_ALIASES[];
extern uint32_t wifiConnectStartedMs;

bool equalsIgnoreCase(const char *lhs, const char *rhs);
void copyString(char *destination, size_t destinationSize, const char *source);
void trimInPlace(char *text);
bool parseFloatStrict(const char *text, float &value);
bool parseIpAddress(const char *text, IPAddress &address);
void ipAddressToCString(const IPAddress &address, char *buffer, size_t bufferSize);
String ipAddressToString(const IPAddress &address);
bool parseBoolValue(const char *text, bool &value);
bool parseDirectionValue(const char *text, int &direction);
float percentToRadPerSec(float percentOrDmx);
bool parseSpeedValue(const char *text, bool absoluteMode, float &speedRadPerSec);
void copySharedState(SharedState &snapshot);
void copyRuntimeSettings(RuntimeSettings &snapshot);
void saveRuntimeSettings(const RuntimeSettings &settings);
void applyRuntimeSettings(const RuntimeSettings &settings);
void requestWifiReconfigure();
void requestMqttReconnect();
void applyEnableToMotor(size_t motorIndex, bool enabled);
void applyDirectionToMotor(size_t motorIndex, int direction);
void applySpeedToMotor(size_t motorIndex, float speedRadPerSec);
void applyToTargets(int motorIndex, void (*fn)(size_t, bool), bool value);
void applyToTargets(int motorIndex, void (*fn)(size_t, int), int value);
void applyToTargets(int motorIndex, void (*fn)(size_t, float), float value);
int findMotorIndex(const char *token);
int effectiveMotorDirection(const MotorCommand &command, bool inverted);
void setUiMessage(const char *message);
const char *getUiMessage();
