#pragma once

namespace LightControlConfig {

// Fill these in for your WiFi before flashing.
static const char WIFI_SSID[] = "Luftnetz_2GHz";
static const char WIFI_PASSWORD[] = "achtung!warnschuss!";
static const char WIFI_HOSTNAME[] = "esp32-mirrorball1";

// MQTT defaults. These can later be changed in the web UI and are stored in NVS.
static const char MQTT_BROKER_URI[] = "mqtt://192.168.2.10:1883";
static const char MQTT_USERNAME[] = "";
static const char MQTT_PASSWORD[] = "";

// Commands are expected below this root, for example:
//   lightcontrol/motors/Motor 1/enable     -> on/off, true/false, 0/1
//   lightcontrol/motors/Motor 1/direction  -> cw/ccw, forward/reverse
//   lightcontrol/motors/Motor 1/speed      -> 0..100 percent or 0..255 DMX
//   lightcontrol/motors/Motor 1/speed_rad_s -> absolute rad/s
//   lightcontrol/motors/all/set            -> {"enable":true,"direction":"cw","speed":35}
static const char MQTT_COMMAND_ROOT[] = "lightcontrol/motors";
static const char MQTT_DEVICE_ID[] = "esp32-mirrorball-1";

static const char MOTOR_1_NAME[] = "Motor 1";
static const char MOTOR_2_NAME[] = "Motor 2";

// Network defaults for the web-configurable station interface.
static constexpr bool DEFAULT_USE_DHCP = true;
static const char DEFAULT_STATIC_IP[] = "10.39.0.152";
static const char DEFAULT_STATIC_GATEWAY[] = "192.168.2.1";
static const char DEFAULT_STATIC_SUBNET[] = "255.255.0.0";
static const char DEFAULT_STATIC_DNS[] = "192.168.2.1";

// If the station side cannot connect, a small rescue AP is started for the web UI.
static const char FALLBACK_AP_SSID_PREFIX[] = "mirrorball-setup";
static const char FALLBACK_AP_PASSWORD[] = "mirrorball-setup";
static constexpr uint32_t FALLBACK_AP_DELAY_MS = 15000;

static constexpr float UNDERVOLTAGE_THRESHOLD_V = 11.1f;
static constexpr float MOTOR_VOLTAGE_LIMIT_V = 1.5f;
static constexpr float MOTOR_VELOCITY_LIMIT_RAD_S = 50.0f;
static constexpr float MAX_COMMAND_SPEED_RAD_S = 10.0f;
static constexpr float VELOCITY_RAMP_RAD_S2 = 8.0f;

static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 5000;
static constexpr uint32_t STATUS_PUBLISH_INTERVAL_MS = 5000;

// Set to a value > 0 if you want commands to expire after some time.
static constexpr uint32_t COMMAND_TIMEOUT_MS = 0;
static constexpr bool STOP_MOTORS_WHEN_MQTT_DISCONNECTS = false;

}  // namespace LightControlConfig
