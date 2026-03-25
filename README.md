# ESP32 Mirror Ball Motor Controller

This project runs two mirror ball motors on an ESP32-based MKS ESP32 FOC board and exposes control over WiFi using MQTT.

It is designed to work with the `lightcontrol` system by following the same motor attribute model:

- `ENABLE`
- `DIRECTION`
- `SPEED`

The firmware uses two cores on the ESP32:

- Core 1: fast motor control loop
- Core 0: WiFi, MQTT, reconnect handling, status publishing

## What It Does

- Controls 2 BLDC motors with SimpleFOC in open-loop velocity mode
- Connects to WiFi
- Hosts a small web UI for configuration and manual control
- Subscribes to MQTT commands for each motor
- Publishes device and motor status back to MQTT
- Stores runtime settings in NVS so they survive reboot
- Keeps undervoltage protection from the original board example

## Files

- [src/ESP323_Dual_Disco_Ball_Driver.ino](c:\Users\ahoel\Documents\PlatformIO\Projects\260320-103809-esp-wrover-kit\src\ESP323_Dual_Disco_Ball_Driver.ino): main firmware
- [include/LightControlConfig.h](c:\Users\ahoel\Documents\PlatformIO\Projects\260320-103809-esp-wrover-kit\include\LightControlConfig.h): WiFi, MQTT, topic, and motor config
- [platformio.ini](c:\Users\ahoel\Documents\PlatformIO\Projects\260320-103809-esp-wrover-kit\platformio.ini): PlatformIO build config

## Hardware

Current pin mapping matches the existing MKS ESP32 FOC board example:

- Motor 1 PWM: `32`, `33`, `25`
- Motor 2 PWM: `26`, `27`, `14`
- Driver enable: `12`
- VIN sense: `13`

### Important ESP32 ADC Note

On the classic ESP32, GPIO13 is an `ADC2` pin. `ADC2` cannot be sampled reliably while WiFi is active, so live VIN measurement stops working after the board joins WiFi.

The firmware handles this gracefully:

- it keeps the last valid VIN value instead of treating a failed ADC read as `0.00V`
- it does not trigger a false undervoltage shutdown when WiFi starts

If you need continuous VIN monitoring while WiFi is running, move the VIN sense signal to an `ADC1`-capable pin and update the firmware pin mapping.

## Configuration

Edit [include/LightControlConfig.h](c:\Users\ahoel\Documents\PlatformIO\Projects\260320-103809-esp-wrover-kit\include\LightControlConfig.h) before the first flash.

Set at least:

```cpp
static const char WIFI_SSID[] = "your-wifi";
static const char WIFI_PASSWORD[] = "your-password";
static const char MQTT_BROKER_URI[] = "mqtt://192.168.1.10:1883";
```

Optional but useful:

```cpp
static const char MQTT_USERNAME[] = "";
static const char MQTT_PASSWORD[] = "";
static const char MQTT_COMMAND_ROOT[] = "lightcontrol/motors";
static const char MQTT_DEVICE_ID[] = "esp32-mirrorball";
static const char MOTOR_1_NAME[] = "Motor 1";
static const char MOTOR_2_NAME[] = "Motor 2";
```

Default web-configurable network settings:

```cpp
static constexpr bool DEFAULT_USE_DHCP = true;
static const char DEFAULT_STATIC_IP[] = "192.168.1.90";
static const char DEFAULT_STATIC_GATEWAY[] = "192.168.1.1";
static const char DEFAULT_STATIC_SUBNET[] = "255.255.255.0";
static const char DEFAULT_STATIC_DNS[] = "192.168.1.1";
```

Fallback web AP settings:

```cpp
static const char FALLBACK_AP_SSID_PREFIX[] = "mirrorball-setup";
static const char FALLBACK_AP_PASSWORD[] = "mirrorball-setup";
static constexpr uint32_t FALLBACK_AP_DELAY_MS = 15000;
```

You can also tune limits:

- `MOTOR_VOLTAGE_LIMIT_V`
- `MOTOR_VELOCITY_LIMIT_RAD_S`
- `MAX_COMMAND_SPEED_RAD_S`
- `VELOCITY_RAMP_RAD_S2`
- `UNDERVOLTAGE_THRESHOLD_V`

## Build

From this project directory:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

If `platformio` or `pio` is already on your `PATH`, this also works:

```powershell
platformio run
```

## Flash

Upload with:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -t upload
```

Monitor serial output with:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -b 115200
```

## Web UI

After the board joins WiFi, open:

```text
http://<device-ip>/
```

The web page lets you:

- switch between DHCP and static IP
- change MQTT broker URI, username, and password
- invert motor 1 and motor 2 direction independently
- run each motor from buttons with a speed field
- stop one motor or both motors directly from the page

Settings changed in the browser are stored in NVS and survive reboot.

### Fallback AP

If the station side does not connect, the firmware starts a fallback AP after `FALLBACK_AP_DELAY_MS`.

Default SSID pattern:

```text
mirrorball-setup-esp32-mirrorball
```

Then open:

```text
http://192.168.4.1/
```

This is useful if the static IP is wrong or the normal WiFi network is unavailable.

## MQTT Command Interface

Default topic root:

```text
lightcontrol/motors
```

### Per-Motor Commands

Examples for motor 1:

```text
lightcontrol/motors/Motor 1/enable
lightcontrol/motors/Motor 1/direction
lightcontrol/motors/Motor 1/speed
lightcontrol/motors/Motor 1/speed_rad_s
```

Supported aliases:

- `Motor 1`
- `Motor 2`
- `motor1`
- `motor2`
- `1`
- `2`
- `all`
- `both`
- `motors`

### Payloads

`enable`

- `on`
- `off`
- `true`
- `false`
- `1`
- `0`

`direction`

- `cw`
- `ccw`
- `forward`
- `reverse`

`speed`

- `0..100` percent
- `0..255` DMX-style value

`speed_rad_s`

- absolute speed in rad/s

### Composite Command

You can set multiple values at once:

Topic:

```text
lightcontrol/motors/all/set
```

Payload examples:

```json
{"enable":true,"direction":"cw","speed":35}
```

```text
enable=on,direction=ccw,speed=60
```

## Status Topics

The firmware publishes retained status messages here:

```text
lightcontrol/motors/status/esp32-mirrorball/availability
lightcontrol/motors/status/esp32-mirrorball/device
lightcontrol/motors/status/esp32-mirrorball/motor1
lightcontrol/motors/status/esp32-mirrorball/motor2
```

`availability` is:

- `online`
- `offline`

`device` includes:

- WiFi state
- MQTT state
- undervoltage state
- VIN
- DHCP/static mode
- station IP
- fallback AP IP

Each motor status includes:

- `enabled`
- `direction`
- `inverted`
- `speed_pct`
- `target_rad_s`
- `current_rad_s`
- `age_ms`

## lightcontrol Integration Notes

The `lightcontrol` repo already defines a mirror ball motor fixture with:

- `ENABLE`
- `DIRECTION`
- `SPEED`

This firmware mirrors that model.

The exact MQTT topic naming used inside `PyMQTTBridges` was not directly visible in the repo, so this project uses a clear configurable MQTT topic layout instead. If needed, adapt either:

- `include/LightControlConfig.h` topic root and device names
- the `lightcontrol` side to publish to these topics

## Safety Notes

- Open-loop motor control can heat motors and drivers quickly
- Start with low `MOTOR_VOLTAGE_LIMIT_V`
- Check motor pole-pair count in the firmware if your motors differ
- Make sure the undervoltage threshold fits your power supply
- Test with the motor unloaded first

## Troubleshooting

No movement:

- Check `WIFI_SSID`, broker URI, and power supply
- Watch serial monitor for WiFi and MQTT connection messages
- Confirm MQTT topic names match your published commands
- Start with `speed=10`

Wrong direction:

- Use the web UI inversion checkboxes for motor 1 or motor 2
- Or send `cw` / `ccw` over MQTT if you just want a different command direction

Motors stop immediately:

- Check undervoltage on the serial monitor
- Check `COMMAND_TIMEOUT_MS`
- Check `STOP_MOTORS_WHEN_MQTT_DISCONNECTS`

Web UI not reachable:

- Check the serial monitor for the station IP
- If WiFi is not connected, join the fallback AP and open `http://192.168.4.1/`
- If you enabled a static IP, make sure it is in the correct subnet

Build issues:

- Use the local PlatformIO executable from `C:\Users\ahoel\.platformio\penv\Scripts\platformio.exe`
- If your global PlatformIO state is permission-locked, run from PlatformIO IDE or fix permissions in `C:\Users\ahoel\.platformio`
