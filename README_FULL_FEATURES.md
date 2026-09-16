# SAR Drone ESP32 – Full Feature Integration (Bench-Safe)

This version integrates the requested SAR software features into the supplied ESP32 38-pin controller while retaining the original hard safety lock:

`BENCH_TEST_MODE = true`

Therefore the controller computes motor outputs, but the physical ESC commands remain at minimum. This is intentional: the new navigation, battery, thermal, GSM and landing logic has not been flight-validated.

## Integrated features

- MPU6050 calibration
- Complementary roll/pitch attitude filter and gyro yaw integration
- Roll/pitch/yaw-rate PID framework
- Quad-X motor mixer
- BMP280 relative altitude and vertical-rate filtering
- Altitude PID framework
- GPS NMEA checksum parsing, EMA filtering and stale-fix detection
- Home lock
- Manual waypoint
- Lawn-mower survey grid
- RTH state/framework
- Landing state machine (bench-safe state only)
- GPS polygon logging and shoelace area calculation
- Camera/IMU tilt-corrected ground-coordinate estimate
- OV7670 visible candidate processing
- MLX90640 thermal frame acquisition and hotspot analysis
- Visible + thermal multi-frame confirmation
- Duplicate suppression by distance/time
- Evidence records with GPS, altitude, attitude and thermal data
- SIM800L SMS queue and retry logic
- Rescue SMS containing estimated coordinates and Google Maps link
- Low/critical battery handling through INA219 over I2C
- GPS-loss failsafe state
- IMU/BMP280/thermal/battery health tracking
- Excessive-tilt protection
- Mission state machine
- Serial telemetry
- UDP telemetry
- Serial command interface

## Additional hardware required

### I2C bus
- MPU6050: 0x68
- BMP280: 0x76 or 0x77
- MLX90640: 0x33
- INA219: 0x40

All four devices share GPIO21 SDA and GPIO22 SCL. Use appropriate pull-ups and common ground.

### SIM800L
- ESP32 GPIO12 = RX from SIM800L TX
- ESP32 GPIO2 = TX to SIM800L RX

GPIO2 is a boot-strapping pin. Validate the module's startup behavior before permanently wiring it. SIM800L must have a suitable separate power supply with adequate peak current and common ground.

### Battery monitoring
The integrated code expects an INA219 at 0x40 and uses its bus-voltage register. The default thresholds are intended for a 3-cell pack. Change the thresholds for the actual battery chemistry and cell count after measuring the real pack.

### Camera
The original OV7670-style parallel camera pin mapping is retained from the supplied project. The camera must actually deliver frames in a format supported by the installed `esp32-camera` driver; a generic bare OV7670 cannot always be initialized by `esp_camera.h` without a compatible driver/configuration.

## Libraries

Install:
- Adafruit BMP280 Library
- Adafruit MLX90640 Library
- ESP32 Arduino core with `esp_camera.h`, WiFi, ArduinoOTA and LEDC support

## Important limitations

1. The code is bench-safe and keeps physical ESC output locked.
2. BMP280 alone is not a reliable landing/altitude-hold sensor for a drone. The landing state is a software framework and does not claim safe autonomous landing.
3. Camera/thermal coordinates are estimates. Accurate survivor coordinates require a calibrated camera model and a reliable range/terrain-height measurement.
4. OV7670 processing is heuristic and does not identify a human.
5. GPS coordinates in an alert are observation/estimated coordinates, not a guarantee of survivor position.
6. SIM800L cellular coverage and power integrity are external dependencies.
7. PID gains are starting values only and must not be assumed safe for a particular airframe.
8. Do not remove the bench lock merely because compilation succeeds. Validate sensors, signs, mixer order, failsafes and the complete airframe independently.
