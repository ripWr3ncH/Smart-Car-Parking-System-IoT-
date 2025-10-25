## Smart Car Parking System (IoT)

![Project Banner](./images/banner.png)

A compact IoT-enabled smart parking system implemented with Arduino-compatible microcontrollers and camera support. This repository contains working sketches to detect parked cars, report slot status, and (optionally) stream or send images from a camera module.

---

## Repository contents

- `car_parking_with_3_slots.ino` — Main sketch that manages detection for three parking slots and reports their occupancy status.
- `send_data_cam.ino` — Sketch that captures and sends camera data (e.g., using an ESP32-CAM or similar) to a server or endpoint.
- `LICENSE` — Project license (see this file for license terms).

> Note: filenames and behavior above reflect the sketches included in this repository. If you rename or add sketches, update this README accordingly.

## Features

- Real-time detection of up to 3 parking slots
- Local indicator (LEDs/buzzer) support for slot status
- Wi‑Fi-enabled data reporting (for boards like ESP8266/ESP32)
- Camera capture and data send (example sketch included)
- Lightweight and easy-to-extend Arduino sketches

## Hardware (typical)

> The exact hardware used may vary. Update this list to reflect your physical setup.

- Arduino Uno / Nano, or ESP8266 / ESP32 / ESP32-CAM (for Wi‑Fi/camera features)
- Ultrasonic distance sensors (HC-SR04) or IR/IR-reflective sensors — one per slot
- Jumper wires, breadboard
- LEDs and resistors for visual indicators (optional)
- Buzzer (optional)
- Power supply (5V/3.3V depending on board)
- Camera module (e.g., ESP32-CAM) if using `send_data_cam.ino`

## Software prerequisites

- Arduino IDE (recommended) or PlatformIO
- If using Wi‑Fi/camera features: install board support for ESP8266/ESP32
- Required Arduino libraries (see top of each sketch for #include lines). Common libraries used include `WiFi.h`, `ESPAsyncWebServer`, `ArduinoJson`, or sensor-specific libraries. Install via Library Manager or PlatformIO.

## Quick start — Arduino IDE

1. Open the Arduino IDE.
2. Go to File → Open and open `car_parking_with_3_slots.ino` (or `send_data_cam.ino`).
3. Select the correct board and port: Tools → Board, Tools → Port.
4. If required, edit the configuration constants at the top of the sketch (Wi‑Fi SSID, password, pins).
5. Click Upload.

## Quick start — Arduino CLI (PowerShell)

If you prefer `arduino-cli`, here are example PowerShell commands (replace placeholders):

```powershell
# Compile (example for Arduino Uno)
arduino-cli compile --fqbn arduino:avr:uno "car_parking_with_3_slots"

# Upload (replace COM3 with your serial port)
arduino-cli upload -p COM3 --fqbn arduino:avr:uno "car_parking_with_3_slots"
```

For ESP32/ESP8266, change `--fqbn` to the appropriate board FQBN (e.g., `espressif:esp32:esp32`).

## Configuration

Open each `.ino` file and look for a configuration block near the top. Typical values to set:

- WiFi SSID and password (for `send_data_cam.ino` or other Wi‑Fi sketches)
- Server / API endpoint to report slot status
- Sensor pins (trigger/echo for ultrasonic sensors) and LED pins

Example (pseudo):

```cpp
// Edit these
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

// Pins
const int TRIG_PIN_SLOT1 = 2;
const int ECHO_PIN_SLOT1 = 3;
```

## Wiring / Connections

Create an `images/` folder at the repo root and add wiring diagrams or photos. Example filenames suggested below — they are referenced in this README if present:

- `images/wiring-diagram.png` — Overall wiring diagram
- `images/slot-wiring.jpg` — Close-up of sensor wiring
- `images/esp32-cam-setup.png` — Camera module wiring

Placeholders above are referenced in the Images section below.

## How it works (high level)

1. Each parking slot has a proximity sensor (ultrasonic/IR). The microcontroller periodically measures distance.
2. If distance falls below a threshold, the slot is considered occupied.
3. Occupancy state is shown via local indicators (LEDs) and optionally transmitted via Wi‑Fi to a server.
4. The camera sketch can capture images and send them to a configured endpoint for monitoring or analytics.

## Screenshots & Images

Add images to the `images/` folder. Below are multiple suggested placeholders so you can document the project with per-slot photos, wiring diagrams, and demo shots. Replace these placeholders with your real photos or schematics.

Suggested image filenames (create an `images/` folder at the repo root):

- `images/banner.png` — Project banner used at the top of this README
- `images/wiring-diagram.png` — Overall wiring schematic
- `images/slot1.jpg` — Photo or diagram for Slot 1 (sensor/LED close-up)
- `images/slot2.jpg` — Photo or diagram for Slot 2
- `images/slot3.jpg` — Photo or diagram for Slot 3
- `images/demo-1.jpg` — Prototype / assembled view (front)
- `images/demo-2.jpg` — Prototype / assembled view (another angle)

Quick gallery examples

Markdown (simple):

```markdown
![Demo 1](./images/demo-1.jpg)
![Demo 2](./images/demo-2.jpg)
```

Inline HTML for a centered gallery (renders on GitHub):

<p align="center">
	<img src="./images/demo-1.jpg" alt="Demo 1" width="280" />&nbsp;
	<img src="./images/demo-2.jpg" alt="Demo 2" width="280" />&nbsp;
	<img src="./images/wiring-diagram.png" alt="Wiring" width="280" />
</p>

Per-slot documentation (recommended):

- Slot 1 — `images/slot1.jpg`: show sensor orientation, connections, and measured distance range.
- Slot 2 — `images/slot2.jpg`: same for slot 2.
- Slot 3 — `images/slot3.jpg`: same for slot 3.

Tips

- Keep images under ~1–2 MB for fast loading on GitHub. Resize or compress if necessary.
- Use descriptive filenames and captions so contributors can quickly understand what each photo shows.
- If you want, attach a PDF schematic in `images/` (e.g., `images/schematic.pdf`) and link to it from the README.

To add your images, create the `images/` folder and place the files with the suggested names, then commit them. The README examples above will automatically render the images on GitHub.

## Troubleshooting

- No sensor readings / constant values: check wiring, power, and pin definitions.
- Wi‑Fi not connecting: verify SSID/password and board Wi‑Fi firmware; check serial logs for errors.
- Upload fails: confirm correct board and port in IDE; install required board packages.

If you run into issues, open an issue in this repository with logs, photos, and the sketch used.

## Contributing

Contributions are welcome. Suggested workflow:

1. Fork the repo
2. Create a feature branch
3. Commit changes and open a Pull Request describing your changes

Please include hardware details and tested board types for any changes to sketches.

## License

This project includes a `LICENSE` file in the repository root. By default the repo already contains a license — review `LICENSE` for terms.

## Contact / Authors

Maintainer: (add your name and contact info here)

For quick questions, open a GitHub Issue in this repository.

---

If you'd like, I can:

- Add wiring diagrams and photos into `images/` (upload them or tell me where they live).
- Add Arduino library installation instructions or a `platformio.ini` for PlatformIO.
- Generate example server endpoints (Node.js/Flask) to receive parking data.

Tell me which of the above you'd like next.
