# ESP32 Smart Activity Monitor - Vercel Package

## Included files

- `index.html` - Web Bluetooth dashboard, used as the Vercel root page.
- `dashboard.html` - Same dashboard as a standalone page.
- `ESP32_Smart_Activity_Monitor.ino` - Complete ESP32 firmware.
- `vercel.json` - Minimal Vercel configuration.
- `README.md` - This setup guide.

## Vercel deployment

1. Create a new Vercel project.
2. Upload this folder/ZIP contents or connect a Git repository containing these files.
3. No build command is required.
4. No framework preset is required; this is a static HTML site.
5. Deploy.

After deployment, Vercel will provide an HTTPS URL such as:

https://your-project.vercel.app/

Open that URL in a Web Bluetooth compatible browser, normally Chrome or Edge.

## ESP32 setup

Board:
- ESP32 Dev Module

Default I2C pins:
- SDA = GPIO 21
- SCL = GPIO 22

Default addresses:
- MPU6050 = 0x68
- SSD1306 OLED = 0x3C

Required Arduino libraries:
- Adafruit MPU6050
- Adafruit Unified Sensor
- Adafruit GFX Library
- Adafruit SSD1306

The ESP32 BLE service UUIDs are already matched with the dashboard.

BLE device name:
- ESP32-Activity-Monitor

## BLE commands

The dashboard can send:

- RESET_FALL
- RESET_STEPS
- RESET_ALL
- PING

## Important Web Bluetooth notes

- The deployed Vercel URL is HTTPS, which is suitable for Web Bluetooth secure-context requirements.
- Bluetooth access is performed by the browser; the Vercel site does not directly access the PC Bluetooth adapter.
- The user must grant Bluetooth permission when pressing "Connect BLE".
- Use a compatible Chromium-based browser with Web Bluetooth support.

## Fall detection

The fall detector uses a combination of:
- acceleration impact
- gyroscope activity
- orientation change
- post-impact low movement

Thresholds are starting values and should be calibrated using controlled tests for the intended device placement.

This is an engineering prototype and is not a medical or safety-certified fall detector.
