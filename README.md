# Test out the Adafrutit
## ADS1115 Analog to digital Converter
## 

### 

## Contents
- [Notes](#notes)
- [Installation Instructions](#installation-instructions)
- [Issue/Bug report template](#issuebug-report-template)
- [WEMOS LOLIN32 with OLED Board](#WEMOS-LOLIN32)
- [Heltec WiFi LORA 32 V1](#Heltec-WiFi-LORA-32-V1)
- [Adafruit ADS1115](#Adafruit-ADS1115)

## Notes
You may have to push the button to upload to the board.
ADS1115 does not seem to work with HW enabled on the OLED on the WiFi LORA

## Installation Instructions
- Use VS Code and PlatformIO

#### Issue/Bug report template
Finally, if you're sure no one else had the issue, it's probably you.

## WEMOS LOLIN32
Board name WEMOS LOLIN32.<br/>
<ul>
    <li>OLED Connections:</li>
        <li>CLOCK 4</li>
        <li>DATA 5</li>
        <li>RESET 16</li>
</ul>

![Pin Functions](docs/WemosESP32OLEDTop.jpg)
![Pin Functions](docs/WemosESP32OLEDBottom.jpg)
![Pin Functions](docs/WemosESP32OLEDPinout.jpg)

## Heltec WiFi LORA 32 V1
Heltec WiFi LORA 32 V1 – ESP32 with OLED and LORA Board<br/>
To program you need to hold button labeled PRG near coil antenna.<br/>
<ul>
    <li>I2C Connections:</li>
        <li>SCL 22</li>
        <li>SDA 21</li>
    <li>OLED Connections:</li>
        <li>CLOCK 15</li>
        <li>DATA 4</li>
        <li>RESET 16</li>
</ul>

![Pin Functions](docs/WiFi-LORA-32-pinout-Diagram.png)

## Adafruit ADS1115
![Pin Functions](docs/AdafruitADS1015ADS1115PinDiagram.jpg)

## Hint
Keep at it.