<h1 align="center">Mobile Robot with Pick and Place</h1>
<p align="center"><strong>ROBOTICS | EMBEDDED CONTROL | MANIPULATION</strong></p>
<p align="center"><a href="https://github.com/Kaveesha229">Kaveesha Gunawardena | Engineering portfolio</a></p>

## Overview

A mobile robotics project combining line following, a two servo pick and place mechanism, colour sensing and camera related experiments. The repository contains controller firmware, component tests and a project report.

## Repository guide

<table>
<tr><th>File or folder</th><th>Purpose</th></tr>
<tr><td><a href="to_left.ino">to_left.ino</a></td><td>Root ESP32 navigation and task sequence</td></tr>
<tr><td><a href="nano.ino">nano.ino</a></td><td>Arduino servo and colour sensing controller</td></tr>
<tr><td><a href="https://github.com/Kaveesha229/KDU%2DMSD%2DLine%2Dfollowing%2Drobot%2Dwith%2Dpick%2Dand%2Dplace%2Dusing%2Dcolor%2Dsensor%2Dand%2Dimage%2Dprocessing/tree/main/MSD/final%20MSD">MSD/final MSD</a></td><td>Integrated firmware variants and experiments</td></tr>
<tr><td><a href="https://github.com/Kaveesha229/KDU%2DMSD%2DLine%2Dfollowing%2Drobot%2Dwith%2Dpick%2Dand%2Dplace%2Dusing%2Dcolor%2Dsensor%2Dand%2Dimage%2Dprocessing/tree/main/MSD/color%20tcs">MSD/color tcs</a></td><td>TCS3200 and serial communication tests</td></tr>
<tr><td><a href="https://github.com/Kaveesha229/KDU%2DMSD%2DLine%2Dfollowing%2Drobot%2Dwith%2Dpick%2Dand%2Dplace%2Dusing%2Dcolor%2Dsensor%2Dand%2Dimage%2Dprocessing/tree/main/MSD/serial%20com">MSD/serial com</a></td><td>Controller communication experiments</td></tr>
<tr><td><a href="MSD%20Report.pdf">MSD Report.pdf</a></td><td>Project report</td></tr>
</table>

## System architecture

<table>
<tr><th>Subsystem</th><th>Responsibility</th></tr>
<tr><td>ESP32</td><td>Infrared sensing, motor control, task sequencing and Blynk input</td></tr>
<tr><td>Arduino Nano</td><td>Servo movement and TCS3200 colour measurement</td></tr>
<tr><td>Serial connection</td><td>Commands and colour codes exchanged at 9600 baud</td></tr>
<tr><td>Camera experiments</td><td>Separate firmware variants within the MSD folder</td></tr>
</table>

## Implemented behaviour

The root ESP32 sketch starts with a Blynk selection phase, then progresses through a numbered task sequence. It uses eight analogue infrared channels for line following and Serial2 on GPIO 16 and 17 for controller communication. The Nano sketch receives character commands, moves two servos and returns colour codes.

## Hardware and libraries

The ESP32 source includes `WiFi` and `BlynkSimpleEsp32`. The Nano source uses `Servo`. In `nano.ino`, servo outputs are pins 10 and 9, while the TCS3200 uses pins 4 through 8. Camera sketches have their own configuration and should be reviewed individually.

## Getting started

1. Read `MSD Report.pdf` and identify the firmware variant matching your physical robot.
2. Open the ESP32 and Nano sketches separately in Arduino IDE with the correct board selections and libraries.
3. Configure your own WiFi and Blynk settings locally.
4. Check serial voltage compatibility, crossed TX and RX wiring, and common ground between controllers.
5. Verify servo limits and motor direction before running the full sequence.
6. Calibrate the infrared threshold and colour response on the intended track.

## Repository notes

Multiple development variants are retained. The root sketches are starting points for inspection, not a declared universal release. The public tree contains embedded network and service credentials; replace and rotate them before reuse. Do not publish replacement credentials.

## Team contribution and results

This documentation describes the available system and code. Individual team responsibilities, repeatability measurements and task completion results should be added from the project records.

## Project media

Photos and demonstration videos will be added here as the project presentation is updated.

<p align="center"><sub>Practical engineering through building, testing and refinement.</sub></p>
