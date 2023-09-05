# Sketch for streaming MPD on a RaspiAudio MUSE Luxe.

This is a modification of the original internet radio firmware for the MUSE Luxe: https://github.com/RASPIAUDIO/Muse_Luxe_radio/

It is tailored for my own personal use, your mileage may vary.

## Modifications:
 - WiFi is hardcoded, builtin WiFi AP and HTTP server for setup removed.
 - It uses a Grove Light Gesture Color Proximity Sensor (TMG39931) for sleeping when dark.
 - A long press on Plus and Minus buttons will skip to the next/previous song.

## Installation

See https://forum.raspiaudio.com/t/muse-luxe-internet-radio/301 for a tutorial on how to compile and install it.

You'll need:
 - ESP32 package (see tutorial)
 - The MUSE library from: https://github.com/RASPIAUDIO/Simple_Bluetooth_Speaker_ESP32
 - The TMG39931 library: https://github.com/Seeed-Studio/Seeed_TMG3993

Modify the configuration of the sketch to adapt to your WiFi and MPD service.
