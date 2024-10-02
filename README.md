"picopower" is a small Raspberry Pi Pico based home energy display device.

# Hardware
The hardware required for this project is:
* Raspberry Pi Pico W
* OLED I2C Screen
* 4 NeoPixels

# Connections
The following Pico pins are used:

* OLED SDA  > 01 (SDA)
* OLED SDL  > 02 (SDL)
* OLED 3.3V > 36 (3V3)
* OLED Gnd  > 03 (GND)
* Neopixel 5V   > 40 (VBUS)
* Neopixel Data > 34 (GP28)
* Neopixel Gnd  > 38 (GND)

# Libraries
The following default libraries are required:
* time
* math
* network
* neopixel

The following additional libraries need to be installed:
* writer
* micropython-ssd1306
* umqtt.simple
