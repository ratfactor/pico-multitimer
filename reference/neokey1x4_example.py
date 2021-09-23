# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import board
import busio
import digitalio
from adafruit_neokey.neokey1x4 import NeoKey1x4

# use default I2C bus
#i2c_bus = board.I2C()

# Dave 2021 - I've modified this example for the Raspberry Pi Pico
# microcontroller board. (RP2040)
i2c_bus = busio.I2C(board.GP1, board.GP0)

# Create a NeoKey object
neokey = NeoKey1x4(i2c_bus, addr=0x30)

print("Adafruit NeoKey simple test")

# Check each button, if pressed, light up the matching neopixel!
while True:
    if neokey[0]:
        print("Button A")
        neokey.pixels[0] = 0xFF0000
    else:
        neokey.pixels[0] = 0x0

    if neokey[1]:
        print("Button B")
        neokey.pixels[1] = 0xFFFF00
    else:
        neokey.pixels[1] = 0x0

    if neokey[2]:
        print("Button C")
        neokey.pixels[2] = 0x00FF00
    else:
        neokey.pixels[2] = 0x0

    if neokey[3]:
        print("Button D")
        neokey.pixels[3] = 0x00FFFF
    else:
        neokey.pixels[3] = 0x0
