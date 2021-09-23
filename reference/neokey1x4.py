# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
``adafruit_neokey1x4``
====================================================

1x4 mechanical key buttons and RGB LEDs

* Author(s): ladyada

Implementation Notes
--------------------

**Hardware:**

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

* Adafruit Seesaw CircuitPython library
  https://github.com/adafruit/Adafruit_CircuitPython_seesaw/releases
"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_NeoKey.git"

from micropython import const
from adafruit_seesaw.neopixel import NeoPixel
from adafruit_seesaw.seesaw import Seesaw

_NEOKEY1X4_ADDR = const(0x30)

_NEOKEY1X4_NEOPIX_PIN = const(3)

_NEOKEY1X4_NUM_ROWS = const(1)
_NEOKEY1X4_NUM_COLS = const(4)
_NEOKEY1X4_NUM_KEYS = const(4)


class NeoKey1x4(Seesaw):
    """Driver for the Adafruit NeoKey 1x4."""

    def __init__(self, i2c_bus, interrupt=False, addr=_NEOKEY1X4_ADDR):
        super().__init__(i2c_bus, addr)
        self.interrupt_enabled = interrupt
        self.pixels = NeoPixel(
            self, _NEOKEY1X4_NEOPIX_PIN, _NEOKEY1X4_NUM_KEYS, brightness=0.2
        )
        # set the pins to inputs, pullups
        for b in range(4, 8):
            self.pin_mode(b, self.INPUT_PULLUP)

    def __getitem__(self, index):
        if not isinstance(index, int) or (index < 0) or (index > 3):
            raise RuntimeError("Index must be 0 thru 3")
        return not self.digital_read(index + 4)
