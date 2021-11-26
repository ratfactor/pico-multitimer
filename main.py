# Dave's Multi-Timer - 2021, MIT License
# Hardware:
#  - Raspberry Pi Pico microcontroller (RP2040 32-bit dual ARM Cortex-M0+)
#  - Adafruit NeoKey 1x4 QT I2C Breakout (Cherry-compatible keypad + NeoPixels)
#  - SparkFun SerLCD (RGB 16x2 LCD + AVR microcontroller with I2C, etc.)
# Runtime/VM: MicroPython
# http://docs.micropython.org/en/latest/rp2/quickref.html


# TODO: turn these into "from <module> import <object>" to (marginally) reduce
# source size and make it clearer at the outset what's being used.
import machine
import utime
import micropython
import binascii
from machine import Timer
from array import array

# When running as 'main.py' on startup, devices weren't ready without
# a wait - let's just go ahead and wait a whole second to be sure.
utime.sleep_ms(1000)

pico_led = machine.Pin(25, machine.Pin.OUT)


##############################################################################
# SETUP I2C (MicroPython machine.I2C interface)
# https://docs.micropython.org/en/latest/library/machine.I2C.html
#
# Setup the Pico's I2C interface. Both the KeoKey and SerLCD are on
# the same physical bus. Each is accessed via an address or ID number.

bus_id = 0   # i2c bus 0 on pi pico
sda = machine.Pin(0)
scl = machine.Pin(1)
i2c = machine.I2C(bus_id, sda=sda, scl=scl, freq=200000)

# I2C Addresses of devices
# find with: print(i2c.scan())
print(i2c.scan())
SERLCD_ID = 114 # sparkfun SerLCD default
NEOKEY_ID = 48  # adafruit NeoKey 1x4 default

# Make some handy I/O functions for these i2c devices. The write functions
# take a list and convert it into a bytearray (which supports buffer
# operations, which are required by writeto()). The read functions take
# an amount of bytes we wish to read from the device.
def write_lcd(byte_list):
    #print("writing",byte_list,"to",SERLCD_ID)
    i2c.writeto(SERLCD_ID, bytearray(byte_list))
    utime.sleep_ms(10)
    
def write_neokey(byte_list):
    #print("writing",byte_list,"to",NEOKEY_ID)
    i2c.writeto(NEOKEY_ID, bytearray(byte_list))

def read_lcd(amount_bytes):
    return i2c.readfrom(SERLCD_ID, amount_bytes)

def read_neokey(amount_bytes):
    return i2c.readfrom(NEOKEY_ID, amount_bytes)

##############################################################################
# SETUP LCD (SparkFun SerLCD)
# https://github.com/sparkfun/Qwiic_SerLCD_Py/blob/main/qwiic_serlcd.py

# All SerLCD commands start with one of two prefixes. Everything else is
# treated as character data to be written directly to the screen.
LCD_SETCMD     = 0x7C
LCD_CLEAR      = [LCD_SETCMD, 0x2D]
LCD_CONTRAST   = [LCD_SETCMD, 0x18] # followed by 0-255 contrast level
LCD_SETRGB     = [LCD_SETCMD, 0x2B] # followed by 0xFFFFFF style byte list (RGB)
LCD_SPECIALCMD = [0xFE]
LCD_SETDDRAMADDR = 0x80

# Set contrast and clear screen.
# (Contrast was determined by a little looping test program.)
#write_lcd(LCD_CONTRAST + [0xAA])
write_lcd(LCD_CLEAR)
write_lcd(LCD_SETRGB + [0xFF, 0xFF, 0xFF])


write_lcd("Once upon a time...")
utime.sleep_ms(500) # see if a wait here helps seesaw get ready

def write_lcd_line_two():
    # Setting DRAM address is a "special command"
    # Addr 0 is the first character on the first row.
    LCD_SETDDRAMADDR = 0x80
    write_lcd(LCD_SPECIALCMD + [LCD_SETDDRAMADDR | 0x40])


##############################################################################
# NeoKey (keypad switches and NeoPixel RGB LEDs with mcu with Seesaw for I2C)
# https://github.com/adafruit/Adafruit_CircuitPython_seesaw/blob/main/adafruit_seesaw/neopixel.py
# https://github.com/adafruit/Adafruit_CircuitPython_seesaw/blob/main/adafruit_seesaw/keypad.py
# https://github.com/adafruit/Adafruit_CircuitPython_seesaw/blob/main/adafruit_seesaw/seesaw.py
# https://github.com/adafruit/Adafruit_CircuitPython_NeoKey
# https://learn.adafruit.com/adafruit-seesaw-atsamd09-breakout/neopixel

# TODO: these specific functions are superfluous and not very helpful for
# readability. Convert all calls into write_neokey() and define 
# device+command combos like I did for the SerLCD above:
#
#     LCD_CLEAR    = [LCD_SETCMD, 0x2D]
#
# The Adafruit NeoKey uses a "seesaw" microcontroller. To specify a device on
# the seesaw, we use a "base" ID. Here are some handy write functions to talk
# to the seesaw devices. We don't need to do this for read because we WRITE to
# a specific device but READ from the seesaw as a whole.
def write_neokey_status(byte_list):
    write_neokey([0x00] + byte_list) # STATUS

def write_gpio(byte_list):
    write_neokey([0x01] + byte_list) # GPIO

def write_neopixel(byte_list):
    write_neokey([0x0E] + byte_list) # NEOPIXEL

def write_keypad(byte_list):
    write_neokey([0x10] + byte_list) # KEYPAD

# seesaw status commands (which are "register" addresses)
SS_SWRST = 0x7F # seesaw status software reset
SS_HW_ID = 0x01 # seesaw status get hardare id
# gpio commands
GP_DIRCLR = 0x03 # clear (input) direction for pins
GP_SET = 0x05 # enable IO for pins
GP_PULLENABLE = 0x0B # enable pull resistors for pins
GP_READ_BULK = 0x04 # request all pins
GP_INTENSET = 0x08 # set gpio interrupt for pins
GP_INTFLAG = 0x0A # request interrupt flags and clear them
# neopixel commands
NP_PIN = 0x01 # set the neopixel pin
NP_BUF_LENGTH = 0x03 # set buffer length
NP_BUF = 0x04 # set the actual pixel buffer
NP_SHOW = 0x05 # show the pixels!
# keypad commands
KP_STATUS = const(0x00)
KP_EVENT = const(0x01)
KP_INTENSET = const(0x02) # enable interrupt
KP_INTENCLR = const(0x03)
KP_COUNT = const(0x04)
KP_FIFO = const(0x10)

# Reset seesaw and get the ID just to prove we can talk to it
write_neokey_status([SS_SWRST, 0xFF]) # reset (write to reset "register")
utime.sleep_ms(500)
write_neokey_status([SS_HW_ID]) # request chip id
utime.sleep_ms(10)
chip_id = read_neokey(1) # read (bytes)
print("seesaw chip id:", chip_id)

# Set neopixel pin 3, buffer length 12 bytes (3x4 pixels)
write_neopixel([NP_PIN, 3])
write_neopixel([NP_BUF_LENGTH, 0, 12]) # two bytes: 00 12

# Set keypad pins to PULLUP and enable interrupts
kp_pins = [0, 0, 0, 0b11110000] # pins 4-7 (out of 64 possible bits)
write_gpio([GP_DIRCLR] + kp_pins)     # input
write_gpio([GP_PULLENABLE] + kp_pins) # enable pullup
write_gpio([GP_SET] + kp_pins)        # enable I/O
utime.sleep_ms(10)
write_gpio([GP_INTENSET] + kp_pins)

# TODO: functions to manipulate and then write/display this buffer
# neopixel_buffer = sets of 3 bytes (rgb) for each pixel
# NOTE: first two bytes are start address of the portion of the buffer
# which is the beginning because we can fit the whole thing for this
# one: 0, 0.
# 0x04 = "NEOPIXEL_BUF"
neopixel_buffer_cmd = [
    NP_BUF,
    0, 0, # start address
    0x11, 0x00, 0x00,
    0x00, 0x33, 0x00,
    0x00, 0x00, 0x33,
    0x00, 0x33, 0x33,
    ]

write_neopixel(neopixel_buffer_cmd)
write_neopixel([NP_SHOW])

# Two arrays store which timers are active and the current elapsed times in
# seconds.
# B=unsigned byte
# L=unsigned long (4 bytes)
keys_active  = array('B', [0, 0, 0, 0])
timers       = array('L', [469, 945, 200, 9340])

# We store the minutes calculations and totals so we can efficiently update
# the LCD only as needed
minutes      = array('L', [0, 0, 0, 0])
total_minutes = 0
prev_total_minutes = -1

# Setup IRQ and handler for keypress interrupt from neokey
def on_keypress(pin):
    micropython.schedule(handle_keypress, 0) # second param not used

def handle_keypress(throwaway_arg):
    # Get keypress statuses
    write_gpio([GP_INTFLAG])
    utime.sleep_ms(10)
    flags = read_neokey(4) # bytes
    
    # Clear all keys, currently have just one active...
    keys_active[0] = 0
    keys_active[1] = 0
    keys_active[2] = 0
    keys_active[3] = 0
            
    key = "unknown"
    if flags[3] == 16:
        key = "A"
        keys_active[0] = 1
        write_lcd(LCD_SETRGB + [0x00, 0xFF, 0x00])

    if flags[3] == 32:
        key = "B"
        keys_active[1] = 1
        write_lcd(LCD_SETRGB + [0xFF, 0x00, 0x00])

    if flags[3] == 64:
        key = "C"
        keys_active[2] = 1
        write_lcd(LCD_SETRGB + [0x00, 0x00, 0xFF])

    if flags[3] == 128:
        key = "D"
        keys_active[3] = 1
        write_lcd(LCD_SETRGB + [0xFF, 0x00, 0xFF])


# Set IRQ for the interrupt coming from the keypad into the pico
p14 = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP)
p14.irq(on_keypress, machine.Pin.IRQ_FALLING)

def per_second(arg1):
    global timers, minutes, total_minutes, prev_total_minutes
    # Wink Pico's on-board LED so we can see the ticks
    pico_led.value(not pico_led.value())

    # Increment the elapsed seconds of an active timer. Silly
    # but it'll do. I might clean all this up later...or not.
    # I don't see a better method than divmod() for integer
    # division?
    if keys_active[0] == 1:
        timers[0] += 1
        minutes[0] = divmod(timers[0], 60)[0]
    if keys_active[1] == 1:
        timers[1] += 1
        minutes[1] = divmod(timers[1], 60)[0]
    if keys_active[2] == 1:
        timers[2] += 1
        minutes[2] = divmod(timers[2], 60)[0]
    if keys_active[3] == 1:
        timers[3] += 1
        minutes[3] = divmod(timers[3], 60)[0]
    
    total_minutes = sum(minutes) #minutes[0] + minutes[1] + minutes[2] + minutes[3]
    
    # If the minutes have increased, write new totals to the LCD
    if total_minutes > prev_total_minutes:
        prev_total_minutes = total_minutes
        write_lcd(LCD_CLEAR)
        write_lcd("Total min: " + str(total_minutes))
        #write_lcd("abcdefghijklmnopqrstuvwxyz")
        # The address offset + the position in memory for the second
        # row (starting at 0x40), then +4 for padding for each timer counter
        write_lcd(LCD_SPECIALCMD + [LCD_SETDDRAMADDR | 0x40])
        write_lcd(str(minutes[0]))
        write_lcd(LCD_SPECIALCMD + [LCD_SETDDRAMADDR | 0x44])
        write_lcd(str(minutes[1]))
        write_lcd(LCD_SPECIALCMD + [LCD_SETDDRAMADDR | 0x48])
        write_lcd(str(minutes[2]))
        write_lcd(LCD_SPECIALCMD + [LCD_SETDDRAMADDR | 0x4c])
        write_lcd(str(minutes[3]))        

# Create an interrupt every second from the RTC (Real Time Clock)
timer = Timer(period=1000, mode=Timer.PERIODIC, callback=per_second)
