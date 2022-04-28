import serial
import time
import keyboard

s = serial.Serial(port = '/dev/cu.usbmodem143101',baudrate=9600,timeout=0.1)

#e = serial.Serial(port = '/dev/cu.usbmodem#####',baudrate=115200,timeout=0.1)

def write_mobile(x):
    s.write(bytes(x,'utf-8'))
    time.sleep(0.05)
    data = s.readline()
    return data

def write_end(x):
    s.write(bytes(x,'utf-8'))
    time.sleep(0.05)
    data = s.readline()
    return data

while True:
    event = keyboard.read_event()
    if event.event_type == keyboard.KEY_DOWN and event.name == 'up':
        write_mobile('w')
    elif event.event_type == keyboard.KEY_DOWN and event.name == 'down':
        write_mobile('s')
    elif event.event_type == keyboard.KEY_DOWN and event.name == 'left':
        write_mobile('a')
    elif event.event_type == keyboard.KEY_DOWN and event.name == 'right':
        write_mobile('d')
    elif event.event_type == keyboard.KEY_DOWN and event.name == 'space':
        write_end('lift')
    else:
        write_mobile(' ')
