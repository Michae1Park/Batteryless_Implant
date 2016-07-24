import time
import serial

init1 = b'010A0003041001210000'
init2 = b'010C00030410002101020000'
init3 = b'0109000304F0000000'
init4 = b'0109000304F1FF0000'
command = b'010C0003041802AA07020000'

usb = serial.Serial('/dev/ttyUSB0', 115200)

usb.write(init1)
time.sleep(.5)
data_size = usb.inWaiting()
data = usb.read(data_size)
print(data)

usb.write(init2)
time.sleep(.5)
data_size = usb.inWaiting()
data = usb.read(data_size)
print(data)

usb.write(init3)
time.sleep(.5)
data_size = usb.inWaiting()
data = usb.read(data_size)
print(data)

usb.write(init4)
time.sleep(.5)
data_size = usb.inWaiting()
data = usb.read(data_size)
print(data)

usb.write(command)
time.sleep(.5)
data_size = usb.inWaiting()
data = usb.read(data_size)
print(data)

usb.write(b'00F')
time.sleep(.5)
data_size = usb.inWaiting()
data = usb.read(data_size)
usb.close()
