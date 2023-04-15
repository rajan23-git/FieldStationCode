#Imports for LoRa
# Import Python System Libraries
import time
# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import the SSD1306 module.
import adafruit_ssd1306
# Import RFM9x
import adafruit_rfm9x

#Imports for read_serial
import time
import busio
import digitalio
import board

import serial
import time

from datetime import datetime
# import pytz
from csv import DictWriter
import os
from threading import Thread, Lock
from queue import Queue
from datetime import datetime
from pytz import timezone
import time
# Debug msgs
DEBUG = True

# Delay between sending radio data, in minutes
SENSOR_SEND_DELAY = 1

# Create the I2C interface.
i2c = busio.I2C(board.SCL, board.SDA)

# Create library object using our Bus I2C port
#bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

##Define buttons for testing
# Button A
btnA = DigitalInOut(board.D5)
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP

# Button B
btnB = DigitalInOut(board.D6)
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP

# Button C
btnC = DigitalInOut(board.D12)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP


# 128x32 OLED Display
reset_pin = DigitalInOut(board.D4)
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)
# Clear the display.
display.fill(0)
display.show()
width = display.width
height = display.height

#define radio frequency
RADIO_FREQ_MHZ = 915.0

# Configure LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

#set transmit power to max
rfm9x.tx_power = 23

#prev_packet = None

# Define the onboard LED
LED = digitalio.DigitalInOut(board.D13)
LED.direction = digitalio.Direction.OUTPUT

# create empty packet for sensor data
#sensor_data = bytearray(8)




def sendSensorData(dataa):
    packet = None
    # draw a box to clear the image
    display.fill(0)
    display.text('RasPi LoRa', 35, 0, 1)
   
    if dataa is None:
        display.show()
        display.text('- Waiting for PKT -', 15, 20, 1)
        return
    else:
        display.fill(0)
        rfm9x.send(dataa)
        display.text('sent data', 15, 20, 1)
       
    display.show()
    time.sleep(0.1)
    return

def writeFileThread(queue, folder):
    currDate = (0,0,0) # y, m, d
        # y=0, m=0, d=0 is an impossible date
    currFile = None
    while True:
        if(queue.empty() == False):
            
            data = queue.get()
            packetDate = data.split(',')[0]
            if(currDate != packetDate): # switch files each day
                currDate = packetDate
                if currFile != None:
                    currFile.close()
                currFile = open(folder + str(currDate[0]) + "-" + str(currDate[1]) + "-" + str(currDate[2]) + "w.txt", "a")
            currFile.write(data + '\n')

def serial_port_thread(port, locks, list, idx, baudrate=9600, timeout=5):

    with serial.Serial(port=port, baudrate=baudrate, timeout=timeout) as ard:
        time.sleep(2) # wait for port to be "ready"
            # unsure as to why this is necessary
            # if sleep is <2 then the first packet is lost
        startSendTime = time.perf_counter()
        
        while(True):

            endSendTime = time.perf_counter()
            receivedTime = rfm9x.receive()
            #if it receives time adjustment value, it will send this value to the Arduino RTC every 10 seconds
            if((receivedTime is not None)  and ((endSendTime -startSendTime) > 10) ):
                try:
                    ard.write(receivedTime)
                    print("Sending the time ", str(receivedTime,'utf-8')," to arduino RTC")
                except BaseException as k:
                    print(k)
                startSendTime = endSendTime
                    
                        
            locks[0].acquire() # wait for signal from main thread
            ard.write((255).to_bytes(1,'big'))
            if DEBUG: print("child thread: pinging arduino on", port)
            data = ard.readline()[:-2]
            if DEBUG: print("child thread: received", data, "on", port)
            list[idx] = data
            locks[1].release() # send signal to main thread


ports = ("/dev/ttyACM0", "/dev/ttyACM1") # ports used
max_packet_num = 256 # max packet number before wrap around
n_ports = len(ports)
threads = [] # arduino threads
ard_out = [] # list for threads to output to
locks = [] # list of locks for controlling threads
sample_rate = 1 # how many seconds to wait between samples
path = "database/" # path to database folder



# init file thread
fqueue = Queue() # packets to write to file
fthread = Thread(target=writeFileThread, args=(fqueue, path))
fthread.start()


# init arduino threads
for i in range(n_ports):
    if DEBUG: print("main thread: init thread for port", ports[i])
    # init locks
    l1 = Lock()
    l2 = Lock()
    l1.acquire(blocking=False)
    l2.acquire(blocking=False)
    locks.append((l1, l2))
    # init output arr
    ard_out.append(b'\x00')
    # init thread
    t = Thread(target=serial_port_thread, args=(ports[i],(l1, l2),ard_out,i))
    t.start()
    threads.append(t)

packet_num = 0

while True:
    # release threads
            
    for i in range(n_ports):
        if DEBUG: print("main thread: releasing thread", i)
        locks[i][0].release()
    # wait for threads to finish
    for i in range(n_ports):
        locks[i][1].acquire()
        if DEBUG: print("main thread: thread", i, "has finished")
    # get time
    epoch = int(time.time())
    # create packet
    packet = str(epoch)
    for out in ard_out:
        try:
            packet += ',' + out.decode('UTF-8')
        except BaseException as k:
            print(k)
       
            
    packet_num = (packet_num + 1) % max_packet_num
    if DEBUG: print("packet:", packet_num, packet)
    if(len(bytearray(packet,'UTF-8'))>0):
        sendSensorData(bytearray(packet,'UTF-8'))
        fqueue.put(packet)
    # wait for next sample time
    time.sleep(sample_rate)

