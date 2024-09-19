
import time 
import serial 
import azimuth_elevation_angle as azel
import RPi.GPIO as GPIO
import subprocess
import shlex
from datetime import datetime
import sys
import re

## This class is designed to work with both lora_send and lora_receive components
## Any additional features that needs to be added to the lora_send or lora_receive
## Can be added as a method to this class

class ground_station:

    HAB_lat = []
    HAB_lon = []
    HAB_time = []
    HAB_alt = []

    def __init__(self,serial_num):

        # The hardware UART of Pi3B+,Pi4B is /dev/ttyS0
        self.ser = serial.Serial(serial_num,9600)
        self.ser.flushInput()

    def getGroundData(self): 

        print("getGroundData")
        sys.stderr.write("getGroundData")
        bearing_flag = 0
        ground_flag = 0

        while bearing_flag == 0 or ground_flag == 0:   

            msg = str(self.ser.readline())

            if msg.find("bearing") == 2:

                bearing = float(msg[10:-5])
                time.sleep(1)
                bearing_flag = 1

            if msg.find("GPGGA") == 3:

                ground_lat = msg[19:29]
                ground_lon = msg[33:43]
                ground_alt = msg[51:55]
                ground_flag = 1


                if(msg[30] == "S"):

                    ground_lat = -1*float(ground_lat)
                    ground_lat = float(ground_lat)/100
                
                elif(msg[30] == "N"):
                    
                    ground_lat = 1*float(ground_lat)
                    ground_lat = float(ground_lat)/100
                
                if(msg[44] == "W"):

                    ground_lon = -1*float(ground_lon)
                    ground_lon = float(ground_lon)/100

                elif(msg[44] == "E"):

                    ground_lon = 1*float(ground_lon)
                    ground_lon = float(ground_lon)/100

        return [ground_lat, ground_lon, ground_alt, bearing]

    # This method returns relevant HAB data
    def getHABData(self):

        print("getHABData")
        sys.stderr.write("getHABData")
        time_flag = 0
        alt_flag = 0
        lat_flag = 0
        lon_flag = 0
	
        while time_flag == 0 or alt_flag == 0 or lat_flag == 0 or lon_flag == 0:   

            msg = str(self.ser.readline())
            datafile = open('output.txt', 'a')
            now = datetime.now()
            date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
            log_msg = date_time + "," + msg + "\n"
            datafile.write(log_msg)
            datafile.close()

            if msg.find("time") == 2:

                HAB_time = msg[8:12]
                time_flag = 1

            if msg.find("alt") == 2:

                HAB_alt = msg[7:17]
                alt_flag = 1
            
            if msg.find("lat") == 2:

                HAB_lat = msg[7:17]
                size = len(HAB_lat)

                if(HAB_lat[size-1] == "N"):
                
                    HAB_lat = HAB_lat[:size-1]
                    HAB_lat = float(HAB_lat)
                    lat_flag = 1
                    
                else:
                
                    HAB_lat = HAB_lat[:size-1]
                    HAB_lat = -1*float(HAB_lat)
                    lat_flag = 1
                    
            if msg.find("lon") == 2:

                HAB_lon = msg[7:17]
                size = len(HAB_lon)
            
                if(HAB_lon[size-1] == "E"):
                
                    HAB_lon = HAB_lon[:size-1]
                    HAB_lon = float(HAB_lon)
                    lon_flag = 1
                    
                else:
                    HAB_lon = HAB_lon[:size-1]
                    HAB_lon = -1*float(HAB_lon)
                    lon_flag = 1
        
        return [HAB_lat, HAB_lon, HAB_alt, HAB_time]

    # This method drives the ground station
    def moveGroundStation(self, ground_lat, ground_lon, ground_alt, HAB_lat, HAB_lon, HAB_alt, bearing):
    
      [az,el] = azel.azimuth_elevation_angle(ground_lat, ground_lon, ground_alt, HAB_lat, HAB_lon, HAB_alt, bearing)

      az = int(az)
      el = int(el)
      ser1 = serial.Serial("/dev/ttyUSB1")
      
      # method that will convert variable into a 3 digit string
      if (az < 0):
       az_string = f'{az:04d}'
      else:
       az_string = f'{az:03d}'

      if (el < 0):
       el_string = f'{el:04d}'
      else:
       el_string = f'{el:03d}'

      ser1.write(bytearray(f"W{az_string} {el_string}\r", 'ascii'))
      #ser1.write(bytearray(f"W090 090\r", 'ascii'))
