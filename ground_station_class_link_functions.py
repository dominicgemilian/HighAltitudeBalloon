
import time 
import serial 
#import azimuth_elevation_angle as azel
#import RPi.GPIO as GPIO
import subprocess
import shlex
from datetime import datetime
import sys
import re
import linkFunctions

## This class is designed to work with both lora_send and lora_receive components
## Any additional features that needs to be added to the lora_send or lora_receive
## Can be added as a method to this class

class ground_station:

    HAB_lat = []
    HAB_lon = []
    HAB_time = []
    HAB_alt = []

    def __init__(self,serial_num,baud_rate):

        # The hardware UART of Pi3B+,Pi4B is /dev/ttyS0
        self.ser = serial.Serial(serial_num,baud_rate)
        self.ser.flushInput()

    def getGroundData(self, debugFlag = False): 

        sys.stderr.write("getGroundData")
        bearing_flag = 0
        lat_flag = 0
        lon_flag = 0
        alt_flag = 0
        
        while bearing_flag == 0 or lat_flag == 0 or lon_flag == 0 or alt_flag == 0:   

            msg = str(self.ser.readline())
                        
            if (debugFlag):
             print("=========================== Incoming PNT Message =========================")
             print(msg)
             print(msg.find("Latitude"))


            if msg.find("bearing: ") != -1:

                bearing_str1 = msg.replace("b'bearing: ", '')
                bearing_str2 = bearing_str1.replace("\\r\\n'", '')
                bearing = float(bearing_str2)
                bearing_flag = 1
                

            if msg.find("Latitude: ") != -1:
                
                
                ground_lat_str1 = msg.replace("b'Latitude: ", '')
                ground_lat_str2 = ground_lat_str1.replace("\\r\\n'", '')
                ground_lat = float(ground_lat_str2)
                lat_flag = 1

            elif msg.find("Longitude: ") != -1:
                 
                ground_lon_str1 = msg.replace("b'Longitude: ", '')
                ground_lon_str2 = ground_lon_str1.replace("\\r\\n'", '')
                ground_lon = float(ground_lon_str2)
                lon_flag = 1
                
            elif msg.find("Altitude (meters): ") != -1:

                ground_alt_str1 = msg.replace("b'Altitude (meters): ", '')
                ground_alt_str2 = ground_alt_str1.replace("\\r\\n'", '')
                ground_alt = float(ground_alt_str2)
                alt_flag = 1              
                
        return [ground_lat, ground_lon, ground_alt, bearing]

    # This method returns relevant HAB data
    def getHABData(self, debugFlag=False):

        print("getHABData")
        time_flag = 0
        alt_flag = 0
        lat_flag = 0
        lon_flag = 0

        while time_flag == 0 or alt_flag == 0 or lat_flag == 0 or lon_flag == 0:   
            msg = str(self.ser.readline())
            if (debugFlag):
             print(msg)
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

                HAB_alt = msg[7:15]
                alt_flag = 1

            if msg.find("lat") == 2:

                HAB_lat = msg[7:16]
                size = len(HAB_lat)
                HAB_lat = float(HAB_lat)
                lat_flag = 1

            if msg.find("lon") == 2:

                HAB_lon = msg[7:16]
                size = len(HAB_lon)
                HAB_lon = float(HAB_lon)
                lon_flag = 1
                
        return [HAB_lat, HAB_lon, HAB_alt, HAB_time]

    # This method drives the ground station
    def moveGroundStation(self, groundStation, habPayload, bearing, debugFlag = False, noSerial = False):
      
      [sourceLookENU, targetLookENU] = linkFunctions.calculateLookVectorENU(groundStation.location, habPayload.location, True)
      [thetaSource, phiSource, thetaTarget, phiTarget] = linkFunctions.calculateSphericalAngles(sourceLookENU, targetLookENU, True)

      az = int(phiSource)
      el = int(thetaSource)
     
      if (debugFlag):
       print("======================== PTU Pointing Angles ========================")
       print("Az: ", az, " Degrees")
       print("El: ", el, " Degrees")

      
      # method that will convert variable into a 3 digit string
      if (az < 0):
       az_string = f'{az:04d}'
      else:
       az_string = f'{az:03d}'

      if (el < 0):
       el_string = f'{el:04d}'
      else:
       el_string = f'{el:03d}'
      
      if (noSerial == False):
       ser1 = serial.Serial("/dev/ttyUSB1")
       ser1.write(bytearray(f"W{az_string} {el_string}\r", 'ascii'))
      #ser1.write(bytearray(f"W090 090\r", 'ascii'))
      
