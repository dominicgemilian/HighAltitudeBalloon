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


            if msg.find("bearing: ") != -1:

                bearing_str = msg.replace("b'bearing: ", '')
                bearing_str = bearing_str.replace("\\r\\n'", '')
                bearing = float(bearing_str)
                bearing_flag = 1
                

            if msg.find("Latitude: ") != -1:
                
                
                ground_lat_str = msg.replace("b'Latitude: ", '')
                ground_lat_str = ground_lat_str.replace("\\r\\n'", '')
                ground_lat = float(ground_lat_str)
                lat_flag = 1

            elif msg.find("Longitude: ") != -1:
                 
                ground_lon_str = msg.replace("b'Longitude: ", '')
                ground_lon_str = ground_lon_str.replace("\\r\\n'", '')
                ground_lon = float(ground_lon_str)
                lon_flag = 1
                
            elif msg.find("Altitude (meters): ") != -1:

                ground_alt_str = msg.replace("b'Altitude (meters): ", '')
                ground_alt_str = ground_alt_str.replace("\\r\\n'", '')
                ground_alt = float(ground_alt_str)
                alt_flag = 1              

            if debugFlag:
		
                if bearing_flag == 1:
             		
                    print("Bearing: ", str(bearing), " degrees")
                
                if lat_flag == 1:
	
                    print("Latitude: ", str(ground_lat), " degrees")

                if lon_flag == 1:

                    print("Longitude: ", str(ground_lon), " degrees")

                if alt_flag == 1:

                    print("Altitude" , ground_alt, " meters")

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
            
            #atafile = open('output.txt', 'a')
            #now = datetime.now()
            #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
            #log_msg = date_time + "," + msg + "\n"
            #datafile.write(log_msg)
            #datafile.close()
                        
            if (debugFlag):
             print("=========================== Incoming PNT Message =========================")
             print(msg)


            if msg.find("time: ") != -1:

                time_str = msg.replace("b'time: ", '')
                time_str = time_str.replace("\\r\\n'", '')
                time = float(time_str)
                time_flag = 1
                

            if msg.find("Latitude: ") != -1:
                
                
                hab_lat_str = msg.replace("b'Latitude: ", '')
                hab_lat_str = hab_lat_str.replace("\\r\\n'", '')
                hab_lat = float(hab_lat_str)
                lat_flag = 1

            elif msg.find("Longitude: ") != -1:
                 
                hab_lon_str = msg.replace("b'Longitude: ", '')
                hab_lon_str = hab_lon_str.replace("\\r\\n'", '')
                hab_lon = float(hab_lon_str)
                lon_flag = 1
                
            elif msg.find("Altitude (kilometers): ") != -1:

                hab_alt_str = msg.replace("b'Altitude (kilometers): ", '')
                hab_alt_str = hab_alt_str.replace("\\r\\n'", '')
                hab_alt = float(hab_alt_str)/1000
                alt_flag = 1              

            if debugFlag:
		
                if time_flag == 1:
             		
                    print("Time: ", str(time), " UTC")
                
                if lat_flag == 1:
	
                    print("Latitude: ", str(hab_lat), " degrees")

                if lon_flag == 1:

                    print("Longitude: ", str(hab_lon), " degrees")

                if alt_flag == 1:

                    print("Altitude: " , hab_alt, " meters")

        return [hab_lat, hab_lon, hab_alt, bearing]


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
      
