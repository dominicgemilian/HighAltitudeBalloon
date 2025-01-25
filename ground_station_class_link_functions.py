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

        print("getGroundData")
        bearing_flag = 0
        position_flag = 0
        
        while bearing_flag == 0 or position_flag == 0:   

            msg = str(self.ser.readline())

            if (debugFlag):
             print("=========================== Incoming PNT Message =========================")
             print(msg)


            if msg.find("bearing: ") != -1:

                bearing_str = msg.replace("b'bearing: ", '')
                bearing_str = bearing_str.replace("\\r\\n'", '')
                bearing = float(bearing_str)
                bearing_flag = 1
                

            if msg.find("GNRMC") != -1:
                result = self.parse_gnrmc(msg)
                ground_lat = result['latitude']
                ground_lon = result['longitude']
                ground_alt = 0  
                position_flag = 1             
                    

            if debugFlag:
		
                if bearing_flag == 1:
             		
                    print("Bearing: ", str(bearing), " degrees")
                
                if position_flag == 1:
	
                    print("Latitude: ", str(ground_lat), " degrees")
                    print("Longitude: ", str(ground_lon), " degrees")
                    print("Altitude: " , ground_alt, " meters")

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


            if msg.find("Unix Time: ") != -1:

                time_str = msg.replace("b'Unix Time: ", '')
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
             		
                    print("Time: ", str(time))
                
                if lat_flag == 1:
	
                    print("Latitude: ", str(hab_lat), " degrees")

                if lon_flag == 1:

                    print("Longitude: ", str(hab_lon), " degrees")

                if alt_flag == 1:

                    print("Altitude: " , hab_alt, " meters")

        return [hab_lat, hab_lon, hab_alt, time]


    # This method drives the ground station
    def moveGroundStation(self, groundStation, habPayload, bearing, debugFlag = False, noSerial = False):
      
      [sourceLookENU, targetLookENU] = linkFunctions.calculateLookVectorENU(groundStation.location, habPayload.location, True)
      [thetaSource, phiSource, thetaTarget, phiTarget] = linkFunctions.calculateSphericalAngles(sourceLookENU, targetLookENU, True)

      az = (int(phiSource) - int(bearing))%360
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
      
    def parse_gnrmc(self, message):
        """
        Parses a GNRMC NMEA message for latitude and longitude.

        Args:
            message (str): The GNRMC NMEA sentence as a string.

        Returns:
            dict: A dictionary containing latitude, longitude, and status (if valid).
        """
        
        if not message.startswith("b'$GNRMC"):
            raise ValueError("Invalid GNRMC message")

        # Split the message into components
        parts = message.split(",")
        
        if len(parts) < 12:
            raise ValueError("Incomplete GNRMC message")

        # Extract data fields
        status = parts[2]  # A = Active, V = Void
        latitude = parts[3]
        latitude_dir = parts[4]
        longitude = parts[5][1:]
        longitude_dir = parts[6]

        # Check if the data is valid
        if status != "A":
            return {"status": "Invalid data", "latitude": None, "longitude": None}

        # Convert latitude and longitude to decimal degrees
        def convert_to_decimal(degrees, direction):
            if not degrees:
                return None
            
            # Degrees are in ddmm.mmmm format
            d = int(degrees[:2])
            m = float(degrees[2:])
            decimal = d + (m / 60)

            # Apply hemisphere
            if direction in ["S", "W"]:
                decimal = -decimal

            return decimal

        lat_decimal = convert_to_decimal(latitude, latitude_dir)
        lon_decimal = convert_to_decimal(longitude, longitude_dir)

        return {
            "status": "Active",
            "latitude": lat_decimal,
            "longitude": lon_decimal
        }
