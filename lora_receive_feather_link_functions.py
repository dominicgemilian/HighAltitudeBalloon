import time 
import serial 
import ground_station_class_link_functions as ground_station
import sys
import linkFunctions
import terminal
import numpy as np

def main():
    
    print("Press Ctrl+C to exit")
    sys.stderr.write("Press Ctrl+C to exit\n")
    
    node_ground_station = ground_station.ground_station(serial_num = "/dev/ttyUSB1",baud_rate = 38400)
    node_payload = ground_station.ground_station(serial_num = "/dev/ttyACM0",baud_rate = 9600)
    node_rotor = ground_station.ground_station(serial_num = "/dev/ttyUSB0",baud_rate = 9600)

    ptu_ready = 0
    
    groundStation = terminal.terminal()
    habPayload = terminal.terminal()

    while True:    

     print("Waiting for incoming data.")

     if len(sys.argv) > 1 and sys.argv[1] == "debugPTU":

      groundStation.location.updateLocation([float(sys.argv[2])],[float(sys.argv[3])],[float(sys.argv[4])])
      groundStation.location.geodeticToECEF(True)

      habPayload.location.updateLocation([float(sys.argv[6])],[float(sys.argv[7])],[float(sys.argv[8])])
      habPayload.location.geodeticToECEF(True)

      ptu_ready = 1

      if ptu_ready == 1:
        
       node_rotor.moveGroundStation(groundStation, habPayload, float(sys.argv[5]), True)


     if len(sys.argv) > 1 and sys.argv[1] == "debugPayload":
              
      try:
       [HAB_lat, HAB_lon, HAB_alt, HAB_time] = node_payload.getHABData(True)
       #habPayload.location.updateLocation([float(HAB_lat)],[float(HAB_lon)],[float(HAB_alt)])
       #habPayload.location.geodeticToECEF(False)
       sys.stderr.write(f"HAB Alt: {HAB_alt}\n")
       sys.stderr.write(f"HAB Lat: {HAB_lat}\n") 
       sys.stderr.write(f"HAB Lon: {HAB_lon}\n") 
       sys.stderr.write(f"HAB Time: {HAB_time}\n")
      except:
       print("Payload Message Failure")

     elif len(sys.argv) > 1 and sys.argv[1] == "debugGroundStation":

      try:
       [ground_lat, ground_lon, ground_alt, ground_bearing] = node_ground_station.getGroundData(True)
       groundStation.location.updateLocation([float(ground_lat)],[float(ground_lon)],[float(ground_alt)])
       groundStation.location.geodeticToECEF(False)
      except:
       print("Ground Station Failure")

      sys.stderr.write(f"Ground Alt: {ground_alt}\n")
      sys.stderr.write(f"Ground Lat: {ground_lat}\n")
      sys.stderr.write(f"Ground Lon: {ground_lon}\n")
      sys.stderr.write(f"Ground Time: {ground_time}\n")

     else:
       
      try:
        
       [ground_lat, ground_lon, ground_alt, ground_bearing] = node_ground_station.getGroundData()
       groundStation.location.updateLocation([float(ground_lat)],[float(ground_lon)],[float(ground_alt)])
       groundStation.location.geodeticToECEF(False)
       sys.stderr.write(f"Ground Lat: {ground_lat}\n") 
       sys.stderr.write(f"Ground Lon: {ground_lon}\n") 
       sys.stderr.write(f"Ground Alt: {ground_alt}\n") 
       sys.stderr.write(f"Ground Bearing: {ground_bearing}\n") 
     
      except:
        
       sys.stderr.write("Ground station failure\n")
       ptu_ready = 0
     
      if node_payload.ser.inWaiting() > 20:
        
       try:
         
        [HAB_lat, HAB_lon, HAB_alt, HAB_time] = node_payload.getHABData()
        habPayload.location.updateLocation([float(HAB_lat)],[float(HAB_lon)],[float(HAB_alt)])
        habPayload.location.geodeticToECEF(False)
        sys.stderr.write(f"HAB Lat: {HAB_lat}\n") 
        sys.stderr.write(f"HAB Lon: {HAB_lon}\n")
        sys.stderr.write(f"HAB Alt: {HAB_alt}\n")
        sys.stderr.write(f"HAB Time: {HAB_time}\n") 
        ptu_ready = 1
        
       except:
         
        print("Payload Message Failure")
        ptu_ready = 0
    
      if ptu_ready == 1:
      
       try:
        print("PTU ready")
        if len(sys.argv) > 1 and sys.argv[1] == "debugPointing":
         node_rotor.moveGroundStation(groundStation, habPayload, float(ground_bearing), True, True)
        else:
         node_rotor.moveGroundStation(groundStation, habPayload, float(ground_bearing))
       except:
        print("PTU failure")
        sys.stderr.write("PTU failure\n")

if __name__=="__main__":
    main()
