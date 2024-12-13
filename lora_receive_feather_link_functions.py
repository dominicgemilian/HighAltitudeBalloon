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
    
    node_usb = ground_station.ground_station(serial_num = "/dev/ttyUSB0")
    node_s0 = ground_station.ground_station(serial_num = "/dev/ttyACM0")
    node_rotor = ground_station.ground_station(serial_num = "/dev/ttyUSB1")

    ptu_ready = 0
    
    groundStation = terminal.terminal()
    habPayload = terminal.terminal()

    while True:    

     print("Waiting for ground data.")
     sys.stderr.write("Waiting for ground data.\n")

     if len(sys.argv) > 1 and sys.argv[1] == "debugPTU":

      groundStation.location.updateLocation([float(sys.argv[2])],[float(sys.argv[3])],[float(sys.argv[4])])
      groundStation.location.geodeticToECEF(True)

      habPayload.location.updateLocation([float(sys.argv[6])],[float(sys.argv[7])],[float(sys.argv[8])])
      habPayload.location.geodeticToECEF(True)

      ptu_ready = 1

      if ptu_ready == 1:

      # try:
       print("PTU ready")
       node_rotor.moveGroundStation(groundStation, habPayload, float(sys.argv[5]), True)
      # except:
      #  print("PTU failure")

     if len(sys.argv) > 1 and sys.argv[1] == "debugPayload":

      groundStation.location.updateLocation([float(sys.argv[2])],[float(sys.argv[3])],[float(sys.argv[4])])
      groundStation.location.geodeticToECEF(True)
              
      try:
       [HAB_lat, HAB_lon, HAB_alt, HAB_time] = node_s0.getHABData()
       habPayload.location.updateLocation([float(HAB_lat)],[float(HAB_lon)],[float(HAB_alt)])
       habPayload.location.geodeticToECEF(True)
      except:
       print("Payload Message Failure")
       HAB_alt = np.NaN
       HAB_lat = np.NaN
       HAB_lon = np.NaN
       HAB_time = np.NaN

      sys.stderr.write(f"HAB Alt: {HAB_alt}\n")
      sys.stderr.write(f"HAB Lat: {HAB_lat}\n") 
      sys.stderr.write(f"HAB Lon: {HAB_lon}\n") 
      sys.stderr.write(f"HAB Time: {HAB_time}\n")
      ptu_ready = 1

      if ptu_ready == 1:
      
      # try:
       print("PTU ready")
       node_rotor.moveGroundStation(groundStation, habPayload, float(sys.argv[5]), True)
      # except:
      #  print("PTU failure")

     elif len(sys.argv) > 1 and sys.argv[1] == "debugGroundStation":

      try:
       [ground_lat, ground_lon, ground_alt, ground_bearing] = node_usb.getGroundData(True)
       groundStation.location.updateLocation([float(ground_lat)],[float(ground_lon)],[float(ground_alt)])
       groundStation.location.geodeticToECEF(True)
      except:
       print("Ground Station Failure")
       ground_alt = np.NaN
       ground_lat = np.NaN
       ground_lon = np.NaN
       ground_time = np.NaN
       
      habPayload.location.updateLocation([float(sys.argv[2])],[float(sys.argv[3])],[float(sys.argv[4])])
      habPayload.location.geodeticToECEF(True)

      sys.stderr.write(f"Ground Alt: {ground_alt}\n")
      sys.stderr.write(f"Ground Lat: {ground_lat}\n")
      sys.stderr.write(f"Ground Lon: {ground_lon}\n")
      sys.stderr.write(f"Ground Time: {ground_time}\n")
      ptu_ready = 1

      if ptu_ready == 1:

       try:
        print("PTU ready")
        node_rotor.moveGroundStation(groundStation, habPayload, float(ground_bearing), True)
       except:
        print("PTU failure")
        sys.stderr.write("PTU failure\n")

     else:
      try:
       [ground_lat, ground_lon, ground_alt, ground_bearing] = node_usb.getGroundData()
       groundStation.location.updateLocation([float(ground_lat)],[float(ground_lon)],[float(ground_alt)])
       groundStation.location.geodeticToECEF(True)
       sys.stderr.write(f"Ground Lat: {ground_lat}\n") 
       sys.stderr.write(f"Ground Lon: {ground_lon}\n") 
       sys.stderr.write(f"Ground Alt: {ground_alt}\n") 
       sys.stderr.write(f"Ground Bearing: {ground_bearing}\n") 
     
      except:
       sys.stderr.write("Ground station failure\n")
       ptu_ready = 0
     
      if node_s0.ser.inWaiting() > 20:
       try:
        [HAB_lat, HAB_lon, HAB_alt, HAB_time] = node_s0.getHABData()
        habPayload.location.updateLocation([float(HAB_lat)],[float(HAB_lon)],[float(HAB_alt)])
        habPayload.location.geodeticToECEF(True)
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
        node_rotor.moveGroundStation(groundStation, habPayload, float(ground_bearing))
       except:
        print("PTU failure")
        sys.stderr.write("PTU failure\n")

if __name__=="__main__":
    main()
