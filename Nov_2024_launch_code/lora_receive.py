import time 
import serial 
import sx126x_class as sx126x
import sys

def main():
    
    print("Press Ctrl+C to exit")
    sys.stderr.write("Press Ctrl+C to exit\n")
    
    node_usb = sx126x.sx126x(serial_num = "/dev/ttyUSB0",freq=868,addr=101,power=22,rssi=True)
    node_s0 = sx126x.sx126x(serial_num = "/dev/ttyS0",freq=868,addr=100,power =22,rssi=True)
    node_rotor = sx126x.sx126x(serial_num = "/dev/ttyUSB1", freq=868,addr=100,power=22,rssi=True)

    ptu_ready = 0
    
    while True:    
     print("Waiting for ground data.")
     sys.stderr.write("Waiting for ground data.\n")
     try:
      [ground_lat, ground_lon, ground_alt, ground_bearing] = node_usb.getGroundData()
      print("Ground Lat: ", ground_lat)
      sys.stderr.write(f"Ground Lat: {ground_lat}\n") #, ground_lat)
      print("Ground Lon: ", ground_lon)
      sys.stderr.write(f"Ground Lon: {ground_lon}\n") #, ground_lon)
      print("Ground Alt: ", ground_alt)
      sys.stderr.write(f"Ground Alt: {ground_alt}\n") #, ground_alt)
      print("Ground Bearing: ", ground_bearing)
      sys.stderr.write(f"Ground Bearing: {ground_bearing}\n") #, ground_bearing)
     
     except:
      print("Ground station failure")
      sys.stderr.write("Ground station failure\n")
     
     print(node_s0.ser.inWaiting())
     if node_s0.ser.inWaiting() > 20:
      [HAB_lat, HAB_lon, HAB_alt, HAB_time] = node_s0.getHABData()
      print("HAB Lat: ", HAB_lat)
      sys.stderr.write(f"HAB Lat: {HAB_lat}\n") # , HAB_lat)
      print("HAB Lon: ", HAB_lon)
      sys.stderr.write(f"HAB Lon: {HAB_lon}\n") # , HAB_lon)
      print("HAB Alt: ", HAB_alt)
      sys.stderr.write(f"HAB Alt: {HAB_alt}\n") # , HAB_alt)
      print("HAB Time: ", HAB_time)
      sys.stderr.write(f"HAB Time: {HAB_time}\n") # , HAB_time)
      ptu_ready = 1
     
     if ptu_ready == 1:
      
      try:
       node_rotor.moveGroundStation(float(ground_lat), float(ground_lon), float(ground_alt), float(HAB_lat), float(HAB_lon), float(HAB_alt), float(ground_bearing))
      except:
       print("PTU failure")
       sys.stderr.write("PTU failure\n")

if __name__=="__main__":
    main()
