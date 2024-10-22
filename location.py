import math
import navpy

# Location class for earth terminals, space stations, and general earth-centric coordinates
class location():
  
  def __init__(self):
    
    self.latitude = []
    self.longitude = []
    self.altitude = []
    self.xECEF = []
    self.yECEF = []
    self.zECEF = []

  def updateLocation(self, latitude, longitude, altitude):

    # Geodetic latitude [degrees]
    self.latitude = latitude
    # Geodetic longitude [degrees]
    self.longitude = longitude
    # Geodetic altitude [meters]
    self.altitude = altitude

  def geodeticToECEF(self, debugFlag = False): 
      
      rEarth = 6378137                 # meters
      e = 8.1819190842622e-2           # Eccentricity
      rEarthSquared = rEarth*rEarth    # Earth radius squares
      eSquared = e*e                   # Eccentricity squared

      N = []                           # Radius of curvature in prime vertical direction [meters]
      
      self.xECEF = []
      self.yECEF = []
      self.zECEF = []
      
      # Loop through all locations
      for locationIndex in range(len(self.altitude)):
      
        # Calculate the value of N [meters]
        N.append(rEarth/math.sqrt(1 - eSquared*math.pow(math.sin(math.radians(self.latitude[locationIndex])),2)))   
        
        # Calculate the ECEF coordinates
        ecef = navpy.lla2ecef(self.latitude[locationIndex], self.longitude[locationIndex], self.altitude[locationIndex])
        
        # ECEF x-coordinate [meters]                                     # 
        self.xECEF.append(ecef[0])

        # ECEF y-coordinate [meters]
        self.yECEF.append(ecef[1])

        # ECEF z-coordinate [meters]
        self.zECEF.append(ecef[2])

      if (debugFlag):

        print("=========================== Geodetic Location Information ===========================")
        print("Latitude: ", self.latitude[locationIndex], "degrees")
        print("Longitude: ", self.longitude[locationIndex], "degrees")
        print("Altitude: ", self.altitude[locationIndex], "meters")
            
        print("=========================== WGS-84 ECEF Location Information ========================")
        print("ECEF X Coordinate: ", self.xECEF[locationIndex], "meters")
        print("ECEF Y Coordinate: ", self.yECEF[locationIndex], "meters")
        print("ECEF Z Coordinate: ", self.zECEF[locationIndex], "meters")







