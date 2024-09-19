import math

def deg_to_rad(degrees):
    return degrees * (math.pi / 180)

def rad_to_deg(radians):
    return radians * (180 / math.pi)

def geodetic_to_ecef(latitude, longitude, altitude):
    # Constants for the WGS84 ellipsoid
    a = 6378137.0  # Semi-major axis
    b = 6356752.314245  # Semi-minor axis
    e_squared = 1 - (b**2 / a**2)  # Eccentricity squared

    latitude_rad = deg_to_rad(latitude)
    longitude_rad = deg_to_rad(longitude)

    sin_lat = math.sin(latitude_rad)
    cos_lat = math.cos(latitude_rad)
    sin_lon = math.sin(longitude_rad)
    cos_lon = math.cos(longitude_rad)

    N = a / math.sqrt(1 - e_squared * sin_lat**2)

    x = (N + altitude) * cos_lat * cos_lon
    y = (N + altitude) * cos_lat * sin_lon
    z = (N * (1 - e_squared) + altitude) * sin_lat

    return x, y, z

def azimuth_elevation_angle(observer_lat, observer_lon, observer_alt, target_lat, target_lon, target_alt, bearing):
    observer_ecef = geodetic_to_ecef(observer_lat, observer_lon, observer_alt)
    target_ecef = geodetic_to_ecef(target_lat, target_lon, target_alt)

    observer_x, observer_y, observer_z = observer_ecef
    target_x, target_y, target_z = target_ecef

    delta_x = target_x - observer_x
    delta_y = target_y - observer_y
    delta_z = target_z - observer_z

    # Convert ECEF to NED (North-East-Down) coordinates
    sin_lat = math.sin(deg_to_rad(observer_lat))
    cos_lat = math.cos(deg_to_rad(observer_lat))
    sin_lon = math.sin(deg_to_rad(observer_lon))
    cos_lon = math.cos(deg_to_rad(observer_lon))

    ned_x = -sin_lat * cos_lon * delta_x - sin_lat * sin_lon * delta_y + cos_lat * delta_z
    ned_y = -sin_lon * delta_x + cos_lon * delta_y
    ned_z = cos_lat * cos_lon * delta_x + cos_lat * sin_lon * delta_y + sin_lat * delta_z

    # Calculate azimuth angle
    azimuth_rad = math.atan2(ned_y, ned_x)
    azimuth_deg = rad_to_deg(azimuth_rad)

    # Calculate elevation angle
    distance = math.sqrt(ned_x**2 + ned_y**2 + ned_z**2)
    elevation_rad = math.asin(ned_z / distance)
    elevation_deg = rad_to_deg(elevation_rad)

    # Adjust azimuth angle by bearing
    adjusted_azimuth_deg = azimuth_deg + bearing
    adjusted_azimuth_deg = adjusted_azimuth_deg % 360  # Wrap around to 0-360 range

    print(f"Azimuth: {adjusted_azimuth_deg} degrees")
    print(f"Elevation: {elevation_deg} degrees")

    return adjusted_azimuth_deg, elevation_deg
