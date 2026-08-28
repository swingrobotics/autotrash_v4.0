import math


class LocalENUConverter:
    EARTH_RADIUS_M = 6378137.0

    def __init__(self, origin_latitude, origin_longitude, origin_altitude=0.0):
        self.origin_latitude = float(origin_latitude)
        self.origin_longitude = float(origin_longitude)
        self.origin_altitude = float(origin_altitude or 0.0)
        self._origin_latitude_radians = math.radians(self.origin_latitude)

    def to_enu(self, latitude, longitude, altitude=None):
        latitude_delta = math.radians(float(latitude) - self.origin_latitude)
        longitude_delta = math.radians(float(longitude) - self.origin_longitude)
        east = self.EARTH_RADIUS_M * longitude_delta * math.cos(self._origin_latitude_radians)
        north = self.EARTH_RADIUS_M * latitude_delta
        up = float(altitude or self.origin_altitude) - self.origin_altitude
        return east, north, up

    def to_dict(self):
        return {
            "origin_latitude": self.origin_latitude,
            "origin_longitude": self.origin_longitude,
            "origin_altitude": self.origin_altitude,
        }
