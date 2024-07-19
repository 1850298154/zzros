
import math
import numpy as np
from src            import entity
from src            import tracker




class NED:

    def __init__(self) -> None:
        self.clear()
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    def clear(self) -> None:
        # longitude and latitude : Radian system, angle system
        self.first_update    = True
        self.home_lon_angle  = None
        self.home_lat_angle  = None
        self.home_lon_radian = None
        self.home_lat_radian = None
        self.home_x          = None
        self.home_y          = None
        self.CONSTANTS_RADIUS_OF_EARTH = 6371000.       # meters (m)
        pass

    # @staticmethod
    # def set_home_gps(lon, lat) -> None:
    def set_home_gps(self, lon, lat) -> None:
        if self.first_update == True:
            self.home_lon_angle     = lon
            self.home_lat_angle     = lat
            self.home_lon_radian    = math.radians(float(lon))
            self.home_lat_radian    = math.radians(float(lat))
            self.home_x             = 0
            self.home_y             = 0
            # self.first_update       = False
            if self.debug_flag == True:
                tracker.debug_tracker.instance.pf('config update')
                tracker.debug_tracker.instance.pf('self.first_update')
                tracker.debug_tracker.instance.pf(self.first_update)
                tracker.debug_tracker.instance.current_position_print()
        else:
            if self.debug_flag == True:
                tracker.debug_tracker.instance.pf('Not the first initialization. Not update')
                tracker.debug_tracker.instance.pf('self.first_update')
                tracker.debug_tracker.instance.pf(self.first_update)
                tracker.debug_tracker.instance.current_position_print()
        pass

    # @staticmethod
    # def gps_to_xy(lat, lon):  # 41.7041358  123.4332115
    def gps_to_xy(self, lat, lon):  # 41.7041358  123.4332115
        lat_rad = math.radians(lat)  # 0.7278744814088395
        lon_rad = math.radians(lon)  # 2.1543159469855286

        sin_lat = math.sin(lat_rad)  # 0.6652842477494599   # 赤道面上投影
        cos_lat = math.cos(lat_rad)  # 0.746590161799923    # 轴投影
        # ref_sin_lat = math.sin(self.ref_lat_rad)
        # ref_cos_lat = math.cos(self.ref_lat_rad)
        ref_sin_lat = math.sin(self.home_lat_radian)
        ref_cos_lat = math.cos(self.home_lat_radian)

        # cos_d_lon = math.cos(lon_rad - self.ref_lon_rad)
        cos_d_lon = math.cos(lon_rad - self.home_lon_radian)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)  # 1.0
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * self.CONSTANTS_RADIUS_OF_EARTH)
        # y = float(k * cos_lat * math.sin(lon_rad - self.ref_lon_rad) * self.CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - self.home_lon_radian) * self.CONSTANTS_RADIUS_OF_EARTH)

        return x, y

    # @staticmethod
    # def xy_to_gps(x, y):
    def xy_to_gps(self, x, y):
        x_rad = float(x) / self.CONSTANTS_RADIUS_OF_EARTH
        y_rad = float(y) / self.CONSTANTS_RADIUS_OF_EARTH
        c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

        # ref_sin_lat = math.sin(self.ref_lat_rad)
        # ref_cos_lat = math.cos(self.ref_lat_rad)
        ref_sin_lat = math.sin(self.home_lat_radian)
        ref_cos_lat = math.cos(self.home_lat_radian)

        if abs(c) > 0:
            sin_c = math.sin(c)
            cos_c = math.cos(c)

            lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
            # lon_rad = (self.ref_lon_rad + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))
            lon_rad = (self.home_lon_radian + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))

            lat = math.degrees(lat_rad)
            lon = math.degrees(lon_rad)

        else:
            # lat = math.degrees(self.ref_lat)
            # lon = math.degrees(self.ref_lon)
            lon = math.degrees(self.home_lon_angle)
            lat = math.degrees(self.home_lat_angle)

        return float(lat), float(lon)  # (40N, 116E)


instance = NED()

