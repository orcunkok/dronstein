"""Simulated IMU, barometer, magnetometer and GPS from the true state, with noise and bias drift."""
import datetime
import math

import numpy as np

EARTH_RADIUS = 6371000.0  # m, PX4 MapProjection


def earth_field(lat, lon, alt):
    """WMM field at a place, today: NED in gauss."""
    from pygeomag import GeoMag
    now = datetime.date.today()
    year = now.year + (now.timetuple().tm_yday - 1) / 365.25
    r = GeoMag().calculate(glat=lat, glon=lon, alt=alt / 1000, time=year, allow_date_outside_lifespan=True)
    return np.array([r.x, r.y, r.z]) * 1e-5


def isa_pressure(alt):
    """ISA static pressure (hPa) and temperature (degC) at an altitude AMSL (m)."""
    return 1013.25 * (1 - 2.25577e-5 * alt) ** 5.25588, 15.0 - 0.0065 * alt


def ned_to_lla(n, e, d, home):
    """PX4 MapProjection::reproject (azimuthal equidistant) around home -> lat, lon (deg), alt (m)."""
    lat0, lon0 = math.radians(home["lat"]), math.radians(home["lon"])
    x, y = n / EARTH_RADIUS, e / EARTH_RADIUS
    c = math.hypot(x, y)
    if c > 0:
        sin_c, cos_c = math.sin(c), math.cos(c)
        lat = math.asin(cos_c * math.sin(lat0) + x * sin_c * math.cos(lat0) / c)
        lon = lon0 + math.atan2(y * sin_c, c * math.cos(lat0) * cos_c - x * math.sin(lat0) * sin_c)
    else:
        lat, lon = lat0, lon0
    return math.degrees(lat), math.degrees(lon), home["alt"] - d


class Sensors:
    def __init__(self, s, home):
        self.s, self.home = s, home
        self.rng = np.random.default_rng()
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.field = earth_field(home["lat"], home["lon"], home["alt"])

    def imu(self, f_b, w_b, dt):
        """Accelerometer (true specific force, m/s^2) and gyro (rad/s), body FRD, sampled every dt."""
        s, n = self.s, self.rng.standard_normal
        self.accel_bias += s["accel_bias_walk"] * math.sqrt(dt) * n(3)
        self.gyro_bias += s["gyro_bias_walk"] * math.sqrt(dt) * n(3)
        accel = f_b + self.accel_bias + s["accel_noise"] / math.sqrt(dt) * n(3)
        gyro = w_b + self.gyro_bias + s["gyro_noise"] / math.sqrt(dt) * n(3)
        return accel, gyro

    def baro(self, alt):
        """Pressure (hPa), pressure altitude (m), temperature (degC)."""
        p, t = isa_pressure(alt)
        p += self.s["baro_noise"] * self.rng.standard_normal()
        return p, (1 - (p / 1013.25) ** (1 / 5.25588)) / 2.25577e-5, t

    def mag(self, R):
        """Earth field in the body frame (gauss)."""
        return R.T @ self.field + self.s["mag_noise"] * self.rng.standard_normal(3)

    def gps(self, p, v):
        """lat, lon (deg), alt (m AMSL), velocity NED (m/s)."""
        s, n = self.s, self.rng.standard_normal
        p = p + np.array([s["gps_noise_h"], s["gps_noise_h"], s["gps_noise_v"]]) * n(3)
        return (*ned_to_lla(p[0], p[1], p[2], self.home), v + s["gps_noise_vel"] * n(3))
