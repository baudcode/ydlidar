import os
import traceback
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import logging
from timeit import default_timer

logging.basicConfig(level=logging.DEBUG)

RMAX = 32.0


fig = plt.figure()
fig.canvas.set_window_title('YDLidar LIDAR Monitor')
lidar_polar = plt.subplot(polar=True)
lidar_polar.autoscale_view(True, True, True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)
ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"
for key, value in ports.items():
    port = value

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)  # TYPE_LIDAR
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 20)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
scan = ydlidar.LaserScan()

ret = laser.initialize()

try:
    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
        prev = default_timer()

        while ret and ydlidar.os_isOk():
            r = laser.doProcessSimple(scan)
            current = default_timer()
            if r:
                logging.info(
                    f"scan received[{scan.stamp} - elapsed={(current - prev):.3f}s] {scan.points.size()} ranges is [{1.0/scan.config.scan_time}] Hz")
            else:
                logging.warning("Failed to get Lidar Data.")

            prev = current
        laser.turnOff()
except Exception as e:
    traceback.print_exc()
finally:
    print("disconnecting...")
    laser.disconnecting()
