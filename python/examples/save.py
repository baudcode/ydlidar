import gzip
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
from pathlib import Path
import time

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
laser.setlidaropt(ydlidar.LidarPropSampleRate, 10)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
scan = ydlidar.LaserScan()

# angle increment: 0.0028508100658655167

ret = laser.initialize()
output_file = Path("scans", f"lidar_scan_{time.time()}.csv.gz")
output_file.parent.mkdir(exist_ok=True, parents=True)

try:
    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
        prev = default_timer()
        with gzip.open(output_file, "wb") as writer:
            writer.write(b"ts;points;inc\n")

            while ret and ydlidar.os_isOk():
                r = laser.doProcessSimple(scan)
                current = default_timer()
              
                if r:
                    points = []
                    for p in scan.points:
                        intensity = p.intensity
                        angle = p.angle
                        rangee = p.range
                        points.append(rangee)

                    points_str = ",".join(map(str, points))
                    writer.write(bytes(f"{scan.stamp};{points_str};{scan.config.angle_increment}\n", 'utf-8'))

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
