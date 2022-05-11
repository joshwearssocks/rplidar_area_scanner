#!/usr/bin/python3

import socket
import signal
import math
from typing import Tuple, List
import pathlib
import argparse

HOST = '192.168.2.69'
PORT = 6666

TICKS_PER_REV = 20400
REPORTED_TICKS_PER_REV = 20390
#TICKS_PER_REV = 20390
UP_PITCH = math.radians(90.65)
Z_TILT = math.radians(0.48)
#Z_TILT = 0.4
FOWARD_OFFSET_MM = 14 #12
point_num = 0
# HACK to account for mismatch in ticks per rev in firmware and script
tick_rollovers = 0
prev_ticks = 0

def parse_point(line: str) -> Tuple[float,float,float, float,float,float, int, int, float]:
    """Convert a line to a 3D cartesian point.
             __
           (  --)-          <-- FORWARD_OFFSET
        |----------------|

    """
    global point_num
    global tick_rollovers
    global prev_ticks
    
    pitch_str, dist_str, quality_str, ticks_str = line.split(',')
    dist = float(dist_str)
    ticks = float(ticks_str)
    pitch = math.radians(float(pitch_str)) + UP_PITCH
    if dist == 0:
        return None
    if ticks < prev_ticks:
        tick_rollovers += 1
    prev_ticks = ticks
    total_ticks = 20390 * tick_rollovers + ticks
    theta_pre_tilt = math.radians(360 * (total_ticks % TICKS_PER_REV) / TICKS_PER_REV)
    phi_pre_tilt = pitch
    dtheta = math.atan(math.sin(Z_TILT)*math.cos(phi_pre_tilt) / math.sin(phi_pre_tilt))
    theta = theta_pre_tilt + dtheta
    try:
        meat = math.sin(phi_pre_tilt) / math.cos(dtheta)
        phi = math.asin(min(1,max(meat,-1)))
        phi = phi_pre_tilt
    except ValueError:
        print(f"phi / distance: {phi_pre_tilt} / {dist_str}")
        print(f"dtheta: {math.degrees(dtheta)}")
        print(f"cos(dtheta): {math.cos(dtheta)}")
        print(f"sin(phi): {math.sin(phi_pre_tilt)}")
        print(f"abs: {abs(math.sin(phi_pre_tilt) / math.cos(dtheta))}")
        raise
    #phi = math.asin(math.sqrt(math.sin(phi_pre_tilt)**2+math.sin(math.radians(Z_TILT))**2))
    
    z = dist * math.cos(phi)
    x = dist * math.sin(phi) * math.cos(theta) + FOWARD_OFFSET_MM * math.cos(theta)
    y = dist * math.sin(phi) * math.sin(theta) + FOWARD_OFFSET_MM * math.sin(theta)
    point_num += 1
    hemisphere = 0
    if total_ticks % TICKS_PER_REV > TICKS_PER_REV / 2:
        hemisphere = 1
    quality = 100/dist
    
    nz = -1 * math.cos(phi)
    nx = -1 * math.sin(phi) * math.cos(theta)
    ny = -1 * math.sin(phi) * math.sin(theta)
    
    return (x, y, z, nx, ny, nz, point_num, hemisphere, quality)
    
def write_ply_file(ply_path: pathlib.Path, points: List[Tuple[float,float,float, float,float,float, int, int, float]]) -> None:
    with open(ply_path, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {len(points)}\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property float nx\n')
        f.write('property float ny\n')
        f.write('property float nz\n')
        f.write('property uchar red\n')
        f.write('property uchar green\n')
        f.write('property uchar blue\n')
        f.write('property float quality\n')
        f.write('end_header\n')
        for x, y, z, nx, ny, nz, num, hemisphere, quality in points:
            col = int(150 * num / point_num) + 100
            f.write(f'{x:.3f} {y:.3f} {z:.3f} {nx:.3f} {ny:.3f} {nz:.3f} {355-col} {hemisphere*255} {col} {quality}\n')
            
def collect_points(in_file: pathlib.Path) -> None:
    point_list = []

    # Capture the points
    #with open('raw_capture.txt', 'r') as raw_f:
    with open(in_file, 'r') as raw_f:
        lines = raw_f.readlines()

    for line in lines:
        tup = parse_point(line)
        if tup:
            point_list.append(tup)
                
    # Write the points to a ply file
    write_ply_file(f'{in_file.stem}.ply', point_list)
    
parser = argparse.ArgumentParser()
parser.add_argument('text_file', type=str, help="Text file containing point strings")
p = parser.parse_args()

if __name__ == '__main__':
    collect_points(pathlib.Path(p.text_file))


