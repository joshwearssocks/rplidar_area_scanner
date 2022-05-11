"""
pyqtgraph
pyside6
pyopengl
"""

import numpy as np
import argparse
import math

import pathlib

import dataclasses

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph import functions as fn
from pyqtgraph.Qt import QtCore, QtWidgets

from typing import List

REPORTED_TICKS_PER_REV = 20390

@dataclasses.dataclass
class ScanParams:
    ticks_per_rev: int = 20400
    up_pitch: float = 90.65
    z_tilt: float = 0.54 #0.48
    forward_offset: float = 14.5
    sideways_offset: float = -1.0
    end_point_trim: int = 20
    
class PointCloudParser:
    def __init__(self, txt_path: pathlib.Path):
        self.txt_path = txt_path
        # Parse the file into lists
        self.pitches: List[float] = []
        self.distances: List[float] = []
        self.qualities: List[int] = []
        self.ticks: List[int] = []
        self.read_txt_file(self.txt_path)
            
        self.points = np.zeros((len(self.pitches),3))
        self.normals = np.zeros((len(self.pitches),3))
        self.colors = np.zeros((len(self.pitches),4))
            
        self.prev_ticks = 0
        self.tick_rollovers = 0
        
        self.params = ScanParams()
        
        self.parse_all_points()
        
    def read_txt_file(self, txt_path: pathlib.Path):
        with open(txt_path, 'r') as txt_file:
            lines = txt_file.readlines()
        for line in lines:
            pitch_str, dist_str, quality_str, ticks_str = line.split(',')
            self.pitches.append(float(pitch_str))
            self.distances.append(float(dist_str))
            self.qualities.append(int(quality_str))
            self.ticks.append(int(ticks_str))
    
    def parse_point(self, pitch, dist, ticks):
        """Convert a line to a 3D cartesian point.
                 __
               (  --)-          <-- FORWARD_OFFSET
            |----------------|

        """
        if dist == 0:
            return None
            
        pitch_corr = math.radians(pitch) + math.radians(self.params.up_pitch)
        
        if abs(dist * math.sin(pitch_corr)) < 30:
            return None
        
        if ticks < self.prev_ticks:
            self.tick_rollovers += 1
        self.prev_ticks = ticks
        total_ticks = REPORTED_TICKS_PER_REV * self.tick_rollovers + ticks
        theta_pre_tilt = math.radians(360 * (total_ticks % self.params.ticks_per_rev) / self.params.ticks_per_rev)
        phi_pre_tilt = pitch_corr
        dtheta = math.atan(math.sin(math.radians(self.params.z_tilt))*math.cos(phi_pre_tilt) / math.sin(phi_pre_tilt))
        theta = theta_pre_tilt + dtheta
        #phi = math.atan(math.sin(phi_pre_tilt)/(math.cos(math.radians(self.params.z_tilt))*math.cos(phi_pre_tilt)*math.cos(dtheta)))
        phi = phi_pre_tilt
        z = dist * math.cos(phi)
        x = (dist * math.sin(phi) + self.params.forward_offset) * math.cos(theta) + (math.sin(theta) * self.params.sideways_offset)
        y = (dist * math.sin(phi) + self.params.forward_offset) * math.sin(theta) - (math.cos(theta) * self.params.sideways_offset)
        hemisphere = 0
        if total_ticks % self.params.ticks_per_rev > self.params.ticks_per_rev / 2:
            hemisphere = 1
        
        nz = -1 * math.cos(phi)
        nx = -1 * math.sin(phi) * math.cos(theta)
        ny = -1 * math.sin(phi) * math.sin(theta)
        
        return ((x, y, z), (nx, ny, nz), hemisphere)
        
    def parse_all_points(self):
        self.prev_ticks = 0
        self.tick_rollovers = 0
        
        for i in range(len(self.pitches)):
            # Filter out the first and last 20 points
            if i < self.params.end_point_trim or len(self.pitches) - i < self.params.end_point_trim:
                self.points[i,:] = (0,0,0)
                self.normals[i,:] = (0,0,0)
                self.colors[i,:] = (0,0,0,0)
                continue
            tup = self.parse_point(self.pitches[i], self.distances[i], self.ticks[i])
            if not tup:
                continue
            self.points[i,:] = tup[0]
            self.normals[i,:] = tup[1]
            col = int(150 * i / len(self.pitches)) + 100
            self.colors[i,:] = ((355-col)/256, (tup[2]*255)/256, col/256, 0.8)
            
    def write_ply_file(self) -> None:
        with open(f"{self.txt_path.stem}.ply", 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(self.pitches)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property float nx\n')
            f.write('property float ny\n')
            f.write('property float nz\n')
            f.write('property uchar red\n')
            f.write('property uchar green\n')
            f.write('property uchar blue\n')
            f.write('end_header\n')
                
            for i in range(len(self.pitches)):
                f.write(f'{self.points[i,0]:.3f} {self.points[i,1]:.3f} {self.points[i,2]:.3f} '
                        f'{self.normals[i,0]:.3f} {self.normals[i,1]:.3f} {self.normals[i,2]:.3f} '
                        f'{self.colors[i,0]:.3f} {self.colors[i,1]:.3f} {self.colors[i,2]:.3f}')

class ScanVisualizer(QtWidgets.QWidget):
    def __init__(self, pcp: PointCloudParser):
        super().__init__()
        
        """
        ticks_per_rev: int = 20400
        up_pitch: float = 90.65
        z_tilt: float = 0.48
        forward_offset: float = 14
        """
        
        self.gl = gl.GLViewWidget()
        
        self.pcp = pcp
        
        self.spin_ticks_per_rev = QtWidgets.QSpinBox()
        self.spin_ticks_per_rev.setMaximum(30000)
        self.spin_ticks_per_rev.setValue(self.pcp.params.ticks_per_rev)
        
        self.spin_up_pitch = QtWidgets.QDoubleSpinBox()
        self.spin_up_pitch.setValue(self.pcp.params.up_pitch)
        self.spin_up_pitch.setSingleStep(0.05)
        
        self.spin_z_tilt = QtWidgets.QDoubleSpinBox()
        self.spin_z_tilt.setValue(self.pcp.params.z_tilt)
        self.spin_z_tilt.setSingleStep(0.02)
        
        self.spin_forward_offset = QtWidgets.QDoubleSpinBox()
        self.spin_forward_offset.setSingleStep(0.1)
        self.spin_forward_offset.setValue(self.pcp.params.forward_offset)
        
        self.spin_sideways_offset = QtWidgets.QDoubleSpinBox()
        self.spin_sideways_offset.setMinimum(-10.)
        self.spin_sideways_offset.setSingleStep(0.1)
        self.spin_sideways_offset.setValue(self.pcp.params.sideways_offset)
        
        self.spin_end_point_trim = QtWidgets.QSpinBox()
        self.spin_end_point_trim.setMaximum(100)
        self.spin_end_point_trim.setValue(self.pcp.params.end_point_trim)
        
        self.form_layout = QtWidgets.QFormLayout()
        self.form_layout.addRow(("Ticks per Rev"), self.spin_ticks_per_rev)
        self.form_layout.addRow(("Up Pitch"), self.spin_up_pitch)
        self.form_layout.addRow(("Z Tilt"), self.spin_z_tilt)
        self.form_layout.addRow(("Forward Offset"), self.spin_forward_offset)
        self.form_layout.addRow(("Sideways Offset"), self.spin_sideways_offset)
        self.form_layout.addRow(("Endpoint Trim"), self.spin_end_point_trim)
        
        self.button_open = QtWidgets.QPushButton("Open")
        self.button_export = QtWidgets.QPushButton("Export")
        
        self.sidebar_layout = QtWidgets.QVBoxLayout()
        self.sidebar_layout.addLayout(self.form_layout)
        self.sidebar_layout.addWidget(self.button_open)
        self.sidebar_layout.addWidget(self.button_export)

        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.addWidget(self.gl, 5)
        self.layout.addLayout(self.sidebar_layout, 1)

        self.spin_ticks_per_rev.valueChanged.connect(self.update)
        self.spin_up_pitch.valueChanged.connect(self.update)
        self.spin_z_tilt.valueChanged.connect(self.update)
        self.spin_forward_offset.valueChanged.connect(self.update)
        self.spin_sideways_offset.valueChanged.connect(self.update)
        self.spin_end_point_trim.valueChanged.connect(self.update)
        self.button_open.released.connect(self.open)
        self.button_export.released.connect(self.save)

        self.scatter = gl.GLScatterPlotItem(pos=self.pcp.points, color=self.pcp.colors, size=0.01, pxMode=False)

        self.gl.addItem(self.scatter)

    @QtCore.Slot()
    def update(self):
        self.pcp.params.ticks_per_rev = self.spin_ticks_per_rev.value()
        self.pcp.params.up_pitch = self.spin_up_pitch.value()
        self.pcp.params.z_tilt = self.spin_z_tilt.value()
        self.pcp.params.forward_offset = self.spin_forward_offset.value()
        self.pcp.params.sideways_offset = self.spin_sideways_offset.value()
        self.pcp.params.end_point_trim = self.spin_end_point_trim.value()
        
        self.pcp.parse_all_points()
        self.scatter.setData(pos=self.pcp.points, color=self.pcp.colors)
        
    @QtCore.Slot()
    def open(self):
        dlg = QtWidgets.QFileDialog()
        dlg.setNameFilter("Text files (*.txt)")
        dlg.setFileMode(QtWidgets.QFileDialog.ExistingFile)
        if dlg.exec():
            file_name = dlg.selectedFiles()[0]
            self.pcp.__init__(file_name)
        self.update()
        
    @QtCore.Slot()
    def save(self):
        self.pcp.write_ply_file()
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('text_file', type=str, help="Text file containing point strings")
    p = parser.parse_args()
    pcp = PointCloudParser(pathlib.Path(p.text_file))

    app = pg.mkQApp("3D Scanner Parameter Visualizer")
    widget = ScanVisualizer(pcp)
    widget.resize(800, 600)
    widget.show()
    pg.exec()
