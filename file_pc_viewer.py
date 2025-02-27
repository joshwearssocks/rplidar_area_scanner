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
R_IDX = 0
T_IDX = 1
P_IDX = 2
X_IDX = 0
Y_IDX = 1
Z_IDX = 2

@dataclasses.dataclass
class ScanParams:
    ticks_per_rev: int = 25600
    up_pitch: float = 90
    z_tilt: float = 0 #0.48
    forward_offset: float = 0
    sideways_offset: float = 0
    end_point_trim: int = 20
    
class PointCloudParser:
    def __init__(self, txt_path: pathlib.Path):
        self.txt_path = txt_path

        self.params = ScanParams()
        self.raw_rtt = np.array([]) # radius, theta, ticks
        self.rtp = np.array([]) # radius, theta, phi
        self.xyz = np.array([]) # points
        self.nxyz = np.array([]) # normals
        self.rgba = np.array([]) # colors for pyqtgraph

        self.xyz_trimmed = np.array([])
        self.rgba_trimmed = np.array([])

        self.r_hist_y = np.array([])
        self.r_hist_x = np.array([])
        
        # Parse the file into lists
        self.read_txt_file(self.txt_path)
        self.parse_all_points()
        
    def read_txt_file(self, txt_path: pathlib.Path):
        with open(txt_path, 'r') as txt_file:
            lines = txt_file.readlines()

        self.raw_rtt = np.empty((len(lines),3))

        for i in range(len(lines)):
            pitch_str, dist_str, quality_str, ticks_str = lines[i].split(',')
            self.raw_rtt[i,R_IDX] = float(dist_str)
            self.raw_rtt[i,T_IDX] = math.radians(float(pitch_str))
            self.raw_rtt[i,P_IDX] = int(ticks_str)

        # Compute radius histogram
        self.r_hist_y, self.r_hist_x = np.histogram(self.raw_rtt[:,R_IDX], 1000)

    def apply_scan_params(self):
        radius, theta, ticks = np.moveaxis(self.raw_rtt, -1, 0)

        self.rtp = np.empty_like(self.raw_rtt)

        pre_selector = ((slice(None),) * self.rtp.ndim)[:-1]

        self.rtp[(*pre_selector, R_IDX)] = radius
        self.rtp[(*pre_selector, T_IDX)] = theta + math.radians(self.params.up_pitch)
        self.rtp[(*pre_selector, P_IDX)] = 2 * np.pi * ticks / self.params.ticks_per_rev
    
    def to_xyz(self):
        r, t, p = np.moveaxis(self.rtp, -1, 0)

        self.xyz = np.empty_like(self.rtp)

        pre_selector = ((slice(None),) * self.xyz.ndim)[:-1]

        self.xyz[(*pre_selector, X_IDX)] = (r * np.sin(t) + self.params.forward_offset) * np.cos(p) + (np.sin(p) * self.params.sideways_offset)
        self.xyz[(*pre_selector, Y_IDX)] = (r * np.sin(t) + self.params.forward_offset) * np.sin(p) - (np.cos(p) * self.params.sideways_offset)
        self.xyz[(*pre_selector, Z_IDX)] = r * np.cos(t)

    def parse_normals(self):
        r, t, p = np.moveaxis(self.rtp, -1, 0)

        self.nxyz = np.empty_like(self.rtp)

        pre_selector = ((slice(None),) * self.nxyz.ndim)[:-1]

        self.nxyz[(*pre_selector, X_IDX)] = -1 * np.cos(t)
        self.nxyz[(*pre_selector, Y_IDX)] = -1 * np.sin(t) * np.cos(p)
        self.nxyz[(*pre_selector, Z_IDX)] = -1 * np.sin(t) * np.sin(p)

    def parse_colors(self):
        radius, _, ticks = np.moveaxis(self.raw_rtt, -1, 0)

        self.rgba = np.empty((radius.shape[0], 4))

        pre_selector = ((slice(None),) * self.rgba.ndim)[:-1]

        self.rgba[(*pre_selector, 0)] = 1 - (radius / 4000)
        self.rgba[(*pre_selector, 1)] = ticks / self.params.ticks_per_rev
        self.rgba[(*pre_selector, 2)] = 1 - (ticks / self.params.ticks_per_rev)
        self.rgba[(*pre_selector, 3)] = 0.8

    def trim(self):
        self.xyz_trimmed = self.xyz[self.params.end_point_trim:,:]
        self.rgba_trimmed = self.rgba[self.params.end_point_trim:,:]
        
    def parse_all_points(self):
        self.apply_scan_params()
        self.to_xyz()
        self.parse_normals()
        self.parse_colors()
        self.trim()
            
    def write_ply_file(self) -> None:
        with open(f"{self.txt_path.stem}.ply", 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {self.xyz.shape[0]}\n')
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
                
            for i in range(self.xyz.shape[0]):
                f.write(f'{self.xyz[i,0]:.3f} {self.xyz[i,1]:.3f} {self.xyz[i,2]:.3f} '
                        f'{self.nxyz[i,0]:.3f} {self.nxyz[i,1]:.3f} {self.nxyz[i,2]:.3f} '
                        f'{(self.rgba[i,0]*255):.0f} {(self.rgba[i,1]*255):.0f} {(self.rgba[i,2]*255):.0f}\n')

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
        self.spin_up_pitch.setMinimum(-180.)
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
        self.spin_end_point_trim.setMaximum(100000)
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

        self.hist_widget = pg.GraphicsLayoutWidget(show=True)
        self.hist_plt = self.hist_widget.addPlot()
        self.hist_plt.plot(self.pcp.r_hist_x, self.pcp.r_hist_y, stepMode="center", fillLevel=0, fillOutline=True, brush=(0,0,255,150))
        
        self.sidebar_layout = QtWidgets.QVBoxLayout()
        self.sidebar_layout.addLayout(self.form_layout)
        self.sidebar_layout.addWidget(self.hist_widget)
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

        self.scatter = gl.GLScatterPlotItem(pos=self.pcp.xyz_trimmed, color=self.pcp.rgba_trimmed, size=0.01, pxMode=False)

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
        self.scatter.setData(pos=self.pcp.xyz_trimmed, color=self.pcp.rgba_trimmed)
        
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
