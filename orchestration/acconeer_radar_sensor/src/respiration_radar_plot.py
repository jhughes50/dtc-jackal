#!/usr/bin/env python3

# Author: Eric Eaton, University of Pennsylvania
# Date: 2024-09-07
#
# ROS node for plotting Presence and Respiration data from the XE125 board
# Code partially based on Acconeer's breathing detection example app and adapted for ROS

import rospy
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets  # Updated to QtWidgets to fix QApplication issue
from PySide6.QtGui import QFont
from acconeer_radar_sensor.msg import Presence, Respiration
from std_msgs.msg import Float32, String
import acconeer.exptool as et


class PGUpdater:
    def __init__(self):
        # Initialize distances and other configurations here, or based on ROS messages
        self.distances = None  # Update distances based on received data

    def setup(self, win):
        # Define pens, font, and set up the plots
        blue_color = et.utils.color_cycler(0)
        orange_color = et.utils.color_cycler(1)
        brush = et.utils.pg_brush_cycler(0)
        self.blue = dict(
            pen=pg.mkPen(blue_color, width=2),
            symbol="o",
            symbolSize=1,
            symbolBrush=brush,
            symbolPen="k",
        )
        self.orange = dict(
            pen=pg.mkPen(orange_color, width=2),
            symbol="o",
            symbolSize=1,
            symbolBrush=brush,
            symbolPen="k",
        )

        font = QFont()
        font.setPixelSize(16)

        # Presence plot
        self.presence_plot = win.addPlot(row=0, col=0)
        self.presence_plot.setMenuEnabled(False)
        self.presence_plot.showGrid(x=True, y=True)
        self.presence_plot.addLegend()
        self.presence_plot.setLabel("left", "Presence score")
        self.presence_plot.setLabel("bottom", "Distance (m)")
        self.presence_plot_curve = [self.presence_plot.plot(**self.blue), self.presence_plot.plot(**self.orange)]

        # Time series plot
        self.time_series_plot = win.addPlot(row=1, col=0)
        self.time_series_plot.setMenuEnabled(False)
        self.time_series_plot.showGrid(x=True, y=True)
        self.time_series_plot.addLegend()
        self.time_series_plot.setLabel("left", "Displacement")
        self.time_series_plot.setLabel("bottom", "Time (s)")
        self.time_series_curve = self.time_series_plot.plot(**self.blue)

        # Breathing PSD plot
        self.breathing_psd_plot = win.addPlot(row=2, col=0)
        self.breathing_psd_plot.setMenuEnabled(False)
        self.breathing_psd_plot.showGrid(x=True, y=True)
        self.breathing_psd_plot.addLegend()
        self.breathing_psd_plot.setLabel("left", "PSD")
        self.breathing_psd_plot.setLabel("bottom", "Breathing rate (Hz)")
        self.breathing_psd_curve = self.breathing_psd_plot.plot(**self.blue)

        # Breathing rate plot
        self.breathing_rate_plot = win.addPlot(row=3, col=0)
        self.breathing_rate_plot.setMenuEnabled(False)
        self.breathing_rate_plot.showGrid(x=True, y=True)
        self.breathing_rate_plot.addLegend()
        self.breathing_rate_plot.setLabel("left", "Breaths per minute")
        self.breathing_rate_plot.setLabel("bottom", "Time (s)")
        self.breathing_rate_curves = [self.breathing_rate_plot.plot(**self.blue)]

    def update_presence(self, data):
        # Update presence plot based on the received ROS message data
        self.presence_plot_curve[0].setData(data.inter)
        self.presence_plot_curve[1].setData(data.intra)
        rospy.loginfo(f"Updated presence plot with inter: {data.inter}, intra: {data.intra}")

    def update_respiration(self, data):
        # Update breathing PSD plot and rate plot based on the received ROS message data
        self.time_series_curve.setData(data.time_vector, data.motion)
        self.breathing_psd_curve.setData(data.psd_frequencies, data.psd)
        self.breathing_rate_curves[0].setData(data.time_vector, data.all_respiration_rate_history)
        rospy.loginfo(f"Updated respiration plot with rate: {data.respiration_rate}, motion: {data.motion}")

    def update_status(self, presence_status, respiration_status):
        # Optionally handle updates to status if necessary
        rospy.loginfo(f"Presence status: {presence_status}, Respiration status: {respiration_status}")


def ros_plotter():
    rospy.init_node('acconeer_plotter', anonymous=True)

    # Create a QApplication instance
    app = QtWidgets.QApplication([])  # Corrected from QtGui to QtWidgets for QApplication

    # Create window for plots
    win = pg.GraphicsLayoutWidget(show=True, title="Acconeer Presence and Respiration Data")
    win.resize(1000, 600)

    pg_updater = PGUpdater()
    pg_updater.setup(win)

    # ROS subscriptions for presence and respiration data
    rospy.Subscriber('/acconeer/presence', Presence, pg_updater.update_presence)
    rospy.Subscriber('/acconeer/respiration', Respiration, pg_updater.update_respiration)
    rospy.Subscriber('/acconeer/presence_status', String, lambda msg: pg_updater.update_status(msg.data, None))
    rospy.Subscriber('/acconeer/respiration_status', String, lambda msg: pg_updater.update_status(None, msg.data))

    # Enter the ROS loop
    rospy.loginfo("ROS Acconeer Plotter Node Started")
    while not rospy.is_shutdown():
        # Keep the PyQtGraph GUI responsive
        QtWidgets.QApplication.processEvents()  # QApplication is now correctly referenced
        rospy.sleep(0.05)  # Sleep to avoid using 100% CPU


if __name__ == '__main__':
    try:
        ros_plotter()
    except rospy.ROSInterruptException:
        pass
