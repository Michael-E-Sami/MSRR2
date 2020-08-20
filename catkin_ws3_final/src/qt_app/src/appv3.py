#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.14.2
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWebEngineWidgets import QWebEngineView,QWebEngineSettings
from PyQt5.QtDataVisualization import (Q3DSurface, Q3DScatter, Q3DTheme, QAbstract3DGraph,
                                       QHeightMapSurfaceDataProxy, QSurface3DSeries, QSurfaceDataItem,
                                       QSurfaceDataProxy, QValue3DAxis, QScatter3DSeries, QAbstract3DSeries,
                                       QScatterDataItem, QLogValue3DAxisFormatter, QScatterDataProxy)
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtCore import QDir, Qt, QUrl,QEvent,QPointF,QTimer
from PyQt5.QtGui import QFont, QVector3D, QQuaternion, QLinearGradient
from PyQt5.QtChart import (QAreaSeries, QBarSet, QChart, QChartView,
                           QLineSeries, QPieSeries, QScatterSeries, QSplineSeries,
                           QStackedBarSeries)
import numpy as np
from functools import partial
import random
import time
import os

class Ui_MainWindow(object):
    mode_of_operation = "Simulation"
    robots_to_operate = []
    robots_connected = [1,2,5]
    time_since_start = 0.0
    working_from_load = False
    lastTab = 0
    bigGuy = "video"
    lastPressed = "video"
    canGetSmall = False
    firstRoutine = True
    # videoSize = None
    # scatGraphSize = None
    # robotsGraphsSize = None
    # pathPlanSize = None
    robotVariablesStr = ["Position X", "Position Y", "Yaw angle", "Linear Speed", "Angular Speed", "Roll Angle",
                         "Pitch Angle", "Front tof right", "Front tof left", "Front ir",
                         "Back tof right", "Back tof left", "Back ir"]

    dictStr = ["/pos_x","/pos_y","/yaw","/linear","/angular","/f_roll","/f_pitch",
               "/front_face_tof1","/front_face_tof2","/front_face_ir","/body_tof1","/body_tof2","/body_ir"]
    graphDict = {}
    graphSeriesDict = {}
    dataDict = {}
    scatterRobotSeries = {}
    data_3d = {}
    data_3d_array = {}

    def __init__(self,robots_connected = [1,2,5],mode_of_operation = "Simulation"):
        self.robots_connected = robots_connected
        self.mode_of_operation = mode_of_operation
        import rospy
        rospy.init_node('MSSR_application', anonymous=True)
        if mode_of_operation == "Simulation":
            from simClass import simClass as opCl
        if mode_of_operation == "Pysical":
            from pysClass import pysClass as opCl

        self.operate = opCl(self.robots_connected)
        os.system('xset r off')


    def setupUi(self, MainWindow):
        import os

        self.direct = os.path.dirname(__file__)
        # if self.mode_of_operation == "Pysical":

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1402, 807)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralWidget)
        self.gridLayout_2.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_2.setSpacing(6)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.tabWidget = QtWidgets.QTabWidget(self.centralWidget)
        self.tabWidget.setMinimumSize(QtCore.QSize(1380, 720))
        self.tabWidget.setStyleSheet("QTabBar::tab {\n"
                                     "    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,\n"
                                     "                                stop: 0 #E1E1E1, stop: 0.4 #DDDDDD,\n"
                                     "                                stop: 0.5 #D8D8D8, stop: 1.0 #D3D3D3);\n"
                                     "    border: 2px solid #C4C4C3;\n"
                                     "    border-bottom-color: #C2C7CB; /* same as the pane color */\n"
                                     "    border-top-left-radius: 4px;\n"
                                     "    border-top-right-radius: 4px;\n"
                                     "    min-width: 8ex;\n"
                                     "    padding: 2px;\n"
                                     "}\n"
                                     "QTabBar::tab:selected, QTabBar::tab:hover {\n"
                                     "    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,\n"
                                     "                                stop: 0 #fafafa, stop: 0.4 #f4f4f4,\n"
                                     "                                stop: 0.5 #e7e7e7, stop: 1.0 #fafafa);\n"
                                     "}\n"
                                     "\n"
                                     "QTabBar::tab:selected {\n"
                                     "    border-color: #9B9B9B;\n"
                                     "    border-bottom-color: #C2C7CB; /* same as pane color */\n"
                                     "}")
        self.tabWidget.setTabPosition(QtWidgets.QTabWidget.West)
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget.setIconSize(QtCore.QSize(100, 100))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_6 = QtWidgets.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.tab_6)
        self.verticalLayout_4.setContentsMargins(6, 6, 6, 6)
        self.verticalLayout_4.setSpacing(6)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.bigWindow = QtWidgets.QWidget(self.tab_6)
        self.bigWindow.setObjectName("bigWindow")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.bigWindow)
        self.horizontalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_6.setSpacing(6)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.widget_11 = QtWidgets.QWidget(self.bigWindow)
        self.widget_11.setObjectName("widget_11")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget_11)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setSpacing(6)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.widget_12 = QtWidgets.QWidget(self.widget_11)
        #######################################################################################
        # self.widget_12.setMinimumSize(QtCore.QSize(640, 480))
        self.widget_12.setSizePolicy(QtWidgets.QSizePolicy.Ignored,QtWidgets.QSizePolicy.Ignored)
        #######################################################################################
        self.widget_12.setObjectName("widget_12")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.widget_12)
        self.verticalLayout_6.setContentsMargins(6, 6, 6, 6)
        self.verticalLayout_6.setSpacing(6)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.layout1 = QtWidgets.QHBoxLayout()
        self.layout1.setSpacing(6)
        self.layout1.setObjectName("layout1")
        #######################################################################################

        if self.mode_of_operation == "Simulation":
            self.video = QWebEngineView(QtWidgets.QWidget(self.widget_12))
            # self.video.setMinimumSize(QtCore.QSize(640, 480))
            self.video.setObjectName("video")
            self.url = QUrl("http://localhost:8080/stream?topic=/RealSense/color/image_raw")
            self.video.settings().setAttribute(QWebEngineSettings.PluginsEnabled,True)
            self.video.setUrl(self.url)
            # self.video.setSizePolicy(QtWidgets.QSizePolicy.Maximum,QtWidgets.QSizePolicy.Maximum)
            # self.widget.load(url)
            self.layout1.addWidget(self.video)
            


        #######################################################################################
        # if self.mode_of_operation == "Simulation":
        #     self.video = QVideoWidget(QtWidgets.QWidget(self.widget_12))
        #     # self.video.setMinimumSize(QtCore.QSize(640    , 480))
        #     self.video.setObjectName("video")
        #     self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        #     self.mediaPlayer.setVideoOutput(self.video)
        #     # self.mediaPlayer.setMedia(
        #     #     QMediaContent(QUrl.fromLocalFile('/home/micha/PycharmProjects/testsAndOS/workout.mp4')))
        #     self.mediaPlayer.setMedia(QMediaContent(QUrl("http://localhost:8080/stream?topic=/RealSense/color/image_raw")))
        #     self.mediaPlayer.play()
        #     self.layout1.addWidget(self.video)

        if self.mode_of_operation == "Physical":

            self.video = QVideoWidget(QtWidgets.QWidget(self.widget_12))
            # self.video.setMinimumSize(QtCore.QSize(640, 480))
            self.video.setObjectName("video")
            self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
            self.mediaPlayer.setVideoOutput(self.video)
            # self.url = QUrl("https://www.youtube.com/embed/R7bkoH_JVp0")
            # self.mediaPlayer.setMedia(
            #         QMediaContent(self.url))
            self.mediaPlayer.setMedia(
                QMediaContent(QUrl.fromLocalFile('/home/micha/PycharmProjects/testsAndOS/workout.mp4')))
            self.mediaPlayer.play()
            self.layout1.addWidget(self.video)

            # import simClass as sc
            # self.sclass = sc.simClass()

        #######################################################################################
        self.verticalLayout_6.addLayout(self.layout1)
        self.verticalLayout_5.addWidget(self.widget_12)
        self.widget_13 = QtWidgets.QWidget(self.widget_11)
        self.widget_13.setObjectName("widget_13")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_13)
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_8.setSpacing(6)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.widget_15 = QtWidgets.QWidget(self.widget_13)
        self.widget_15.setObjectName("widget_15")
        self.verticalLayout_16 = QtWidgets.QVBoxLayout(self.widget_15)
        self.verticalLayout_16.setContentsMargins(11, 11, 11, 11)
        self.verticalLayout_16.setSpacing(6)
        self.verticalLayout_16.setObjectName("verticalLayout_16")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setSpacing(6)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setSpacing(6)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.roll_left = QtWidgets.QPushButton(self.widget_15)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.roll_left.sizePolicy().hasHeightForWidth())
        self.roll_left.setSizePolicy(sizePolicy)
        self.roll_left.setObjectName("roll_left")
        self.verticalLayout_7.addWidget(self.roll_left)
        self.roll_right = QtWidgets.QPushButton(self.widget_15)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.roll_right.sizePolicy().hasHeightForWidth())
        self.roll_right.setSizePolicy(sizePolicy)
        self.roll_right.setObjectName("roll_right")
        self.verticalLayout_7.addWidget(self.roll_right)
        self.horizontalLayout_7.addLayout(self.verticalLayout_7)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setSpacing(6)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.pitch_up = QtWidgets.QPushButton(self.widget_15)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pitch_up.sizePolicy().hasHeightForWidth())
        self.pitch_up.setSizePolicy(sizePolicy)
        self.pitch_up.setObjectName("pitch_up")
        self.verticalLayout_8.addWidget(self.pitch_up)
        self.pitch_down = QtWidgets.QPushButton(self.widget_15)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pitch_down.sizePolicy().hasHeightForWidth())
        self.pitch_down.setSizePolicy(sizePolicy)
        self.pitch_down.setObjectName("pitch_down")
        self.verticalLayout_8.addWidget(self.pitch_down)
        self.horizontalLayout_7.addLayout(self.verticalLayout_8)
        self.verticalLayout_16.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_8.addWidget(self.widget_15)
        self.widget_14 = QtWidgets.QWidget(self.widget_13)
        self.widget_14.setObjectName("widget_14")
        self.verticalLayout_15 = QtWidgets.QVBoxLayout(self.widget_14)
        self.verticalLayout_15.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_15.setSpacing(6)
        self.verticalLayout_15.setObjectName("verticalLayout_15")
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setSpacing(6)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.button_left = QtWidgets.QPushButton(self.widget_14)
        self.button_left.setMaximumSize(QtCore.QSize(50, 50))
        self.button_left.setStyleSheet("QPushButton{\n"
                                       "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,x3: 0, y3: 0, stop: 0 white, stop: 1 grey);\n"
                                       "border-style: solid;\n"
                                       "border-color: black;\n"
                                       "border-width: 5px;\n"
                                       "border-radius: 10px;\n"
                                       "}\n"
                                       "QPushButton::pressed{\n"
                                       "background-color: qlineargradient(x1: 1, y1: 1, x2: 1, y2: 0, stop: 0 white, stop: 1 grey);\n"
                                       "border-style: solid;\n"
                                       "border-color: blue;\n"
                                       "border-width: 5px;\n"
                                       "border-radius: 10px;\n"
                                       "}")
        self.button_left.setText("")
        self.button_left.setObjectName("button_left")
        self.gridLayout_4.addWidget(self.button_left, 1, 2, 1, 1)
        self.button_forward = QtWidgets.QPushButton(self.widget_14)
        self.button_forward.setMaximumSize(QtCore.QSize(50, 50))
        self.button_forward.setStyleSheet("QPushButton{\n"
                                          "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,x3: 0, y3: 0, stop: 0 white, stop: 1 grey);\n"
                                          "border-style: solid;\n"
                                          "border-color: black;\n"
                                          "border-width: 5px;\n"
                                          "border-radius: 10px;\n"
                                          "}\n"
                                          "QPushButton::pressed{\n"
                                          "background-color: qlineargradient(x1: 1, y1: 1, x2: 1, y2: 0, stop: 0 white, stop: 1 grey);\n"
                                          "border-style: solid;\n"
                                          "border-color: blue;\n"
                                          "border-width: 5px;\n"
                                          "border-radius: 10px;\n"
                                          "}")
        self.button_forward.setText("")
        self.button_forward.setObjectName("button_forward")
        self.gridLayout_4.addWidget(self.button_forward, 0, 3, 1, 1)
        self.button_backward = QtWidgets.QPushButton(self.widget_14)
        self.button_backward.setMaximumSize(QtCore.QSize(50, 50))
        self.button_backward.setStyleSheet("QPushButton{\n"
                                           "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,x3: 0, y3: 0, stop: 0 white, stop: 1 grey);\n"
                                           "border-style: solid;\n"
                                           "border-color: black;\n"
                                           "border-width: 5px;\n"
                                           "border-radius: 10px;\n"
                                           "}\n"
                                           "QPushButton::pressed{\n"
                                           "background-color: qlineargradient(x1: 1, y1: 1, x2: 1, y2: 0, stop: 0 white, stop: 1 grey);\n"
                                           "border-style: solid;\n"
                                           "border-color: blue;\n"
                                           "border-width: 5px;\n"
                                           "border-radius: 10px;\n"
                                           "}")
        self.button_backward.setText("")
        self.button_backward.setObjectName("button_backward")
        self.gridLayout_4.addWidget(self.button_backward, 2, 3, 1, 1)
        self.button_right = QtWidgets.QPushButton(self.widget_14)
        self.button_right.setMaximumSize(QtCore.QSize(50, 50))
        self.button_right.setStyleSheet("QPushButton{\n"
                                        "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,x3: 0, y3: 0, stop: 0 white, stop: 1 grey);\n"
                                        "border-style: solid;\n"
                                        "border-color: black;\n"
                                        "border-width: 5px;\n"
                                        "border-radius: 10px;\n"
                                        "}\n"
                                        "QPushButton::pressed{\n"
                                        "background-color: qlineargradient(x1: 1, y1: 1, x2: 1, y2: 0, stop: 0 white, stop: 1 grey);\n"
                                        "border-style: solid;\n"
                                        "border-color: blue;\n"
                                        "border-width: 5px;\n"
                                        "border-radius: 10px;\n"
                                        "}")
        self.button_right.setText("")
        self.button_right.setObjectName("button_right")
        self.gridLayout_4.addWidget(self.button_right, 1, 4, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_4.addItem(spacerItem, 1, 1, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_4.addItem(spacerItem1, 1, 0, 1, 1)
        self.verticalLayout_15.addLayout(self.gridLayout_4)
        self.horizontalLayout_8.addWidget(self.widget_14)
        self.verticalLayout_5.addWidget(self.widget_13)
        self.horizontalLayout_6.addWidget(self.widget_11)
        self.widget_9 = QtWidgets.QWidget(self.bigWindow)
        self.widget_9.setObjectName("widget_9")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.widget_9)
        self.verticalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_9.setSpacing(6)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.layout2 = QtWidgets.QVBoxLayout()
        self.layout2.setSpacing(6)
        self.layout2.setObjectName("layout2")
        #######################################################################
        self.chart_scatGraph = QChart()
        self.chart_scatGraph.setTitle("Robot position")

        self.series_scatGraph_bounds = QLineSeries(self.chart_scatGraph)
        self.series_scatGraph_bounds.append(QPointF(-0.9,-0.5))
        self.series_scatGraph_bounds.append(QPointF(0.9, -0.5))
        self.series_scatGraph_bounds.append(QPointF(0.9, 0.5))
        self.series_scatGraph_bounds.append(QPointF(-0.9, 0.5))
        self.series_scatGraph_bounds.append(QPointF(-0.9,-0.5))
        self.series_scatGraph_bounds.setName("bounds")
        self.chart_scatGraph.addSeries(self.series_scatGraph_bounds)
        self.chart_scatGraph.createDefaultAxes()
        self.scatGraph = QChartView(self.chart_scatGraph,QtWidgets.QWidget(self.widget_9))
        # self.scatGraph.setMinimumSize(QtCore.QSize(300, 0))
        self.scatGraph.setObjectName("scatGraph")
        self.layout2.addWidget(self.scatGraph)
        #######################################################################
        # self.verticalLayout_9.addLayout(self.layout2,0,0)
        self.verticalLayout_9.addLayout(self.layout2)
        self.layout3 = QtWidgets.QVBoxLayout()
        self.layout3.setSpacing(6)
        self.layout3.setObjectName("layout3")
        #######################################################################
        self.chart_robotsGraphs = QChart()
        self.chart_robotsGraphs.setTitle("Readings")


        self.robotsGraphs = QChartView(self.chart_robotsGraphs,QtWidgets.QWidget(self.widget_9))
        self.robotsGraphs.setObjectName("robotsGraphs")
        self.layout3.addWidget(self.robotsGraphs)
        #######################################################################
        # self.verticalLayout_9.addLayout(self.layout3,1,0)
        self.verticalLayout_9.addLayout(self.layout3)
        self.layout4 = QtWidgets.QVBoxLayout()
        self.layout4.setSpacing(6)
        self.layout4.setObjectName("layout4")
        #######################################################################
        self.chart_pathPlan = QChart()
        self.series_pathPlan = QLineSeries(self.chart_pathPlan)
        self.series_pathPlan.setName("Series " + str(1))
        self.chart_pathPlan.addSeries(self.series_pathPlan)
        self.chart_pathPlan.createDefaultAxes()
        self.chart_pathPlan.setTitle("Path plan")
        self.pathPlan = QChartView(self.chart_pathPlan,QtWidgets.QWidget(self.widget_9))
        self.pathPlan.setObjectName("pathPlan")
        self.layout4.addWidget(self.pathPlan)

        #######################################################################
        # self.verticalLayout_9.addLayout(self.layout4,2,0)
        self.verticalLayout_9.addLayout(self.layout4)
        self.horizontalLayout_6.addWidget(self.widget_9)
        self.widget_10 = QtWidgets.QWidget(self.bigWindow)
        self.widget_10.setObjectName("widget_10")
        self.verticalLayout_13 = QtWidgets.QVBoxLayout(self.widget_10)
        self.verticalLayout_13.setContentsMargins(11, 11, 11, 11)
        self.verticalLayout_13.setSpacing(6)
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        ##############################################################################
        self.combo_box_driving_mode = QtWidgets.QComboBox(self.widget_10)
        self.combo_box_driving_mode.setObjectName("combo_box_driving_mode")
        self.combo_box_driving_mode.addItem("Manual")
        self.combo_box_driving_mode.addItem("Auto")
        self.verticalLayout_13.addWidget(self.combo_box_driving_mode)
        ##############################################################################
        self.groupBox_3 = QtWidgets.QGroupBox(self.widget_10)
        self.groupBox_3.setObjectName("groupBox_3")
        self.formLayout = QtWidgets.QFormLayout(self.groupBox_3)
        self.formLayout.setContentsMargins(11, 11, 11, 11)
        self.formLayout.setSpacing(6)
        self.formLayout.setObjectName("formLayout")
        self.label = QtWidgets.QLabel(self.groupBox_3)
        self.label.setObjectName("label")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label)
        self.wheel_speed_spin_box = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.wheel_speed_spin_box.setObjectName("wheel_speed_spin_box")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.wheel_speed_spin_box)
        self.label_2 = QtWidgets.QLabel(self.groupBox_3)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_2)
        self.face_speed_spin_box = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.face_speed_spin_box.setObjectName("face_speed_spin_box")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.face_speed_spin_box)
        self.label_3 = QtWidgets.QLabel(self.groupBox_3)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.rotation_speed_spin_box = QtWidgets.QDoubleSpinBox(self.groupBox_3)
        self.rotation_speed_spin_box.setObjectName("rotation_speed_spin_box")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.rotation_speed_spin_box)
        self.verticalLayout_13.addWidget(self.groupBox_3)
        ####################################################################
        self.treeWidget= QtWidgets.QTreeWidget(self.widget_10)
        self.treeWidget.setObjectName("treeWidget")
        self.treeWidget.setHeaderLabel('Show Graphs')
        self.treeWidget.itemChanged.connect(self.treeFunc)
        # self.treeWidget.itemClicked.connect(self.treeFunc)
        # self.treeWidget.currentItemChanged.connect(self.treeFunc)



        self.verticalLayout_13.addWidget(self.treeWidget)
        ####################################################################
        self.listWidget_2 = QtWidgets.QListWidget(self.widget_10)
        self.listWidget_2.setObjectName("listWidget_2")
        self.verticalLayout_13.addWidget(self.listWidget_2)
        for i in range(len(self.robots_connected)):
            self.item = QtWidgets.QListWidgetItem()
            self.listWidget_2.addItem(self.item)
            self.item.setText("ditto" + str(self.robots_connected[i]))

        self.listWidget_2.itemSelectionChanged.connect(self.listItemActivated)
        self.listWidget_2.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.listWidget_2.setCurrentItem(self.item)
        ####################################################################
        self.horizontalLayout_6.addWidget(self.widget_10)
        self.verticalLayout_4.addWidget(self.bigWindow)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(self.direct + "/controler.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tabWidget.addTab(self.tab_6, icon, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setStyleSheet("")
        self.tab_2.setObjectName("tab_2")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.tab_2)
        self.horizontalLayout_4.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_4.setSpacing(6)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.graph_3d = Q3DScatter()
        self.graph_3d_container = QtWidgets.QWidget.createWindowContainer(self.graph_3d,parent = self.tab_2)
        self.graph_3d_container.setObjectName("graph_3d_container")
        self.graph_3d.axisX().setTitle("X")
        self.graph_3d.axisY().setTitle("Time")
        self.graph_3d.axisZ().setTitle("Y")
        self.graph_3d.activeTheme().setType(Q3DTheme.Theme(Q3DTheme.ThemeArmyBlue))
        self.horizontalLayout_4.addWidget(self.graph_3d_container)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSpacing(6)
        self.verticalLayout.setObjectName("verticalLayout")
        ############################################################################################################################
        self.groupBox_2 = QtWidgets.QGroupBox(self.tab_2)
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout.addWidget(self.groupBox_2)

        self.formLayout_2 = QtWidgets.QFormLayout(self.groupBox_2)
        self.axisMinMaxSliderNegX = QtWidgets.QSlider(Qt.Horizontal, minimum=-30, maximum=-10)
        self.axisMinMaxSliderPosX = QtWidgets.QSlider(Qt.Horizontal, minimum=10, maximum=30)
        self.axisMinMaxSliderNegZ = QtWidgets.QSlider(Qt.Horizontal, minimum=-30, maximum=-10)
        self.axisMinMaxSliderPosZ = QtWidgets.QSlider(Qt.Horizontal, minimum=10, maximum=30)
        self.axisMinMaxSliderNegX.valueChanged.connect(self.changeBoundsNegX)
        self.axisMinMaxSliderPosX.valueChanged.connect(self.changeBoundsPosX)
        self.axisMinMaxSliderNegZ.valueChanged.connect(self.changeBoundsNegZ)
        self.axisMinMaxSliderPosZ.valueChanged.connect(self.changeBoundsPosZ)
        self.axisMinMaxSliderNegX.setValue(-30)
        self.axisMinMaxSliderPosX.setValue(30)
        self.axisMinMaxSliderNegZ.setValue(-30)
        self.axisMinMaxSliderPosZ.setValue(30)
        self.formLayout_2.addRow(QtWidgets.QLabel("Negative X"), self.axisMinMaxSliderNegX)
        self.formLayout_2.addRow(QtWidgets.QLabel("Positive X"), self.axisMinMaxSliderPosX)
        self.formLayout_2.addRow(QtWidgets.QLabel("Negative Y"), self.axisMinMaxSliderNegZ)
        self.formLayout_2.addRow(QtWidgets.QLabel("Positive Y"), self.axisMinMaxSliderPosZ)

        self.intervalSlider = QtWidgets.QSlider(Qt.Horizontal, minimum=1, maximum=30)
        self.intervalSlider.setValue(5)
        self.formLayout_2.addRow(QtWidgets.QLabel("Interval"), self.intervalSlider)

        self.groupBox_2.setLayout(self.formLayout_2)
        ############################################################################################################################
        # self.groupBox = QtWidgets.QGroupBox(self.tab_2)
        # self.groupBox.setObjectName("groupBox")
        # self.verticalLayout.addWidget(self.groupBox)
        self.horizontalLayout_4.addLayout(self.verticalLayout)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(self.direct + "/Graph.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tabWidget.addTab(self.tab_2, icon1, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.tab_3)
        self.horizontalLayout_2.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_2.setSpacing(6)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        ###############################################################################################################
        self.tabWidget_2 = QtWidgets.QTabWidget(self.tab_3)
        self.tabWidget_2.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget_2.setObjectName("tabWidget_2")
        self.tab_robots = []
        self.tableWidgets = []
        for i in range(len(self.robots_connected)):
            self.tab_4 = QtWidgets.QWidget()
            self.tab_4.setObjectName("tab_4")
            self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.tab_4)
            self.horizontalLayout_3.setContentsMargins(5, 5, 5, 5)
            self.horizontalLayout_3.setSpacing(6)
            self.horizontalLayout_3.setObjectName("horizontalLayout_3")
            self.tableWidget = QtWidgets.QTableWidget(self.tab_4)
            # self.tab_4.tex('ditto' + str(self.robots_connected[i]))
            self.tableWidget.setObjectName("tableWidget")
            self.tableWidget.setColumnCount(1)
            self.tableWidget.setRowCount(1)
            item = QtWidgets.QTableWidgetItem()
            self.tableWidget.setVerticalHeaderItem(0, item)
            item = QtWidgets.QTableWidgetItem()
            self.tableWidget.setHorizontalHeaderItem(0, item)
            self.horizontalLayout_3.addWidget(self.tableWidget)
            self.tabWidget_2.addTab(self.tab_4, "")
            self.horizontalLayout_2.addWidget(self.tabWidget_2)
            self.tableWidget.setColumnCount(14)
            self.tableWidget.setRowCount(1)
            self.tableWidget.setItem(0, 0, QtWidgets.QTableWidgetItem("Time"))
            for j in range(len(self.robotVariablesStr)):
                self.tableWidget.setItem(0, j + 1, QtWidgets.QTableWidgetItem(self.robotVariablesStr[j]))
            self.tab_robots.append(self.tab_4)
            self.tableWidgets.append(self.tableWidget)
            self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_robots[i]), 'ditto' + str(self.robots_connected[i]))
        ###############################################################################################################
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(self.direct + "/table8.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tabWidget.addTab(self.tab_3, icon2, "")
        self.tab_5 = QtWidgets.QWidget()
        self.tab_5.setObjectName("tab_5")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.tab_5)
        self.horizontalLayout_5.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout_5.setSpacing(6)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.scrollArea = QtWidgets.QScrollArea(self.tab_5)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 1222, 1029))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.scrollAreaWidgetContents)
        self.gridLayout_3.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_3.setSpacing(6)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.widget_5 = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.widget_5.setMinimumSize(QtCore.QSize(590, 331))
        self.widget_5.setObjectName("widget_5")
        self.gridLayout_3.addWidget(self.widget_5, 2, 0, 1, 1)
        self.widget_4 = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.widget_4.setMinimumSize(QtCore.QSize(590, 331))
        self.widget_4.setObjectName("widget_4")
        self.gridLayout_3.addWidget(self.widget_4, 0, 0, 1, 1)
        self.widget_3 = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.widget_3.setMinimumSize(QtCore.QSize(590, 331))
        self.widget_3.setObjectName("widget_3")
        self.gridLayout_3.addWidget(self.widget_3, 0, 1, 1, 1)
        self.widget_6 = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.widget_6.setMinimumSize(QtCore.QSize(590, 331))
        self.widget_6.setObjectName("widget_6")
        self.gridLayout_3.addWidget(self.widget_6, 1, 0, 1, 1)
        self.widget_7 = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.widget_7.setMinimumSize(QtCore.QSize(590, 331))
        self.widget_7.setObjectName("widget_7")
        self.gridLayout_3.addWidget(self.widget_7, 1, 1, 1, 1)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.horizontalLayout_5.addWidget(self.scrollArea)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(self.direct + "/config_2.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tabWidget.addTab(self.tab_5, icon3, "")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(self.direct + "/emergency_clear.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.tabWidget.addTab(self.tab, icon4, "")
        self.gridLayout_2.addWidget(self.tabWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 1402, 26))
        self.menuBar.setObjectName("menuBar")
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtWidgets.QToolBar(MainWindow)
        self.mainToolBar.setEnabled(True)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        self.tabWidget_2.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        #########################################################################
        for i in range(len(self.robots_connected)):
            for j in range(len(self.robotVariablesStr)):
                self.graphDict["ditto" + str(self.robots_connected[i]) + ' ' + self.robotVariablesStr[j]] = False
                # print(self.graphDict["ditto" + str(self.robots_connected[i]) + ' ' + self.robotVariablesStr[j]])

        # self.video.mousePressEvent = self.videoMouseEvent
        self.video.dragLeaveEvent = self.videoMouseEvent
        self.scatGraph.mousePressEvent = self.scatGraphMouseEvent
        self.robotsGraphs.mousePressEvent = self.robotsGraphsMouseEvent
        self.pathPlan.mousePressEvent = self.pathPlanMouseEvent

        self.horizontalLayout_6.setStretch(0, 2)
        self.horizontalLayout_6.setStretch(1, 1)
        self.horizontalLayout_6.setStretch(2, 1)

        self.verticalLayout_9.setStretch(0, 1)
        self.verticalLayout_9.setStretch(1, 1)
        self.verticalLayout_9.setStretch(2, 1)

        self.verticalLayout_5.setStretch(0, 2)
        self.verticalLayout_5.setStretch(1, 1)

        self.horizontalLayout_4.setStretch(0, 4)
        self.horizontalLayout_4.setStretch(1, 1)


        # self.verticalLayout_9.setRowStretch(0,1)
        # self.verticalLayout_9.setRowStretch(1, 1)
        # self.verticalLayout_9.setRowStretch(2, 1)
        self.scatGraph.setMinimumSize(QtCore.QSize(300, 0))
        self.robotsGraphs.setMinimumSize(QtCore.QSize(300, 0))
        self.video.setMinimumSize(QtCore.QSize(300, 0))
        self.scatGraph.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.robotsGraphs.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.pathPlan.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.video.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)

        # self.treeWidget.currentChanged = self.treeEventTest
        MainWindow.keyPressEvent = self.keyPressEvent
        MainWindow.keyReleaseEvent = self.keyReleaseEvent

        self.treeFill(robots_connected=self.robots_connected)
        self.routineTimer = QTimer()
        self.routineTimer.timeout.connect(self.routine)
        self.routineTimer.start(100)
        self.tabWidget.currentChanged.connect(self.whatTab)
        if self.working_from_load:
            self.button_backward.setDisabled(True)
            self.button_forward.setDisabled(True)

            self.button_left.setDisabled(True)
            self.button_right.setDisabled(True)

            self.pitch_down.setDisabled(True)
            self.pitch_up.setDisabled(True)

            self.roll_left.setDisabled(True)
            self.roll_right.setDisabled(True)

        self.button_left.pressed.connect(self.button_left_press)
        self.button_left.released.connect(self.button_left_release)
        self.button_right.pressed.connect(self.button_right_press)
        self.button_right.released.connect(self.button_right_release)
        self.button_forward.pressed.connect(self.button_forward_press)
        self.button_forward.released.connect(self.button_forward_release)
        self.button_backward.pressed.connect(self.button_backward_press)
        self.button_backward.released.connect(self.button_backward_release)
        self.pitch_up.pressed.connect(self.pitch_up_press)
        self.pitch_up.released.connect(self.pitch_up_release)
        self.pitch_down.pressed.connect(self.pitch_down_press)
        self.pitch_down.released.connect(self.pitch_down_release)
        self.roll_left.pressed.connect(self.roll_left_press)
        self.roll_left.released.connect(self.roll_left_release)
        self.roll_right.pressed.connect(self.roll_right_press)
        self.roll_right.released.connect(self.roll_right_release)

        self.rotation_speed_spin_box.setRange(0, 1)
        self.rotation_speed_spin_box.setSingleStep(0.02)
        self.rotation_speed_spin_box.setValue(0.1)
        self.face_speed_spin_box.setRange(0, 1)
        self.face_speed_spin_box.setSingleStep(0.02)
        self.face_speed_spin_box.setValue(0.1)
        self.wheel_speed_spin_box.setRange(0, 1)
        self.wheel_speed_spin_box.setSingleStep(0.02)
        self.wheel_speed_spin_box.setValue(0.1)

        MainWindow.closeEvent = self.closeEvent

        self.wheel_speed_spin_box.focusOutEvent = self.wheelFocOut
        self.wheel_speed_spin_box.focusInEvent = self.wheelFocIn
        self.rotation_speed_spin_box.focusOutEvent = self.rotFocOut
        self.rotation_speed_spin_box.focusInEvent = self.rotFocIn
        self.face_speed_spin_box.focusOutEvent = self.faceFocOut
        self.face_speed_spin_box.focusInEvent = self.faceFocIn

    #     self.video.eventFilter = self.videoEventFilter
    #
    #
    # def videoEventFilter(self,obj,event:QtCore.QEvent):
    #     # if(event.type() == QtCore.QEvent.MouseMove):
    #         print('mouse pressed')




        #########################################################################

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MSSR"))
        self.roll_left.setText(_translate("MainWindow", "Roll Left"))
        self.roll_right.setText(_translate("MainWindow", "Roll Right"))
        self.pitch_up.setText(_translate("MainWindow", "Pitch Up"))
        self.pitch_down.setText(_translate("MainWindow", "Pitch Down"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Configurations"))
        self.label.setText(_translate("MainWindow", "Wheel speed:"))
        self.label_2.setText(_translate("MainWindow", "Face speed:"))
        self.label_3.setText(_translate("MainWindow", "Rotation speed:"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Graph Specs"))
        # self.groupBox.setTitle(_translate("MainWindow", "GroupBox"))
        # item = self.tableWidget.verticalHeaderItem(0)
        # item.setText(_translate("MainWindow", "1"))
        # item = self.tableWidget.horizontalHeaderItem(0)
        # item.setText(_translate("MainWindow", "A"))
        # self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_4), _translate("MainWindow", "Tab 1"))
        __sortingEnabled = self.listWidget_2.isSortingEnabled()

    def wheelFocOut(self, event):

        QtWidgets.QDoubleSpinBox.focusOutEvent(self.wheel_speed_spin_box, event)
        os.system('xset r off')
        print("lost focus")


    def wheelFocIn(self, event):

        QtWidgets.QDoubleSpinBox.focusInEvent(self.wheel_speed_spin_box, event)
        os.system('xset r on')
        print('gained focus')

    def rotFocOut(self, event):

        QtWidgets.QDoubleSpinBox.focusOutEvent(self.rotation_speed_spin_box, event)
        os.system('xset r off')
        print("lost focus")


    def rotFocIn(self, event):

        QtWidgets.QDoubleSpinBox.focusInEvent(self.rotation_speed_spin_box, event)
        os.system('xset r on')
        print('gained focus')

    def faceFocOut(self, event):

        QtWidgets.QDoubleSpinBox.focusOutEvent(self.face_speed_spin_box, event)
        os.system('xset r off')
        print("lost focus")


    def faceFocIn(self, event):

        QtWidgets.QDoubleSpinBox.focusInEvent(self.face_speed_spin_box, event)
        os.system('xset r on')
        print('gained focus')

    def closeEvent(self,event):
        os.system('xset r on')

    def keyPressEvent(self,event):

            if (event.text() == 'a') or (event.text() == 'A'):

                self.button_left.setDown(True)
                self.button_left.pressed.emit()

            if (event.text() == 'd') or (event.text() == 'D'):

                self.button_right.setDown(True)
                self.button_right.pressed.emit()

            if (event.text() == 'w') or (event.text() == 'W'):

                self.button_forward.setDown(True)
                self.button_forward.pressed.emit()

            if (event.text() == 's') or (event.text() == 'S'):

                self.button_backward.setDown(True)
                self.button_backward.pressed.emit()

            if (event.text() == 'r') or (event.text() == 'R'):

                self.roll_left.setDown(True)
                self.roll_left.pressed.emit()

            if (event.text() == 'f') or (event.text() == 'F'):

                self.roll_right.setDown(True)
                self.roll_right.pressed.emit()

            if (event.text() == 't') or (event.text() == 'T'):

                self.pitch_up.setDown(True)
                self.pitch_up.pressed.emit()

            if (event.text() == 'g') or (event.text() == 'G'):

                self.pitch_down.setDown(True)
                self.pitch_down.pressed.emit()


    def keyReleaseEvent(self, event):

            if (event.text() == 'a') or (event.text() == 'A'):
                # os.system('xset r on')
                self.button_left.setDown(False)
                self.button_left.released.emit()

            if (event.text() == 'd') or (event.text() == 'D'):

                self.button_right.setDown(False)
                self.button_right.released.emit()

            if (event.text() == 'w') or (event.text() == 'W'):

                self.button_forward.setDown(False)
                self.button_forward.released.emit()

            if (event.text() == 's') or (event.text() == 'S'):

                self.button_backward.setDown(False)
                self.button_backward.released.emit()

            if (event.text() == 'r') or (event.text() == 'R'):

                self.roll_left.setDown(False)
                self.roll_left.released.emit()

            if (event.text() == 'f') or (event.text() == 'F'):

                self.roll_right.setDown(False)
                self.roll_right.released.emit()

            if (event.text() == 't') or (event.text() == 'T'):

                self.pitch_up.setDown(False)
                self.pitch_up.released.emit()

            if (event.text() == 'g') or (event.text() == 'G'):

                self.pitch_down.setDown(False)
                self.pitch_down.released.emit()


    def routine(self):
        self.time_since_start += 0.1
        if self.firstRoutine:
            self.firstRoutine = False
            # print(self.pitch_up.size())
            self.widget_15.setFixedSize(self.widget_15.size())
            # self.videoSize = self.video.size()
            # self.scatGraphSize = self.scatGraph.size()
            # self.robotsGraphsSize = self.robotsGraphs.size()
            # self.pathPlanSize = self.pathPlan.size()
            self.widget_10.setMaximumSize(self.widget_10.size().width()*0.8,self.widget_10.size().height()*10)
            self.widget_10.setMinimumSize(self.widget_10.size().width(),self.widget_10.size().height())
            # self.series_robotsGraphs = QLineSeries(self.chart_robotsGraphs)

            self.graph_3d.axisZ().setRange(-3, 3)
            self.graph_3d.axisX().setRange(-3, 3)

            random.seed(time)
            for i in range(len(self.robots_connected)):
                dittoName = "ditto" + str(self.robots_connected[i])
                shortName = "d" + str(self.robots_connected[i])
                self.scatterRobotSeries[dittoName] = QScatterSeries(self.chart_scatGraph)
                self.scatterRobotSeries[dittoName].append(QPointF(0, 0))
                self.scatterRobotSeries[dittoName].setName(dittoName)
                self.chart_scatGraph.addSeries(self.scatterRobotSeries[dittoName])

                self.dataDict[dittoName + "/body_ir"] = []
                self.dataDict[dittoName + "/body_tof1"] = []
                self.dataDict[dittoName + "/body_tof2"] = []
                self.dataDict[dittoName + "/front_face_ir"] = []
                self.dataDict[dittoName + "/front_face_tof1"] = []
                self.dataDict[dittoName + "/front_face_tof2"] = []
                self.dataDict[dittoName + "/pos_x"] = []
                self.dataDict[dittoName + "/pos_y"] = []
                self.dataDict[dittoName + "/yaw"] = []
                self.dataDict[dittoName + "/linear"] = []
                self.dataDict[dittoName + "/angular"] = []
                self.dataDict[dittoName + "/f_roll"] = []
                self.dataDict[dittoName + "/f_pitch"] = []
                self.dataDict[dittoName + "/body_ir"].append(0.0)
                self.dataDict[dittoName + "/body_tof1"].append(0.0)
                self.dataDict[dittoName + "/body_tof2"].append(0.0)
                self.dataDict[dittoName + "/front_face_ir"].append(0.0)
                self.dataDict[dittoName + "/front_face_tof1"].append(0.0)
                self.dataDict[dittoName + "/front_face_tof2"].append(0.0)
                self.dataDict[dittoName + "/pos_x"].append(0.0)
                self.dataDict[dittoName + "/pos_y"].append(0.0)
                self.dataDict[dittoName + "/yaw"].append(0.0)
                self.dataDict[dittoName + "/linear"].append(0.0)
                self.dataDict[dittoName + "/angular"].append(0.0)
                self.dataDict[dittoName + "/f_roll"].append(0.0)
                self.dataDict[dittoName + "/f_pitch"].append(0.0)

                self.data_3d[dittoName + "/pos_x"] = []
                self.data_3d[dittoName + "/pos_y"] = []
                self.data_3d[dittoName + "/yaw"] = []
                self.data_3d_array[dittoName] = []
                proxy = QScatterDataProxy()
                series = QScatter3DSeries(proxy)
                series.setItemLabelFormat(
                    dittoName + " @yTitle: @yLabel @xTitle: @xLabel @zTitle: @zLabel")
                # liner_grad = QLinearGradient()
                # series.setColorStyle(Q3DTheme.)
                # series.setMeshSmooth(self.m_smooth)
                self.graph_3d.addSeries(series)
                self.graph_3d.seriesList()[i].setMesh(QAbstract3DSeries.MeshPyramid)

                self.graphSeriesDict[dittoName + "/body_ir"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/body_ir"].setName(shortName + "b_ir")
                self.graphSeriesDict[dittoName + "/body_ir"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/body_tof1"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/body_tof1"].setName(shortName + "b_tof_l")
                self.graphSeriesDict[dittoName + "/body_tof1"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/body_tof2"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/body_tof2"].setName(shortName + "b_tof_r")
                self.graphSeriesDict[dittoName + "/body_tof2"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/front_face_ir"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/front_face_ir"].setName(shortName + "f_ir")
                self.graphSeriesDict[dittoName + "/front_face_ir"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/front_face_tof1"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/front_face_tof1"].setName(shortName + "f_tof_l")
                self.graphSeriesDict[dittoName + "/front_face_tof1"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/front_face_tof2"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/front_face_tof2"].setName(shortName + "f_tof_r")
                self.graphSeriesDict[dittoName + "/front_face_tof2"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/pos_x"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/pos_x"].setName(shortName + "pos_x")
                self.graphSeriesDict[dittoName + "/pos_x"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/pos_y"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/pos_y"].setName(shortName + "pos_y")
                self.graphSeriesDict[dittoName + "/pos_y"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/yaw"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/yaw"].setName(shortName + "yaw")
                self.graphSeriesDict[dittoName + "/yaw"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/linear"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/linear"].setName(shortName + "linear")
                self.graphSeriesDict[dittoName + "/linear"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/angular"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/angular"].setName(shortName + "angular")
                self.graphSeriesDict[dittoName + "/angular"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/f_roll"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/f_roll"].setName(shortName + "f_roll")
                self.graphSeriesDict[dittoName + "/f_roll"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
                self.graphSeriesDict[dittoName + "/f_pitch"] = QLineSeries(self.chart_robotsGraphs)
                self.graphSeriesDict[dittoName + "/f_pitch"].setName(shortName + "f_pitch")
                self.graphSeriesDict[dittoName + "/f_pitch"].setColor(
                    QtGui.QColor(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))

        for i in range(len(self.robots_connected)):
            dittoName = "ditto" + str(self.robots_connected[i])
            self.scatterRobotSeries[dittoName].append(QPointF(-self.operate.getPointY(dittoName),
                                                              self.operate.getPointX(dittoName)))

            self.scatterRobotSeries[dittoName].remove(0)
            self.dataDict[dittoName + "/body_ir"].append(self.operate.data[dittoName + "/body_ir"])
            self.graphSeriesDict[dittoName + "/body_ir"].append(
                QPointF(self.time_since_start, self.operate.data[dittoName + "/body_ir"]))
            self.dataDict[dittoName + "/body_tof1"].append(self.operate.data[dittoName + "/body_tof1"])
            self.graphSeriesDict[dittoName + "/body_tof1"].append(
                QPointF(self.time_since_start, self.operate.data[dittoName + "/body_tof1"]))
            self.dataDict[dittoName + "/body_tof2"].append(self.operate.data[dittoName + "/body_tof2"])
            self.graphSeriesDict[dittoName + "/body_tof2"].append(
                QPointF(self.time_since_start, self.operate.data[dittoName + "/body_tof2"]))
            self.dataDict[dittoName + "/front_face_ir"].append(self.operate.data[dittoName + "/front_face_ir"])
            self.graphSeriesDict[dittoName + "/front_face_ir"].append(
                QPointF(self.time_since_start, self.operate.data[dittoName + "/front_face_ir"]))
            self.dataDict[dittoName + "/front_face_tof1"].append(self.operate.data[dittoName + "/front_face_tof1"])
            self.graphSeriesDict[dittoName + "/front_face_tof1"].append(
                QPointF(self.time_since_start, self.operate.data[dittoName + "/front_face_tof1"]))
            self.dataDict[dittoName + "/front_face_tof2"].append(self.operate.data[dittoName + "/front_face_tof2"])
            self.graphSeriesDict[dittoName + "/front_face_tof2"].append(
                QPointF(self.time_since_start, self.operate.data[dittoName + "/front_face_tof2"]))
            self.dataDict[dittoName + "/pos_x"].append(self.operate.getPointX(dittoName))
            self.graphSeriesDict[dittoName + "/pos_x"].append(
                QPointF(self.time_since_start, self.operate.getPointX(dittoName)))
            self.dataDict[dittoName + "/pos_y"].append(self.operate.getPointY(dittoName))
            self.graphSeriesDict[dittoName + "/pos_y"].append(
                QPointF(self.time_since_start, self.operate.getPointY(dittoName)))
            self.dataDict[dittoName + "/yaw"].append(self.operate.getRotationYaw(dittoName))
            self.graphSeriesDict[dittoName + "/yaw"].append(
                QPointF(self.time_since_start, self.operate.getRotationYaw(dittoName)))
            self.dataDict[dittoName + "/angular"].append(self.operate.getAngularSpeed(dittoName))
            self.graphSeriesDict[dittoName + "/angular"].append(
                QPointF(self.time_since_start, self.operate.getAngularSpeed(dittoName)))
            self.dataDict[dittoName + "/linear"].append(self.operate.getLinearSpeed(dittoName))
            self.graphSeriesDict[dittoName + "/linear"].append(
                QPointF(self.time_since_start, self.operate.getLinearSpeed(dittoName)))
            self.dataDict[dittoName + "/f_roll"].append(self.operate.getFaceRoll(dittoName))
            self.graphSeriesDict[dittoName + "/f_roll"].append(
                QPointF(self.time_since_start, self.operate.getFaceRoll(dittoName)))
            self.dataDict[dittoName + "/f_pitch"].append(self.operate.getFacePitch(dittoName))
            self.graphSeriesDict[dittoName + "/f_pitch"].append(
                QPointF(self.time_since_start, self.operate.getFacePitch(dittoName)))

            self.data_3d[dittoName + "/pos_x"].append(self.operate.getPointX(dittoName))
            self.data_3d[dittoName + "/pos_y"].append(self.operate.getPointY(dittoName))
            self.data_3d[dittoName + "/yaw"].append(self.operate.getRotationYaw(dittoName))
            itm = QScatterDataItem(
                QVector3D(self.data_3d[dittoName + "/pos_x"][-1],
                          self.time_since_start, self.data_3d[dittoName + "/pos_y"][-1]),
                QQuaternion.fromAxisAndAngle(0.0, 1.0, 0.0, self.data_3d[dittoName + "/yaw"][-1] * 180 / np.pi) * QQuaternion.fromAxisAndAngle(0.0, 0.0, 1.0, -90))
            # print(dittoName)
            # print(self.data_3d[dittoName + "/yaw"][-1])

            self.data_3d_array[dittoName].append(itm)
            self.graph_3d.seriesList()[i].dataProxy().resetArray(self.data_3d_array[dittoName])

            self.tableWidgets[i].setRowCount(round(self.time_since_start * 10) + 1)
            self.tableWidgets[i].setItem(round(self.time_since_start * 10), 0, QtWidgets.QTableWidgetItem(str(round(self.time_since_start,1))))
            for j in range(len(self.dictStr)):
                self.tableWidgets[i].setItem(round(self.time_since_start * 10), j + 1,
                                             QtWidgets.QTableWidgetItem(str(round(self.dataDict[dittoName + self.dictStr[j]][-1], 1))))

            # if self.time_since_start > self.intervalSlider.value():
            #     self.data_3d_array[dittoName].pop(0)
                # print(len(self.data_3d_array[dittoName]))

            if self.time_since_start > 30:
                self.graphSeriesDict[dittoName + "/body_ir"].remove(0)
                self.graphSeriesDict[dittoName + "/body_tof1"].remove(0)
                self.graphSeriesDict[dittoName + "/body_tof2"].remove(0)
                self.graphSeriesDict[dittoName + "/front_face_ir"].remove(0)
                self.graphSeriesDict[dittoName + "/front_face_tof1"].remove(0)
                self.graphSeriesDict[dittoName + "/front_face_tof2"].remove(0)
                self.graphSeriesDict[dittoName + "/pos_x"].remove(0)
                self.graphSeriesDict[dittoName + "/pos_y"].remove(0)
                self.graphSeriesDict[dittoName + "/yaw"].remove(0)
                self.graphSeriesDict[dittoName + "/angular"].remove(0)
                self.graphSeriesDict[dittoName + "/linear"].remove(0)
                self.graphSeriesDict[dittoName + "/f_roll"].remove(0)
                self.graphSeriesDict[dittoName + "/f_pitch"].remove(0)

                self.data_3d[dittoName + "/pos_x"].pop(0)
                self.data_3d[dittoName + "/pos_y"].pop(0)
                self.data_3d[dittoName + "/yaw"].pop(0)
                self.data_3d_array[dittoName].pop(0)

            if self.graphDict[dittoName + ' Back ir']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/body_ir"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/body_ir"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/body_ir"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/body_ir"])
            if self.graphDict[dittoName + ' Back tof right']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/body_tof2"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/body_tof2"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/body_tof2"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/body_tof2"])
            if self.graphDict[dittoName + ' Back tof left']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/body_tof1"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/body_tof1"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/body_tof1"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/body_tof1"])
            if self.graphDict[dittoName + ' Front ir']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/front_face_ir"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/front_face_ir"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/front_face_ir"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/front_face_ir"])
            if self.graphDict[dittoName + ' Front tof right']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/front_face_tof2"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/front_face_tof2"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/front_face_tof2"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/front_face_tof2"])
            if self.graphDict[dittoName + ' Front tof left']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/front_face_tof1"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/front_face_tof1"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/front_face_tof1"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/front_face_tof1"])
            if self.graphDict[dittoName + ' Position X']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/pos_x"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/pos_x"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/pos_x"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/pos_x"])
            if self.graphDict[dittoName + ' Position Y']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/pos_y"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/pos_y"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/pos_y"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/pos_y"])
            if self.graphDict[dittoName + ' Yaw angle']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/yaw"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/yaw"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/yaw"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/yaw"])
            if self.graphDict[dittoName + ' Linear Speed']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/linear"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/linear"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/linear"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/linear"])
            if self.graphDict[dittoName + ' Angular Speed']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/angular"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/angular"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/angular"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/angular"])
            if self.graphDict[dittoName + ' Roll Angle']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/f_roll"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/f_roll"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/f_roll"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/f_roll"])
            if self.graphDict[dittoName + ' Pitch Angle']:
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/f_pitch"])
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/f_pitch"])
            else:
                self.chart_robotsGraphs.addSeries(self.graphSeriesDict[dittoName + "/f_pitch"])
                self.chart_robotsGraphs.removeSeries(self.graphSeriesDict[dittoName + "/f_pitch"])

        self.chart_robotsGraphs.createDefaultAxes()
        self.chart_scatGraph.createDefaultAxes()

        if self.time_since_start - self.intervalSlider.value() < 0:
            self.graph_3d.axisY().setRange(0, self.time_since_start)
        else:
            self.graph_3d.axisY().setRange(self.time_since_start - self.intervalSlider.value(), self.time_since_start)

    def button_left_press(self):
        print("left button was pressed")
        self.operate.rotateLeft(self.robots_to_operate, self.rotation_speed_spin_box.value())

    def button_left_release(self):
        print("left button was released")
        self.operate.stopRot(self.robots_to_operate)

    def button_right_press(self):
        print("right button was pressed")
        self.operate.rotateRight(self.robots_to_operate, self.rotation_speed_spin_box.value())

    def button_right_release(self):
        print("right button was released")
        self.operate.stopRot(self.robots_to_operate)

    def button_forward_press(self):
        print("forward button was pressed")
        self.operate.moveForward(self.robots_to_operate, self.wheel_speed_spin_box.value())

    def button_forward_release(self):
        print("forward button was released")
        self.operate.stopFwdBwd(self.robots_to_operate)

    def button_backward_press(self):
        print("backward button was pressed")
        self.operate.moveBackward(self.robots_to_operate, self.wheel_speed_spin_box.value())

    def button_backward_release(self):
        print("backward button was released")
        self.operate.stopFwdBwd(self.robots_to_operate)

    def pitch_up_press(self):
        print("pitch up was pressed")
        self.operate.pitchUp(self.robots_to_operate, self.face_speed_spin_box.value())

    def pitch_up_release(self):
        print("pitch up was released")
        self.operate.stopPitch(self.robots_to_operate)

    def pitch_down_press(self):
        print("pitch down was pressed")
        self.operate.pitchDown(self.robots_to_operate, self.face_speed_spin_box.value())

    def pitch_down_release(self):
        print("pitch down was released")
        self.operate.stopPitch(self.robots_to_operate)

    def roll_left_press(self):
        print("roll left was pressed")
        self.operate.rollLeft(self.robots_to_operate, self.face_speed_spin_box.value())

    def roll_left_release(self):
        print("roll left was released")
        self.operate.stopRoll(self.robots_to_operate)

    def roll_right_press(self):
        print("roll right was pressed")
        self.operate.rollRight(self.robots_to_operate, self.face_speed_spin_box.value())

    def roll_right_release(self):
        print("roll right was released")
        self.operate.stopRoll(self.robots_to_operate)


    def listItemActivated(self):
        # print(self.listWidget_2.currentItem().text())
        self.robots_to_operate = []
        for robot in self.listWidget_2.selectedItems():
            self.robots_to_operate.append(robot.text())

    def check_check(self, box):
        print(box.text())

    def whatTab(self):
        if self.tabWidget.currentIndex() == 4:
            self.tabWidget.setCurrentIndex(self.lastTab)
        self.lastTab = self.tabWidget.currentIndex()
        print(self.tabWidget.currentIndex())

    def treeFunc(self,item,column):
        # print(item.text(0))
        # print(column)
        if item.parent() != None:
            # print(item.parent().text(0))

            if item.checkState(0) == 0:
                checkstr = " unchecked"

            # if item.checkState(0) == 2:
            else:
                checkstr = " checked"

            print(item.parent().text(0) + ' ' + item.text(0) + checkstr)

            self.graphDict[item.parent().text(0) + ' ' + item.text(0)] = bool(item.checkState(0))

            # print(self.graphDict)

        # if self.treeWidget.currentItem() != None:
        #     print(self.treeWidget.currentItem())
        #     print(self.treeWidget.currentItem().text(0))

    def treeFill(self,robots_connected):

        self.treeParents = []
        self.treeChildes = []

        # checkboxStr = ['leftWheelSpeed_','rightWheelSpeed_','frontFacePanSpeed_','frontFaceTiltSpeed_','frontTofSensor_']
        for i in range(len(robots_connected)):
            self.parentRobot = QtWidgets.QTreeWidgetItem(self.treeWidget, ['ditto'+ str(robots_connected[i])])
            self.parentRobot.setCheckState(0, Qt.PartiallyChecked)
            self.parentRobot.setFlags(self.parentRobot.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
            for j in range(len(self.robotVariablesStr)):
                self.childRobot = QtWidgets.QTreeWidgetItem(self.parentRobot, [self.robotVariablesStr[j]])
                self.childRobot.setCheckState(0, Qt.Unchecked)
                self.childRobot.setFlags(self.childRobot.flags() | Qt.ItemIsUserCheckable)

        #     for j in range(len(checkboxStr)):
        #         self.check_box = QtWidgets.QCheckBox(self.scrollAreaWidgetContents_2)
        #         self.check_box.setText(checkboxStr[j] + str(robots_connected[i]))
        #         self.check_box.clicked.connect(partial(self.check_check,self.check_box))
        #         self.verticalLayout_14.addWidget(self.check_box)
        #         self.checkBoxes.append(self.check_box)

        # for i in range(len(self.checkBoxes)):
        #     print(self.checkBoxes[i].text())

    def videoMouseEvent(self,event):

        print("video pressed")
        if self.lastPressed == "scatGraph":
            self.layout1.addWidget(self.video)
            self.layout2.addWidget(self.scatGraph)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.pathPlan)


        if self.lastPressed == "robotsGraphs":
            self.layout1.addWidget(self.video)
            self.layout2.addWidget(self.scatGraph)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.pathPlan)

        if self.lastPressed == "pathPlan":
            self.layout1.addWidget(self.video)
            self.layout2.addWidget(self.scatGraph)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.pathPlan)

        if self.bigGuy == "video":
            if self.canGetSmall == False:
                self.verticalLayout_4.addWidget(self.video)
                self.bigWindow.hide()
                self.canGetSmall = True
            elif self.canGetSmall == True:
                self.layout1.addWidget(self.video)
                self.bigWindow.show()
                self.canGetSmall = False

        self.bigGuy = "video"
        self.lastPressed = "video"

    def scatGraphMouseEvent(self, event):
        print("scatGraph pressed")
        if self.lastPressed == "video":
            self.layout1.addWidget(self.scatGraph)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.pathPlan)

        if self.lastPressed == "robotsGraphs":
            self.layout1.addWidget(self.scatGraph)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.pathPlan)

        if self.lastPressed == "pathPlan":
            self.layout1.addWidget(self.scatGraph)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.pathPlan)

        if self.bigGuy == "scatGraph":
            if self.canGetSmall == False:
                self.verticalLayout_4.addWidget(self.scatGraph)
                self.bigWindow.hide()
                self.canGetSmall = True
            elif self.canGetSmall == True:
                self.layout1.addWidget(self.scatGraph)
                self.bigWindow.show()
                self.canGetSmall = False

        self.bigGuy = "scatGraph"
        self.lastPressed = "scatGraph"

    def robotsGraphsMouseEvent(self, event):
        print("robotsGraphs pressed")
        if self.lastPressed == "video":
            self.layout1.addWidget(self.robotsGraphs)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.scatGraph)
            self.layout4.addWidget(self.pathPlan)

        if self.lastPressed == "scatGraph":
            self.layout1.addWidget(self.robotsGraphs)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.scatGraph)
            self.layout4.addWidget(self.pathPlan)

        if self.lastPressed == "pathPlan":
            self.layout1.addWidget(self.robotsGraphs)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.scatGraph)
            self.layout4.addWidget(self.pathPlan)

        if self.bigGuy == "robotsGraphs":
            if self.canGetSmall == False:
                self.verticalLayout_4.addWidget(self.robotsGraphs)
                self.bigWindow.hide()
                self.canGetSmall = True
            elif self.canGetSmall == True:
                self.layout1.addWidget(self.robotsGraphs)
                self.bigWindow.show()
                self.canGetSmall = False

        self.bigGuy = "robotsGraphs"
        self.lastPressed = "robotsGraphs"

    def pathPlanMouseEvent(self, event):
        print("pathPlan pressed")
        if self.lastPressed == "video":
            self.layout1.addWidget(self.pathPlan)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.scatGraph)

        if self.lastPressed == "robotsGraphs":
            self.layout1.addWidget(self.pathPlan)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.scatGraph)

        if self.lastPressed == "scatGraph":
            self.layout1.addWidget(self.pathPlan)
            self.layout2.addWidget(self.video)
            self.layout3.addWidget(self.robotsGraphs)
            self.layout4.addWidget(self.scatGraph)

        if self.bigGuy == "pathPlan":
            if self.canGetSmall == False:
                self.verticalLayout_4.addWidget(self.pathPlan)
                self.bigWindow.hide()
                self.canGetSmall = True
            elif self.canGetSmall == True:
                self.layout1.addWidget(self.pathPlan)
                self.bigWindow.show()
                self.canGetSmall = False

        self.bigGuy = "pathPlan"
        self.lastPressed = "pathPlan"

    def changeBoundsNegX(self,val):
        self.graph_3d.axisX().setRange(self.axisMinMaxSliderNegX.value()/10, self.axisMinMaxSliderPosX.value()/10)

    def changeBoundsPosX(self,val):
        self.graph_3d.axisX().setRange(self.axisMinMaxSliderNegX.value()/10, self.axisMinMaxSliderPosX.value()/10)

    def changeBoundsNegZ(self,val):
        self.graph_3d.axisZ().setRange(self.axisMinMaxSliderNegZ.value()/10, self.axisMinMaxSliderPosZ.value()/10)

    def changeBoundsPosZ(self,val):
        self.graph_3d.axisZ().setRange(self.axisMinMaxSliderNegZ.value()/10, self.axisMinMaxSliderPosZ.value()/10)

    # def changeInterval(self,val):
    #     if
    #

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
