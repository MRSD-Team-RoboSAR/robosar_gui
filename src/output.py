# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/robosar_gui/src/robosar_gui.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1045, 736)
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(247, 247, 247))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(247, 247, 247))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(247, 247, 247))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        Dialog.setPalette(palette)
        Dialog.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(Dialog)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(Dialog)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setMaximumSize(QtCore.QSize(200, 100))
        self.label_2.setText("")
        self.label_2.setPixmap(QtGui.QPixmap("src/robosar_gui/src/../img/robosar_logo_big.png"))
        self.label_2.setScaledContents(True)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setMaximumSize(QtCore.QSize(10000, 50))
        font = QtGui.QFont()
        font.setPointSize(28)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.horizontalLayout.setStretch(0, 1)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.verticalLayout_6.setSpacing(5)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.groupBox_2 = QtWidgets.QGroupBox(Dialog)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_2.sizePolicy().hasHeightForWidth())
        self.groupBox_2.setSizePolicy(sizePolicy)
        self.groupBox_2.setMaximumSize(QtCore.QSize(16777213, 16777215))
        self.groupBox_2.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.scrollArea = QtWidgets.QScrollArea(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scrollArea.sizePolicy().hasHeightForWidth())
        self.scrollArea.setSizePolicy(sizePolicy)
        self.scrollArea.setMinimumSize(QtCore.QSize(100, 500))
        self.scrollArea.setMaximumSize(QtCore.QSize(400, 10000))
        self.scrollArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setAlignment(QtCore.Qt.AlignCenter)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 84, 498))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.scrollAreaWidgetContents)
        self.verticalLayout.setObjectName("verticalLayout")
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout_9.addWidget(self.scrollArea)
        self.verticalLayout_6.addWidget(self.groupBox_2, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout_2.addLayout(self.verticalLayout_6)
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.groupBox = QtWidgets.QGroupBox(Dialog)
        self.groupBox.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.task_image = QtWidgets.QLabel(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.task_image.sizePolicy().hasHeightForWidth())
        self.task_image.setSizePolicy(sizePolicy)
        self.task_image.setMinimumSize(QtCore.QSize(400, 200))
        self.task_image.setText("")
        self.task_image.setScaledContents(False)
        self.task_image.setAlignment(QtCore.Qt.AlignCenter)
        self.task_image.setObjectName("task_image")
        self.verticalLayout_8.addWidget(self.task_image)
        self.verticalLayout_7.addWidget(self.groupBox, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout_2.addLayout(self.verticalLayout_7)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.groupBox_3 = QtWidgets.QGroupBox(Dialog)
        self.groupBox_3.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.groupBox_3)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.mission_timer_label = QtWidgets.QLabel(self.groupBox_3)
        self.mission_timer_label.setMaximumSize(QtCore.QSize(300, 1000))
        font = QtGui.QFont()
        font.setPointSize(36)
        self.mission_timer_label.setFont(font)
        self.mission_timer_label.setAlignment(QtCore.Qt.AlignCenter)
        self.mission_timer_label.setObjectName("mission_timer_label")
        self.verticalLayout_3.addWidget(self.mission_timer_label)
        self.start_timer_button = QtWidgets.QPushButton(self.groupBox_3)
        self.start_timer_button.setObjectName("start_timer_button")
        self.verticalLayout_3.addWidget(self.start_timer_button)
        self.stop_timer_button = QtWidgets.QPushButton(self.groupBox_3)
        self.stop_timer_button.setObjectName("stop_timer_button")
        self.verticalLayout_3.addWidget(self.stop_timer_button)
        self.restart_timer_button = QtWidgets.QPushButton(self.groupBox_3)
        self.restart_timer_button.setObjectName("restart_timer_button")
        self.verticalLayout_3.addWidget(self.restart_timer_button)
        self.verticalLayout_5.addWidget(self.groupBox_3, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout_2.addLayout(self.verticalLayout_5)
        self.horizontalLayout_2.setStretch(0, 2)
        self.horizontalLayout_2.setStretch(1, 2)
        self.horizontalLayout_2.setStretch(2, 1)
        self.verticalLayout_4.addLayout(self.horizontalLayout_2)
        self.label_6 = QtWidgets.QLabel(Dialog)
        self.label_6.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_4.addWidget(self.label_6)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.start_mission_button = QtWidgets.QPushButton(Dialog)
        self.start_mission_button.setMaximumSize(QtCore.QSize(200, 100))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("src/robosar_gui/src/../img/k4-detail1.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.start_mission_button.setIcon(icon)
        self.start_mission_button.setIconSize(QtCore.QSize(25, 25))
        self.start_mission_button.setObjectName("start_mission_button")
        self.horizontalLayout_3.addWidget(self.start_mission_button)
        self.e_stop_button = QtWidgets.QPushButton(Dialog)
        self.e_stop_button.setMaximumSize(QtCore.QSize(200, 16777215))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("src/robosar_gui/src/../img/stop_sign_button.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.e_stop_button.setIcon(icon1)
        self.e_stop_button.setIconSize(QtCore.QSize(25, 25))
        self.e_stop_button.setObjectName("e_stop_button")
        self.horizontalLayout_3.addWidget(self.e_stop_button)
        self.homing_button = QtWidgets.QPushButton(Dialog)
        self.homing_button.setMaximumSize(QtCore.QSize(200, 16777215))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("src/robosar_gui/src/../img/avengers__endgame__2019__avengers_logo_png__by_mintmovi3_dd4bz30-fullview.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.homing_button.setIcon(icon2)
        self.homing_button.setIconSize(QtCore.QSize(25, 25))
        self.homing_button.setObjectName("homing_button")
        self.horizontalLayout_3.addWidget(self.homing_button)
        self.verticalLayout_4.addLayout(self.horizontalLayout_3)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label.setText(_translate("Dialog", "Mission Control"))
        self.groupBox_2.setTitle(_translate("Dialog", "Agent Status"))
        self.groupBox.setTitle(_translate("Dialog", "Task Allocation"))
        self.groupBox_3.setTitle(_translate("Dialog", "MIssion Timer"))
        self.mission_timer_label.setText(_translate("Dialog", "00:00"))
        self.start_timer_button.setText(_translate("Dialog", "Resume"))
        self.stop_timer_button.setText(_translate("Dialog", "Pause"))
        self.restart_timer_button.setText(_translate("Dialog", "Restart"))
        self.label_6.setText(_translate("Dialog", "Actions"))
        self.start_mission_button.setText(_translate("Dialog", "Start Mission"))
        self.e_stop_button.setText(_translate("Dialog", " E-Stop"))
        self.homing_button.setText(_translate("Dialog", "  Avengers Assemble"))
