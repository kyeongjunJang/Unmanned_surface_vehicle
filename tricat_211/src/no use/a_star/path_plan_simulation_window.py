#!/usr/bin/env python

from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets


class UI_Simulator(object):
    def set_UI(self, A_star_Simulator):
        ################ Simulator
        A_star_Simulator.setObjectName("A_star_Simulator")
        A_star_Simulator.resize(1100, 750)

        ################ Widgets
        ### QGraphicView
        self.graphicsView = QtWidgets.QGraphicsView(A_star_Simulator)
        # self.graphicsView.setGeometry(QtCore.QRect(0, 0, 600, 600))
        self.graphicsView.setObjectName("graphicsView") 

        ### QLabel : Obstacle Search Range
        self.ob_search_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.ob_search_range_label.setObjectName("ob_search_range_label")
        ### QLineEdit : Obstacle Search Range
        self.ob_search_range_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.ob_search_range_lineEdit.setObjectName("ob_search_range_lineEdit")

        ### QLabel : Predict Step
        self.predict_step_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_label.setObjectName("predict_step_label")
        ### QSpinBox : Predict Step
        self.predict_step_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.predict_step_lineEdit.setObjectName("predict_step_lineEdit")

        ### QLabel : Predict Step Size
        self.predict_step_size_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_size_label.setObjectName("predict_step_size_label")
        ### QLineEdit : Predict Step Size
        self.predict_step_size_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.predict_step_size_lineEdit.setObjectName("predict_step_size_lineEdit")

        ### QLabel : g-value Rotate Gain(0)
        self.g_value_rotate_gain_0_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_0_label.setObjectName("g_value_rotate_gain_0_label")
        ### QLineEdit : g-value Rotate Gain(0)
        self.g_value_rotate_gain_0_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.g_value_rotate_gain_0_lineEdit.setObjectName("g_value_rotate_gain_0_lineEdit")

        ### QLabel : g-value Rotate Gain(30)
        self.g_value_rotate_gain_30_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_30_label.setObjectName("g_value_rotate_gain_30_label")
        ### QLineEdit : g-value Rotate Gain(30)
        self.g_value_rotate_gain_30_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.g_value_rotate_gain_30_lineEdit.setObjectName("g_value_rotate_gain_30_lineEdit")

        ### QLabel : g-value Rotate Gain(60)
        self.g_value_rotate_gain_60_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_60_label.setObjectName("g_value_rotate_gain_60_label")
        ### QLineEdit : g-value Rotate Gain(60)
        self.g_value_rotate_gain_60_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.g_value_rotate_gain_60_lineEdit.setObjectName("g_value_rotate_gain_60_lineEdit")

        ### QLabel : g-value Rotate Gain(90)
        self.g_value_rotate_gain_90_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_90_label.setObjectName("g_value_rotate_gain_90_label")
        ### QLineEdit : g-value Rotate Gain(90)
        self.g_value_rotate_gain_90_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.g_value_rotate_gain_90_lineEdit.setObjectName("g_value_rotate_gain_90_lineEdit")

        ### QLabel : h-value Gain
        self.h_value_gain_label = QtWidgets.QLabel(A_star_Simulator)
        self.h_value_gain_label.setObjectName("h_value_gain_label")

        ### QLineEdit : h-value Gain
        self.h_value_gain_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.h_value_gain_lineEdit.setObjectName("h_value_gain_lineEdit")

        ### QLabel : Arrival Range
        self.arrival_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.arrival_range_label.setObjectName("arrival_range_label")
        ### QLineEdit : Arrival Range
        self.arrival_range_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.arrival_range_lineEdit.setObjectName("arrival_range_lineEdit")

        ### QLabel : servo kp
        self.kp_servo_label = QtWidgets.QLabel(A_star_Simulator)
        self.kp_servo_label.setObjectName("kp_servo_label")
        ### QLineEdit : servo kp
        self.kp_servo_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.kp_servo_lineEdit.setObjectName("kp_servo_lineEdit")

        ### QLabel : Current Position Label
        self.cur_pos_label = QtWidgets.QLabel(A_star_Simulator)
        self.cur_pos_label.setObjectName("cur_pos_label")
        ### QLineEdit : Current Position Text
        self.cur_pos_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.cur_pos_lineEdit.setObjectName("cur_pos_lineEdit")

        ### QLabel : Current Bearing Label
        self.cur_bearing_label = QtWidgets.QLabel(A_star_Simulator)
        self.cur_bearing_label.setObjectName("cur_bearing_label")
        ### QLineEdit : Current Bearing Text
        self.cur_bearing_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.cur_bearing_lineEdit.setObjectName("cur_bearing_lineEdit")

        ### QLabel : Current Heading Label
        self.cur_heading_label = QtWidgets.QLabel(A_star_Simulator)
        self.cur_heading_label.setObjectName("cur_heading_label")
        ### QLineEdit : Current Heading Text
        self.cur_heading_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.cur_heading_lineEdit.setObjectName("cur_heading_lineEdit")

        ### QLabel : Next Goal
        self.next_goal_label = QtWidgets.QLabel(A_star_Simulator)
        self.next_goal_label.setObjectName("next_goal_label")
        ### QLineEdit : Next Goal
        self.next_goal_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.next_goal_lineEdit.setObjectName("next_goal_lineEdit")

        ### QLabel : Trajectory[0]
        self.trajectory_0_label = QtWidgets.QLabel(A_star_Simulator)
        self.trajectory_0_label.setObjectName("trajectory_0_label")
        ### QLineEdit : Next Goal
        self.trajectory_0_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.trajectory_0_lineEdit.setObjectName("trajectory_0_lineEdit")

        ### QLabel : Rotate Angle
        self.rotate_angle_label = QtWidgets.QLabel(A_star_Simulator)
        self.rotate_angle_label.setObjectName("rotate_angle_label")
        ### QLineEdit : Rotate Angle
        self.rotate_angle_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.rotate_angle_lineEdit.setObjectName("rotate_angle_lineEdit")

        ### QLabel : Error Angle
        self.error_angle_label = QtWidgets.QLabel(A_star_Simulator)
        self.error_angle_label.setObjectName("error_angle_label")
        ### QLineEdit : Error Angle
        self.error_angle_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.error_angle_lineEdit.setObjectName("error_angle_lineEdit")

        ### QLabel : Servo Control
        self.servo_control_label = QtWidgets.QLabel(A_star_Simulator)
        self.servo_control_label.setObjectName("servo_control_label")
        ### QLineEdit : Error Angle
        self.servo_control_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.servo_control_lineEdit.setObjectName("servo_control_lineEdit")

        ################ Layout
        ### Main Layout
        layout = QtWidgets.QGridLayout()
        A_star_Simulator.setLayout(layout)

        ### Settings Layout
        settings_groupBox = QtWidgets.QGroupBox('Settings') 
        settings_grid = QtWidgets.QGridLayout()

        #addWidget(widget name, row, col, row span, col span)
        settings_grid.addWidget(self.ob_search_range_label, 0, 0, 1, 2)
        settings_grid.addWidget(self.ob_search_range_lineEdit, 0, 2, 1, 2)

        settings_grid.addWidget(self.predict_step_label, 1, 0, 1, 2)
        settings_grid.addWidget(self.predict_step_lineEdit, 1, 2, 1, 2)

        settings_grid.addWidget(self.predict_step_size_label, 2, 0, 1, 2)
        settings_grid.addWidget(self.predict_step_size_lineEdit, 2, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_0_label, 3, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_0_lineEdit, 3, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_30_label, 4, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_30_lineEdit, 4, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_60_label, 5, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_60_lineEdit, 5, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_90_label, 6, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_90_lineEdit, 6, 2, 1, 2)

        settings_grid.addWidget(self.h_value_gain_label, 7, 0, 1, 2)
        settings_grid.addWidget(self.h_value_gain_lineEdit, 7, 2, 1, 2)

        settings_grid.addWidget(self.arrival_range_label, 8, 0, 1, 2)
        settings_grid.addWidget(self.arrival_range_lineEdit, 8, 2, 1, 2)

        settings_grid.addWidget(self.kp_servo_label, 9, 0, 1, 2)
        settings_grid.addWidget(self.kp_servo_lineEdit, 9, 2, 1, 2)

        settings_groupBox.setLayout(settings_grid)

        ### Board Layout
        board_groupBox = QtWidgets.QGroupBox('Board') 
        board_grid = QtWidgets.QGridLayout()

        board_grid.addWidget(self.cur_pos_label, 0, 0, 1, 2)
        board_grid.addWidget(self.cur_pos_lineEdit, 0, 2, 1, 2)

        board_grid.addWidget(self.cur_bearing_label, 1, 0, 1, 2)
        board_grid.addWidget(self.cur_bearing_lineEdit, 1, 2, 1, 2)

        board_grid.addWidget(self.cur_heading_label, 2, 0, 1, 2)
        board_grid.addWidget(self.cur_heading_lineEdit, 2, 2, 1, 2)

        board_grid.addWidget(self.next_goal_label, 3, 0, 1, 2)
        board_grid.addWidget(self.next_goal_lineEdit, 3, 2, 1, 2)

        board_grid.addWidget(self.trajectory_0_label, 4, 0, 1, 2)
        board_grid.addWidget(self.trajectory_0_lineEdit, 4, 2, 1, 2)

        board_grid.addWidget(self.rotate_angle_label, 5, 0, 1, 2)
        board_grid.addWidget(self.rotate_angle_lineEdit, 5, 2, 1, 2)

        board_grid.addWidget(self.error_angle_label, 6, 0, 1, 2)
        board_grid.addWidget(self.error_angle_lineEdit, 6, 2, 1, 2)

        board_grid.addWidget(self.servo_control_label, 7, 0, 1, 2)
        board_grid.addWidget(self.servo_control_lineEdit, 7, 2, 1, 2)

        board_groupBox.setLayout(board_grid)

        ### Main Layout
        layout.addWidget(self.graphicsView, 0, 0, 18, 15)

        layout.addWidget(settings_groupBox, 0, 15, 10, 4)
        layout.addWidget(board_groupBox, 10, 15, 8, 4)

        ################ Initial Setting
        self.retranslate_UI(A_star_Simulator)

    def retranslate_UI(self, A_star_Simulator):
        _translate = QtCore.QCoreApplication.translate
        A_star_Simulator.setWindowTitle(_translate("A_star_Simulator", "A star Simulator"))

        self.ob_search_range_label.setText(_translate("A_star_Simulator", "Obstacle Search Range"))
        self.predict_step_label.setText(_translate("A_star_Simulator", "Predict Step"))
        self.predict_step_size_label.setText(_translate("A_star_Simulator", "Predict Step Size"))
        self.g_value_rotate_gain_0_label.setText(_translate("A_star_Simulator", "g:  0 Rotate Gain"))
        self.g_value_rotate_gain_30_label.setText(_translate("A_star_Simulator", "g: 30 Rotate Gain"))
        self.g_value_rotate_gain_60_label.setText(_translate("A_star_Simulator", "g: 60 Rotate Gain"))
        self.g_value_rotate_gain_90_label.setText(_translate("A_star_Simulator", "   90 Rotate Gain"))
        self.h_value_gain_label.setText(_translate("A_star_Simulator", "h: Goal Cost Gain"))
        self.arrival_range_label.setText(_translate("A_star_Simulator", "Arrival Check Range"))
        self.kp_servo_label.setText(_translate("A_star_Simulator", "Kp Servo"))
        
        self.cur_pos_label.setText(_translate("A_star_Simulator", "Current Position"))
        self.cur_bearing_label.setText(_translate("A_star_Simulator", "Bearing"))
        self.cur_heading_label.setText(_translate("A_star_Simulator", "Current Heading"))
        self.next_goal_label.setText(_translate("A_star_Simulator", "Next Goal"))
        self.trajectory_0_label.setText(_translate("A_star_Simulator", "Trajectory[0]"))
        self.rotate_angle_label.setText(_translate("A_star_Simulator", "Rotate Angle"))
        self.error_angle_label.setText(_translate("A_star_Simulator", "Error Angle"))
        self.servo_control_label.setText(_translate("A_star_Simulator", "Servo Control"))