#!/usr/bin/env python

import numpy as np
import math
import sys
import rospy
import pymap3d as pm
import time

from std_msgs.msg import String, UInt16, Float32
from geometry_msgs.msg import Point
from tricat_211.msg import FilteredObstacles, ObstaclePoint, Control, HeadingAngle

from PyQt5.QtWidgets import QApplication, QDialog, QGraphicsScene
from PyQt5.QtGui import QPen, QBrush, QCloseEvent
from PyQt5.Qt import QTimer
from PyQt5.QtCore import Qt, QLineF
from PyQt5.QtTest import QTest

from path_plan_simulation_window import UI_Simulator

class Obstacle:
    def __init__(self):
        rospy.Subscriber('/filtered_obstacles', FilteredObstacles, self.filtered_obstacle_callback)
        self.ob_list = np.empty((0, 3))
        # self.ob_list = np.array([[6, 4], [7, 4], [8, 4], [9, 4]])

    def filtered_obstacle_callback(self, msg):
        self.ob_list = np.empty((0, 3))
        filtered_obstacle_list = msg.obstacle
        for i in range(len(filtered_obstacle_list)):
            ob_x = filtered_obstacle_list[i].x
            ob_y = filtered_obstacle_list[i].y
            ob_theta = filtered_obstacle_list[i].theta
            self.ob_list = np.append(self.ob_list, np.array([[ob_x, ob_y, ob_theta]]), axis = 0)


class Boat:
    def __init__(self):
        self.cur_pos = np.zeros((0, 2))
        self.cur_heading = np.zeros((0, 2))
        self.bearing = 0.0

        rospy.Subscriber("/enu_position", Point, self.cur_pos_callback)
        rospy.Subscriber("/bearing", HeadingAngle, self.cur_bearing_callback)

    def cur_pos_callback(self, msg):
        self.cur_pos = np.array([msg.x, msg.y])
    
    def cur_bearing_callback(self, msg): #need to reconsider!!!!!, 0&180 degree???
        self.bearing = msg.bearing
        theta = msg.bearing * math.pi / 180
        self.cur_heading = np.array([math.sin(theta), math.cos(theta)])

class PathPlanner:
    def __init__(self):
        ## ROS
        path_plan_config = rospy.get_param("path_plan")
        self.servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

        ## PID
        self.init_servo = 94
        self.kp_servo = path_plan_config["kp_servo"]
        self.rate = 20

        ## edge points
        self.origin_point = list(path_plan_config["origin_point"]) #x, y, z
        self.edge_points = self.points_coordinate(path_plan_config["edge_points"])

        ## boat
        self.boat = Boat()
        # time.sleep(3)
        QTest.qWait(300) #?????????
        self.start_pos = self.boat.cur_pos
        self.start_heading = self.boat.cur_heading
        self.cur_pos = self.start_pos
        self.cur_heading = self.start_heading
        self.bearing = self.boat.bearing

        ## obstacles
        self.obstacle = Obstacle()
        self.obstacle_update()
        self.obstacle_search_range = path_plan_config["obstacle_search_range"]

        ## goal
        self.end_points = self.points_coordinate(path_plan_config["end_points"])
        self.next_goal = self.end_points[0]

        ## trajectory prediction
        self.predict_step = path_plan_config["predict_step"]
        self.predict_step_size = path_plan_config["predict_step_size"]
        self.g_value_rotate_gain_45 = path_plan_config["g_value_rotate_gain_45"]
        self.g_value_rotate_gain_90 = path_plan_config["g_value_rotate_gain_90"]
        self.g_value_rotate_gain_180 = path_plan_config["g_value_rotate_gain_180"]
        self.h_value_gain = path_plan_config["h_value_gain"]
        self.past_path = np.array([self.start_pos])
        self.trajectory = np.zeros((1, 2))  # [x, y]

        ## real moving
        self.arrival_range = path_plan_config["arrival_range"]
        self.thruster_power = path_plan_config["thruster_power"]

    def points_coordinate(self, points):
        converted = np.zeros_like(points)
        for i in range(len(points)):
            converted[i][0], converted[i][1] , converted[i][2] = \
                pm.geodetic2enu(points[i][0], points[i][1], points[i][2], \
                    self.origin_point[0], self.origin_point[1], self.origin_point[2])
        converted = np.delete(converted, 2, axis=1)
        return converted

    def is_finished(self):
        dist_to_goal = math.sqrt((self.cur_pos[0] - self.next_goal[0]) ** 2 + (self.cur_pos[1] - self.next_goal[1]) ** 2)
        if dist_to_goal <= self.arrival_range:
            return True
        else:
            return False

    def make_trajectory(self):
        search_center = np.array([self.cur_pos])
        predict_heading = self.cur_heading

        print("---124")
        print(search_center)

        search_center, predict_heading = self.best_point(search_center, predict_heading)

        self.trajectory = search_center  #init
    
        for i in range(self.predict_step - 1):
            if len(search_center)==0:
                print("-----130")
                continue
            search_center, predict_heading = self.best_point(search_center, predict_heading)
            self.trajectory = np.append(self.trajectory, search_center, axis=0)

    def best_point(self, search_center, predict_heading):
        min_f_value = 10000000
        best_point = np.zeros((0, 2))
        best_heading = np.zeros((0, 2))
        points = self.predict_step_size * np.array(
            [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        for i in range(8):  # better idea??
            points[i][0] += search_center[0][0]
            points[i][1] += search_center[0][1]

        for p in points:
            if self.is_line_out(p):
                continue
            if self.is_obstacle_in(p):
                continue

            # g
            distance_to_cur = (p[0] - self.cur_pos[0]) ** 2 + (p[1] - self.cur_pos[1]) ** 2

            # g
            vec = p - search_center[0]
            vec = vec / math.sqrt((vec[0] ** 2 + vec[1] ** 2))
            sin_theta = round(predict_heading[0] * vec[1] - predict_heading[1] * vec[0], 5)
            theta = math.asin(sin_theta) * 180 / math.pi  # degree

            distance_to_search = (p[0] - search_center[0][0]) ** 2 + (p[1] - search_center[0][1]) ** 2
            if 45 >= abs(theta):
                rotate_cost = distance_to_search * self.g_value_rotate_gain_45
            elif 90 >= abs(theta):
                rotate_cost = distance_to_search * self.g_value_rotate_gain_90
            else:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_180

            # g
            g = distance_to_cur + rotate_cost

            # h
            h = ((p[0] - self.next_goal[0]) ** 2 + (p[1] - self.next_goal[1]) ** 2) * self.h_value_gain

            # f
            f = g + h

            # min f
            if min_f_value > f:
                if np.dot(vec, self.cur_heading) == -1:
                    min_f_value = f
                    best_point = np.array([search_center[0] + self.cur_heading])
                    best_heading = self.cur_heading
                else:
                    min_f_value = f
                    best_point = np.array([p])
                    best_heading = vec

        return best_point, best_heading

    def is_line_out(self, point):
        crossed = 0
        for i in range(len(self.edge_points)):
            j = (i + 1) % len(self.edge_points)
            if (self.edge_points[i][1] > point[1]) != (self.edge_points[j][1] > point[1]):
                intersection = float((self.edge_points[j][0] - self.edge_points[i][0]) * (
                        point[1] - self.edge_points[i][1]) / (
                                             self.edge_points[j][1] - self.edge_points[i][1]) +
                                     self.edge_points[i][0])
                if point[0] < intersection:
                    crossed = crossed + 1
        return (crossed % 2) == 0

    def is_obstacle_in(self, point):
        if len(self.ob_list) == 0:
            return False

        for i in range(len(self.ob_list)):
            distance_to_ob = math.sqrt((point[0] - self.ob_list[i][0]) ** 2 + (point[1] - self.ob_list[i][1]) ** 2)
            if distance_to_ob < self.obstacle_search_range:
                return True

        return False

    def save_past_path(self):
        self.past_path = np.append(self.past_path, np.array([self.cur_pos]), axis=0)
        
    def obstacle_update(self):
        self.ob_list = self.obstacle.ob_list
        for i in range(len(self.ob_list)):
            d = math.sqrt(self.ob_list[i][0]**2 + self.ob_list[i][1]**2)
            self.ob_list[i][0] = self.cur_pos[0] + d * math.cos(self.bearing - self.ob_list[i][2])
            self.ob_list[i][1] = self.cur_pos[1] + d * math.sin(self.bearing - self.ob_list[i][2])
        self.ob_list = self.ob_list[:, 0:2]
    
    def servo_pid_controller(self):
        rotate_angle = math.atan2((self.trajectory[0][0] - self.cur_pos[0]), (self.trajectory[0][1] - self.cur_pos[1]))
        rotate_angle = rotate_angle * 180 / math.pi
        error_angle = rotate_angle - self.bearing # need to check!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.cp_servo = self.kp_servo * error_angle
        servo_pd = -(self.cp_servo)
        servo_control = self.init_servo + servo_pd

        # need to check!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if servo_control > 118: #94+24
            servo_control = 118
        elif servo_control < 70:# 94-24
            servo_control = 70
        else:
            pass

        return servo_control

    def control_publisher(self):
        control = Control()
        control.thruster = self.thruster_power
        control.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(control.thruster)
        self.servo_pub.publish(control.servo)

class Controller:
    def __init__(self):
        rospy.init_node('Path_Plan', anonymous=False)
        self.pp = PathPlanner()

    def board_show(self):
        cur_pos_str = "( " + str(round(self.pp.cur_pos[0], 2)) + " , " + str(
            round(self.pp.cur_pos[1], 2)) + " )"
        window.ui.cur_pos_lineEdit.setText(cur_pos_str)

        cur_heading_str = "( " + str(round(self.pp.cur_heading[0], 2)) + " , " + str(
            round(self.pp.cur_heading[1], 2)) + " )"
        window.ui.cur_heading_lineEdit.setText(cur_heading_str)

        next_goal_str = "( " + str(round(self.pp.next_goal[0], 2)) + " , " + str(
            round(self.pp.next_goal[1], 2)) + " )"
        window.ui.next_goal_lineEdit.setText(next_goal_str)

    def run_to_goal(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.board_show()
            if self.pp.is_finished():
                print("Arrived Next Goal")
                if len(self.pp.end_points) > 1:
                    self.pp.end_points = np.delete(self.pp.end_points, 0, axis=0)
                    self.pp.next_goal = self.pp.end_points[0]
                elif len(self.pp.end_points) == 1:
                    self.pp.next_goal = self.pp.end_points[0]
                    self.pp.end_points = np.delete(self.pp.end_points, 0, axis=0)
                else:
                    print("Arrived Final Goal")
                    window.timer.stop()
                    break
            else:
                print("Not Arrived")
                self.pp.obstacle_update()
                self.pp.make_trajectory()
                self.pp.save_past_path()
                print("-----------------------------------------------------------------")
                print("len : " + str(len(self.pp.trajectory)))
                print("trajectory[0] : " + str(round(self.pp.trajectory[0][0], 2)) + " , " + str(round(self.pp.trajectory[0][1], 2)))
                print(self.trajectory)
                print("cur_pos : " + str(round(self.pp.cur_pos[0], 2)) + " , " + str(round(self.pp.cur_pos[1], 2)))
                print("bearing : " + str(self.pp.bearing))
                window.draw(self.pp.edge_points, self.pp.cur_pos, self.pp.trajectory,
                                self.pp.start_pos, self.pp.past_path, self.pp.ob_list,
                                self.pp.end_points, self.pp.arrival_range)
                # QTest.qWait(150) #?????????
                self.pp.control_publisher()
                self.pp.cur_pos = self.pp.boat.cur_pos
            rate.sleep()

        rospy.spin()

class SimulationWindow(QDialog):
    controller = Controller()
    timer = QTimer()

    def __init__(self, parent=None):
        super(SimulationWindow, self).__init__(parent)
        # self.controller = Controller()
        # self.timer = QTimer()
        self.ui = UI_Simulator()
        self.ui.set_UI(self)
        self.timer.start(50)
        self.timer.timeout.connect(self.controller.run_to_goal)

        # ##Simulator Initial Values
        self.ui.ob_search_range_lineEdit.setText(str(self.controller.pp.obstacle_search_range))
        self.ui.predict_step_lineEdit.setText(str(self.controller.pp.predict_step))
        self.ui.predict_step_size_lineEdit.setText(str(self.controller.pp.predict_step_size))
        self.ui.g_value_rotate_gain_45_lineEdit.setText(str(self.controller.pp.g_value_rotate_gain_45))
        self.ui.g_value_rotate_gain_90_lineEdit.setText(str(self.controller.pp.g_value_rotate_gain_90))
        self.ui.g_value_rotate_gain_180_lineEdit.setText(str(self.controller.pp.g_value_rotate_gain_180))
        self.ui.h_value_gain_lineEdit.setText(str(self.controller.pp.h_value_gain))
        self.ui.arrival_range_lineEdit.setText(str(self.controller.pp.arrival_range))

    def draw(self, edge_points, cur_pos, trajectory, start_pos, past_path, obstacles, end_points, arrival_range):
        scale = 0.09
        c = 1 / scale

        self.scene = GraphicsScene()
        self.ui.graphicsView.setScene(self.scene)

        pen_edges = QPen(Qt.black, 2)
        for i in range(len(edge_points)):
            j = (i + 1) % len(edge_points)
            self.scene.addLine(QLineF(c * edge_points[i][0], -c * edge_points[i][1],
                                      c * edge_points[j][0], -c * edge_points[j][1]), pen_edges)

        pen_grid = QPen(Qt.black, 0.5, Qt.DotLine)
        x_edge_end = [edge_points[0][0], edge_points[0][0]]  # [min, max]
        y_edge_end = [edge_points[0][1], edge_points[0][1]]
        for i in range(len(edge_points)):
            if edge_points[i][0] < x_edge_end[0]:
                x_edge_end[0] = edge_points[i][0]
            if edge_points[i][0] > x_edge_end[1]:  
                x_edge_end[1] = edge_points[i][0]
            if edge_points[i][1] < y_edge_end[0]:
                y_edge_end[0] = edge_points[i][1]
            if edge_points[i][1] > y_edge_end[1]:
                y_edge_end[1] = edge_points[i][1]

        x_grid_num = int((x_edge_end[1] - x_edge_end[0]) / 5)
        y_grid_num = int((y_edge_end[1] - y_edge_end[0]) / 5)
        for i in range(x_grid_num + 1):
            self.scene.addLine(QLineF(c * int(x_edge_end[0] + 5 * i), -c * y_edge_end[0],
                                      c * int(x_edge_end[0] + 5 * i), -c * y_edge_end[1]), pen_grid)
        for i in range(y_grid_num + 1):
            self.scene.addLine(QLineF(c * x_edge_end[0], -c * int(y_edge_end[0] + 5 * i),
                                      c * x_edge_end[1], -c * int(y_edge_end[0] + 5 * i)), pen_grid)

        pen_pastPath = QPen(Qt.green, 1.5)
        if len(past_path) > 1:
            for i in range(len(past_path) - 1):
                self.scene.addLine(QLineF(c * past_path[i][0], -c * past_path[i][1],
                                          c * past_path[i + 1][0], -c * past_path[i + 1][1]), pen_pastPath)

        pen_trajectory = QPen(Qt.red)
        for i in range(len(trajectory) - 1):
            self.scene.addLine(QLineF(c * trajectory[i][0], -c * trajectory[i][1],
                                      c * trajectory[i + 1][0], -c * trajectory[i + 1][1]), pen_trajectory)

        pen_obstacle = QPen(Qt.black)
        for i in range(len(obstacles)):
            obstacle_diameter = c * 0.4
            self.scene.addEllipse(c * obstacles[i][0] - obstacle_diameter / 2,
                                  -c * obstacles[i][1] - obstacle_diameter / 2,
                                  obstacle_diameter, obstacle_diameter, pen_obstacle, QBrush(Qt.black))

        pen_boat = QPen(Qt.green)
        diameter = c * 0.4
        self.scene.addEllipse(c * cur_pos[0] - diameter / 2, -c * cur_pos[1] - diameter / 2,
                              diameter, diameter, pen_boat, QBrush(Qt.green))

        pen_start = QPen(Qt.red)
        diameter = c * 0.4
        self.scene.addEllipse(c * start_pos[0] - diameter / 2, -c * start_pos[1] - diameter / 2,
                              diameter, diameter, pen_start, QBrush(Qt.red))

        pen_arrival_range = QPen(Qt.blue, 0.8)
        for i in range(len(end_points)):
            diameter = c * arrival_range
            self.scene.addEllipse(c * end_points[i][0] - diameter / 2, -c * end_points[i][1] - diameter / 2,
                                  diameter, diameter, pen_arrival_range, QBrush(Qt.white))
            pen_end = QPen(Qt.blue)
            diameter = c * 0.4
            self.scene.addEllipse(c * end_points[i][0] - diameter / 2, -c * end_points[i][1] - diameter / 2,
                                  diameter, diameter, pen_end, QBrush(Qt.blue))

class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent=None)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SimulationWindow()
    window.show()
    sys.exit(app.exec_())
