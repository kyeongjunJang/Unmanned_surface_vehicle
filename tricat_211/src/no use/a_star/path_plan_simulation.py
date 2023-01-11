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

    def filtered_obstacle_callback(self, msg):
        self.ob_list = np.empty((0, 3)) #init
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
    
    def cur_bearing_callback(self, msg):
        self.bearing = msg.bearing #degree
        theta = math.radians(msg.bearing)
        self.cur_heading = np.array([math.sin(theta), math.cos(theta)])
        self.cur_heading /= math.sqrt((self.cur_heading[0] ** 2 + self.cur_heading[1] ** 2))

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
        self.g_value_rotate_gain_0 = path_plan_config["g_value_rotate_gain_0"]
        self.g_value_rotate_gain_30 = path_plan_config["g_value_rotate_gain_30"]
        self.g_value_rotate_gain_60 = path_plan_config["g_value_rotate_gain_60"]
        self.g_value_rotate_gain_90 = path_plan_config["g_value_rotate_gain_90"]
        self.h_value_gain = path_plan_config["h_value_gain"]
        self.past_path = self.start_pos
        self.trajectory = np.zeros((1, 2))

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
        print("|{:-<82}+".format(" make_trajectory()"))
        search_center = self.cur_pos
        predict_heading = self.cur_heading
        self.trajectory = np.zeros((1, 2))

        for i in range(self.predict_step):
            print("| {0:<16} | {1:>8} , {2:>8} || {3:<16} | {4:>8} , {5:>8} |".format(\
                "search_center", str(round(search_center[0], 2)), str(round(search_center[1], 2)),\
                "predict_heading", str(round(predict_heading[0], 2)), str(round(predict_heading[1], 2))))
            search_center, predict_heading = self.best_point(search_center, predict_heading)
            print("| {0} {1:<14} | {2:>8} , {3:>8}    {4:>38} |".format(\
                i, "best_point", str(round(search_center[0], 2)), str(round(search_center[1], 2)), ""))
            self.trajectory = np.append(self.trajectory, np.array([search_center]), axis=0)
        self.trajectory = np.delete(self.trajectory, 0, axis=0)
        print("| {0:<16} | {1:61} |".format("trajectory", ""))
        for x, y in self.trajectory:
            print("| {0:16} | {1:>8} , {2:>8} {3:41} |".format("", str(round(x, 2)), str(round(y, 2)), ""))

    def best_point(self, search_center, predict_heading):
        min_f_value = 10000000
        best_point = np.array([search_center[0] + predict_heading[0] * self.predict_step_size,
                     search_center[1] + predict_heading[1] * self.predict_step_size])
        best_heading = predict_heading

        points = self.predict_step_size * np.array([
            [1, 0], [0.87, 0.50], [0.50, 0.87], [0, 1], [-0.50, 0.87], [-0.87, 0.50],
            [-1, 0], [-0.87, -0.50], [-0.50, -0.87], [0, -1], [0.50, -0.87], [0.87, -0.50]])
        points += search_center

        cnt = 0
        for p in points:
            if self.is_line_out(p):
                continue
            if self.is_obstacle_in(p):
                continue

            cnt += 1
            # g
            distance_to_cur = (p[0] - self.cur_pos[0]) ** 2 + (p[1] - self.cur_pos[1]) ** 2

            # g
            vec = p - search_center
            vec = vec / math.sqrt((vec[0] ** 2 + vec[1] ** 2))
            sin_value = round(predict_heading[0] * vec[1] - predict_heading[1] * vec[0], 5)
            rotate_angle = abs(math.degrees(math.asin(sin_value)))  # degree

            distance_to_search = (p[0] - search_center[0]) ** 2 + (p[1] - search_center[1]) ** 2
            if 0 <= rotate_angle < 30:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_0
            elif 30 <= rotate_angle < 60:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_30
            elif 60 <= rotate_angle < 90:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_60
            else: # 90 < rotate_angle <= 180:
                rotate_cost = distance_to_search * self.g_value_rotate_gain_90

            # g
            g = distance_to_cur + rotate_cost

            # h
            h = ((p[0] - self.next_goal[0]) ** 2 + (p[1] - self.next_goal[1]) ** 2) * self.h_value_gain

            # f
            f = g + h

            # min f
            if min_f_value > f:
                min_f_value = f
                if np.dot(vec, predict_heading) == -1:
                    print("|{:-<82}+".format(" make_trajectory() / best_point() / It Stucked!"))
                    best_point = np.array([search_center[0] + predict_heading[0] * self.predict_step_size,
                     search_center[1] + predict_heading[1] * self.predict_step_size])
                    best_heading = predict_heading
                else:
                    best_point = np.array([p[0], p[1]])
                    best_heading = vec
        print("| make_trajectory() / best_point() / {0} of 8 points are calculated{1:-<18}+".format(cnt, ""))

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
        return (crossed % 2) == 0 #line out = True

    def is_obstacle_in(self, point):
        if len(self.ob_list) == 0:
            return False

        for i in range(len(self.ob_list)):
            distance_to_ob = math.sqrt((point[0] - self.ob_list[i][0]) ** 2 + (point[1] - self.ob_list[i][1]) ** 2)
            if distance_to_ob < self.obstacle_search_range:
                return True #obstacle in = True

        return False

    def save_past_path(self):
        self.past_path = np.append(self.past_path, np.array([self.cur_pos]), axis=0)
        
    def obstacle_update(self):
        print("|{:-<82}+".format(" obstacle_update()"))
        self.ob_list = self.obstacle.ob_list
        for i in range(len(self.ob_list)):
            bearing = self.bearing
            theta = self.ob_list[i][2]
            x = self.cur_pos[0]
            y = self.cur_pos[1]
            d = math.sqrt(self.ob_list[i][0]**2 + self.ob_list[i][1]**2)
            if 0 <= bearing <= 180: # bearing Quadrant 1, 4
                if 0 <= theta <= (180 - bearing):
                    x += d * math.sin(math.radians(abs(theta) + abs(bearing)))
                    y += d * math.cos(math.radians(abs(theta) + abs(bearing)))
                elif (180 - bearing) <= theta < 180:
                    x += d * math.sin(math.radians(-360 + abs(theta) + abs(bearing)))
                    y += d * math.cos(math.radians(-360 + abs(theta) + abs(bearing)))
                else:
                    x += d * math.sin(math.radians(-abs(theta) + abs(bearing)))
                    y += d * math.cos(math.radians(-abs(theta) + abs(bearing)))
            else: # -180 < bearing < 0 # bearing Quadrant 2, 3
                if 0 <= theta <= 180:
                    x += d * math.sin(math.radians(abs(theta) - abs(bearing)))
                    y += d * math.cos(math.radians(abs(theta) - abs(bearing)))
                elif -180 < theta < (-180 - bearing):
                    x += d * math.sin(math.radians(360 - abs(theta) - abs(bearing)))
                    y += d * math.cos(math.radians(360 - abs(theta) - abs(bearing)))
                else:
                    x += d * math.sin(math.radians(-abs(theta) - abs(bearing)))
                    y += d * math.cos(math.radians(-abs(theta) - abs(bearing)))
            self.ob_list[i][0] = x
            self.ob_list[i][1] = y
        self.ob_list = self.ob_list[:, 0:2]
        print("| {0:<16} | {1:61} |".format("ob_list", ""))
        for x, y in self.ob_list:
            print("| {0:16} | {1:>8} , {2:>8} {3:41} |".format("", str(round(x, 2)), str(round(y, 2)), ""))
     
    def servo_pid_controller(self):
        dx = self.trajectory[0][0] - self.cur_pos[0]
        dy = self.trajectory[0][1] - self.cur_pos[1]
        self.angle = math.degrees(math.atan2(dx, dy))

        if dx >= 0 and dy >= 0: # Quadrant 1
            if self.bearing >= 0 : # right bearing
                error_angle = self.angle - self.bearing
            elif self.bearing < 0: # left bearing
                if abs(self.bearing) < (180 - self.angle):
                    error_angle = self.angle - self.bearing
                elif abs(self.bearing) >= (180 - self.angle):
                    error_angle = -(360 - self.angle + self.bearing)

        elif dx < 0 and dy >= 0: # Quadrant 2
            if self.bearing >= 0 :
                if self.bearing < 180 + self.angle:
                    error_angle = self.angle - self.bearing
                elif self.bearing >= 180 + self.angle:
                    error_angle = 360 + self.angle - self.bearing
            elif self.bearing < 0:
                error_angle = self.angle - self.bearing
                
        elif dx < 0 and dy < 0: # Quadrant 3
            if self.bearing >= 0 :
                if self.bearing < 180 + self.angle:
                    error_angle = (self.angle - self.bearing)
                elif self.bearing >= 180 + self.angle:
                    error_angle = 360 + (self.angle - self.bearing)
            elif self.bearing < 0:
                error_angle = (self.angle - self.bearing)

        elif dx >= 0 and dy < 0: # Quadrant 4
            if self.bearing >= 0 :
                error_angle = (self.angle - self.bearing)
            elif self.bearing < 0:
                if abs(self.bearing) < 180 - self.angle:
                    error_angle = (self.angle - self.bearing)
                elif abs(self.bearing) >= 180 - self.angle:
                    error_angle = self.angle - self.bearing - 360

        self.cp_servo = self.kp_servo * error_angle
        servo_pd = -(self.cp_servo)
        servo_control = self.init_servo + servo_pd

        if servo_control > 118: #94+24
            servo_control = 118
        elif servo_control < 70:# 94-24
            servo_control = 70
        else:
            pass

        print("| rotate_angle   {0:>8}  | error_angle   {1:>8}  | servo_control   {2:>11} |".format(str(round(self.angle)), str(round(error_angle)), str(round(servo_control))))

        window.ui.rotate_angle_lineEdit.setText(str(round(self.angle)))
        window.ui.error_angle_lineEdit.setText(str(round(error_angle)))
        window.ui.servo_control_lineEdit.setText(str(round(servo_control)))

        return servo_control

    def control_publisher(self):
        print("|{:-<82}+".format(" control_publisher()"))
        control = Control()
        control.thruster = self.thruster_power
        control.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(control.thruster)
        self.servo_pub.publish(control.servo)

    def print_info(self):
        print("+" + "-"*82 + "+")
        print("+" + "-"*82 + "+")
        print("| {0:<16} | {1:>8} , {2:>8} || {3:<16} | {4:>19} |".format(\
            "cur_pos", str(round(self.cur_pos[0], 2)), str(round(self.cur_pos[1], 2)),\
            "bearing", str(round(self.bearing))))
        print("| {0:<16} | {1:>8} , {2:>8} || {3:<39}|".format(\
            "next_goal", str(round(self.next_goal[0], 3)), str(round(self.next_goal[1], 3)), ""))
        print("| {0:<16} | {1:>8} , {2:>8} || {3:<16} | {4:>8} , {5:>8} |".format(\
            "trajectory[0]", str(round(self.cur_pos[0], 2)), str(round(self.cur_pos[1], 2)),\
            "heading", str(round(self.cur_heading[0], 2)), str(round(self.cur_heading[1], 2))))

class Controller:
    def __init__(self):
        rospy.init_node('Path_Plan', anonymous=False)
        self.pp = PathPlanner()

    def board_show(self):
        cur_pos_str = "( " + str(round(self.pp.cur_pos[0], 2)) + " , " + str(
            round(self.pp.cur_pos[1], 2)) + " )"
        window.ui.cur_pos_lineEdit.setText(cur_pos_str)

        window.ui.cur_bearing_lineEdit.setText(str(self.pp.bearing))

        cur_heading_str = "( " + str(round(self.pp.cur_heading[0], 2)) + " , " + str(
            round(self.pp.cur_heading[1], 2)) + " )"
        window.ui.cur_heading_lineEdit.setText(cur_heading_str)

        next_goal_str = "( " + str(round(self.pp.next_goal[0], 2)) + " , " + str(
            round(self.pp.next_goal[1], 2)) + " )"
        window.ui.next_goal_lineEdit.setText(next_goal_str)

        trajectory_0_str = "( " + str(round(self.pp.trajectory[0][0], 2)) + " , " + str(
            round(self.pp.trajectory[0][1], 2)) + " )"
        window.ui.trajectory_0_lineEdit.setText(trajectory_0_str)

    def run_to_goal(self):
        rate = rospy.Rate(10)
        hz = 1 # hz * 0.1 sec
        time.sleep(3)
        self.pp.start_pos = self.pp.boat.cur_pos
        while not rospy.is_shutdown():
            self.pp.cur_pos = self.pp.boat.cur_pos
            self.pp.cur_heading = self.pp.boat.cur_heading
            self.pp.bearing = self.pp.boat.bearing

            self.board_show()

            hz -= 1
            self.pp.print_info()

            if self.pp.is_finished():
                print("|{:-<82}+".format(" main loop / Arrived Next Goal"))
                if len(self.pp.end_points) > 1:
                    print("|{:-<82}+".format(" main loop / Arrived Next Goal / Set Next Goal"))
                    self.pp.end_points = np.delete(self.pp.end_points, 0, axis=0)
                    self.pp.next_goal = self.pp.end_points[0]
                elif len(self.pp.end_points) == 1:
                    print("|{:-<82}+".format(" main loop / Arrived Next Goal / Set Next Goal"))
                    self.pp.next_goal = self.pp.end_points[0]
                    self.pp.end_points = self.np.delete(self.pp.end_points, 0, axis=0)
                else:
                    print("|{:-<82}+".format(" main loop / Arrived Next Goal / Arrived Final Goal"))
                    self.pp.thruster_pub.publish(1500)
                    self.pp.servo_pub.publish(93)
                    window.timer.stop()
                    break
            else:
                print("|{:-<82}+".format(" main loop / Not Arrived"))
                self.pp.obstacle_update()
                if hz < 1:
                    self.pp.make_trajectory()
                    hz = 20
                self.pp.control_publisher()
                self.pp.save_past_path()
                window.draw(self.pp.edge_points, self.pp.cur_pos, self.pp.trajectory,
                            self.pp.start_pos, self.pp.past_path, self.pp.ob_list,
                            self.pp.end_points, self.pp.arrival_range)

            rate.sleep()
        rospy.spin()

class SimulationWindow(QDialog):
    controller = Controller()
    timer = QTimer()
    def __init__(self, parent=None):
        super(SimulationWindow, self).__init__(parent)
        self.ui = UI_Simulator()
        self.ui.set_UI(self)
        # self.controller = Controller()
        # self.timer = QTimer()
        self.timer.start(50)
        self.timer.timeout.connect(self.controller.run_to_goal)

        # ##Simulator Initial Values
        self.ui.ob_search_range_lineEdit.setText(str(self.controller.pp.obstacle_search_range))
        self.ui.predict_step_lineEdit.setText(str(self.controller.pp.predict_step))
        self.ui.predict_step_size_lineEdit.setText(str(self.controller.pp.predict_step_size))
        self.ui.g_value_rotate_gain_0_lineEdit.setText(str(self.controller.pp.g_value_rotate_gain_0))
        self.ui.g_value_rotate_gain_30_lineEdit.setText(str(self.controller.pp.g_value_rotate_gain_30))
        self.ui.g_value_rotate_gain_60_lineEdit.setText(str(self.controller.pp.g_value_rotate_gain_60))
        self.ui.g_value_rotate_gain_90_lineEdit.setText(str(self.controller.pp.g_value_rotate_gain_90))
        self.ui.h_value_gain_lineEdit.setText(str(self.controller.pp.h_value_gain))
        self.ui.arrival_range_lineEdit.setText(str(self.controller.pp.arrival_range))
        self.ui.kp_servo_lineEdit.setText(str(self.controller.pp.kp_servo))

    def draw(self, edge_points, cur_pos, trajectory, start_pos, past_path, obstacles, end_points, arrival_range):
        scale = 0.05
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
        self.scene.addLine(QLineF(c * cur_pos[0], -c * cur_pos[1],
                                      c * trajectory[0][0], -c * trajectory[0][1]), pen_trajectory)
        for i in range(len(trajectory) - 1):
            self.scene.addLine(QLineF(c * trajectory[i][0], -c * trajectory[i][1],
                                      c * trajectory[i + 1][0], -c * trajectory[i + 1][1]), pen_trajectory)

        pen_obstacle = QPen(Qt.black)
        for i in range(len(obstacles)):
            obstacle_diameter = c * 0.4
            self.scene.addEllipse(c * obstacles[i][0] - obstacle_diameter / 2,
                                  -c * obstacles[i][1] - obstacle_diameter / 2,
                                  obstacle_diameter, obstacle_diameter, pen_obstacle, QBrush(Qt.black))

        pen_boat = QPen(Qt.red)
        diameter = c * 0.4
        self.scene.addEllipse(c * cur_pos[0] - diameter / 2, -c * cur_pos[1] - diameter / 2,
                              diameter, diameter, pen_boat, QBrush(Qt.red))

        pen_start = QPen(Qt.green)
        diameter = c * 0.4
        self.scene.addEllipse(c * start_pos[0] - diameter / 2, -c * start_pos[1] - diameter / 2,
                              diameter, diameter, pen_start, QBrush(Qt.green))

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