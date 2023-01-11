#!/usr/bin/env python

import math
import numpy as np
import rospy
import pymap3d as pm

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle
from geometry_msgs.msg import Point
from tricat_211.msg import HeadingAngle, FilteringObstacles, FilteringWalls, DWA
from tricat_211.msg import FilteringWallsDWA, WallParticle, FilteringObstaclesDWA
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPVT

class Boat:
    def __init__(self):
        self.x = 0.0  # current x coordinate of boat
        self.y = 0.0  # current y coordinate of boat
        self.yaw = math.pi / 2  # [rad] current rotation(yaw) of boat initial yaw
        self.speed = 0.0  # [m/s] current speed of boat
        self.yaw_rate = 0.0  # [rad/s] 
        
        #self.bearing = 10.0

        rospy.Subscriber("/enu_position", Point, self.boat_xy_callback)
        rospy.Subscriber("/bearing", HeadingAngle, self.yaw_callback)
        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        rospy.Subscriber("/ublox_gps/navpvt", NavPVT, self.v_callback)

    def boat_xy_callback(self, msg):
        self.x = msg.x
        self.y = msg.y

    def yaw_callback(self, msg):
        if -90 < msg.bearing <= 180:
            self.yaw = (90 - msg.bearing) * math.pi / 180
        elif -90 >= msg.bearing >= -180:
            self.yaw = -(270 + msg.bearing) * math.pi / 180
        # degree N is 0, right is +, left is -, bearing is yaw najunge byungyung

    def yaw_rate_callback(self, msg):
        self.yaw_rate = msg.angular_velocity.z # rad/s

    def v_callback(self, msg):
        self.speed = float(msg.gSpeed / 1000.0) # m/s

    def boat_status(self):
        return np.array([self.x, self.y, self.yaw, self.speed, self.yaw_rate])


class Goal:
    def __init__(self):
        self.map_list = rospy.get_param("map_dd")

        self.origin = [self.map_list['map_00_lat'], self.map_list['map_00_lon'], self.map_list['map_00_alt']]
        
        self.waypoints = rospy.get_param("waypoint_List/waypoints")
        self.way_list_gps = np.empty((0,3), float)
        for i in range(len(self.waypoints)):
            self.way_list_gps = np.append(self.way_list_gps, np.array([self.waypoints[i]]), axis=0)
            
        self.way_list = np.zeros_like(self.way_list_gps)
        self.get_xy()

    def get_xy(self):
        for i in range(len(self.way_list_gps)):
            self.way_list[i][0], self.way_list[i][1] , self.way_list[i][2] = \
                pm.geodetic2enu(self.way_list_gps[i][0], self.way_list_gps[i][1], self.way_list_gps[i][2], \
                    self.origin[0], self.origin[1], self.origin[2])
        self.way_list = np.delete(self.way_list, 2, axis=1)

    def set_next_point(self):
        self.way_list = np.delete(self.way_list, 0, axis = 0)


class Ob:
    def __init__(self):
        self.ob_list = np.empty((0, 3), float) #  np.array([])
        self.w_list = np.empty((0, 3), float) # np.array([])
        self.obs_list = np.empty((0, 3), float)

        rospy.Subscriber('/filtering_obstacles', FilteringObstaclesDWA, self.filtering_obstacle_callback)
        rospy.Subscriber('/filtering_walls', FilteringWallsDWA, self.filtering_wall_callback)

    def filtering_obstacle_callback(self, msg):
        self.ob_list = np.empty((0, 3), float)
        f_obstacles = msg.circles
        for i in range(len(f_obstacles)):
            ob = np.array([[f_obstacles[i].x, f_obstacles[i].y, f_obstacles[i].theta]])
            self.ob_list = np.append(self.ob_list, ob, axis = 0)

    def filtering_wall_callback(self, msg):
        self.w_list = np.empty((0, 3), float)
        f_walls = msg.particle
        for i in range(len(f_walls)):
            w = np.array([[f_walls[i].x, f_walls[i].y, f_walls[i].theta]])
            self.w_list = np.append(self.w_list, w, axis = 0)
    
    def sum_ob_wall(self):
        self.obs_list = np.empty((0, 3), float)
        self.obs_list = np.append(self.ob_list, self.w_list, axis = 0)
        return self.obs_list

        
class DWA_Calc:
    def __init__(self):
        dwa_config = rospy.get_param("DWA")
        edge_points = rospy.get_param("edge_dd")

        self.max_speed = dwa_config['max_speed']  # [m/s]
        self.min_speed = dwa_config['min_speed']  # [m/s]
        self.max_yaw_rate = dwa_config['max_yaw_rate'] * math.pi / 180.0  # [rad/s]
        self.max_accel = dwa_config['max_accel']  # [m/ss]
        self.max_delta_yaw_rate = dwa_config['max_delta_yaw_rate'] * math.pi / 180.0  # [rad/ss]
        self.v_resolution = dwa_config['v_resolution']  # [m/s]
        self.yaw_rate_resolution = dwa_config['yaw_rate_resolution'] * math.pi / 180.0  # [rad/s]
        self.dt = dwa_config['dt']  # [s] Time tick for motion prediction
        self.predict_time = dwa_config['predict_time']  # [s]
        self.to_goal_cost_gain = dwa_config['to_goal_cost_gain']
        self.speed_cost_gain = dwa_config['speed_cost_gain']
        self.obstacle_cost_gain = dwa_config['obstacle_cost_gain']
        self.stuck_flag_cons = dwa_config['stuck_flag_cons']  # constant to prevent robot stucked
        self.safey_radius = dwa_config['safey_distance']

        self.boat_width = dwa_config['boat_width']  # [m] for collision check
        self.boat_length = dwa_config['boat_length']  # [m] for collision check
        self.goal_range = dwa_config['goal_range'] # [m]

        self.edge1 = [edge_points['map_00_lat'], edge_points['map_00_lon'], edge_points['alt']]
        self.edge2 = [edge_points['map_0y_lat'], edge_points['map_0y_lon'], edge_points['alt']]
        self.edge3 = [edge_points['map_x0_lat'], edge_points['map_x0_lon'], edge_points['alt']]
        self.edge4 = [edge_points['map_xy_lat'], edge_points['map_xy_lon'], edge_points['alt']]
        self.edge_points = [self.edge1, self.edge2, self.edge3, self.edge4]

        self.class_obstacle = Ob()
        self.obstacles = self.class_obstacle.sum_ob_wall()

        self.boat = Boat()

        self.class_goal = Goal()
        self.goal = [self.class_goal.way_list[0][0], self.class_goal.way_list[0][1]]

        self.best_u = []
        self.dwa_pub = rospy.Publisher("/best_U", DWA, queue_size=10)

    def obstacle_init(self):
        self.obstacles = self.class_obstacle.sum_ob_wall()
        for i in range(len(self.obstacles)):
            d = math.sqrt(self.obstacles[i][0]**2 + self.obstacles[i][1]**2)
            self.obstacles[i][0] = self.boat.x + d * math.cos(self.boat.yaw - self.obstacles[i][2]) #must check!!!!!!!!
            self.obstacles[i][1] = self.boat.y + d * math.sin(self.boat.yaw - self.obstacles[i][2]) #must check!!!!!!!!
        self.obstacles = self.obstacles[:, 0:2]

    def goal_update(self):
        self.class_goal.set_next_point()
        if len(self.class_goal.way_list) != 0:
            self.goal = [self.class_goal.way_list[0][0], self.class_goal.way_list[0][1]]


    def arrival_check(self):
        dist_to_goal = math.hypot(self.boat.x - self.goal[0], self.boat.y - self.goal[1])
        if dist_to_goal <= self.goal_range:
            return True
        else:
            return False

    def DWAPublisher(self):
        dwa = DWA()
        dwa.v = self.best_u[0]
        dwa.yaw_rate = self.best_u[1]
        self.dwa_pub.publish(dwa)
    
    def PausePublisher(self):
        dwa = DWA()
        dwa.v = 0.0
        dwa.yaw_rate = 0.0
        self.dwa_pub.publish(dwa)

    def calc_dynamic_window(self):
        Vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate]
        Vd = [self.boat.speed - self.max_accel * self.dt,
              self.boat.speed + self.max_accel * self.dt,
              self.boat.yaw_rate - self.max_delta_yaw_rate * self.dt,
              self.boat.yaw_rate + self.max_delta_yaw_rate * self.dt]
        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_control_and_trajectory(self, dw):
        boat_init = np.array(self.boat.boat_status())
        min_cost = float("inf")
        best_u = [0.0, 0.0]

        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for yaw_rate in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                trajectory = self.predict_trajectory(boat_init, v, yaw_rate)

                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3]) # max v - trajectory v

                if len(self.obstacles) != 0 : #or len(self.walls)
                    ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)
                    final_cost = to_goal_cost + speed_cost + ob_cost # 1>>>  2>  3>>
                else:
                    final_cost = to_goal_cost + speed_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, yaw_rate]
                    if abs(best_u[0]) < self.stuck_flag_cons \
                            and abs(self.boat.speed) < self.stuck_flag_cons:    
                        best_u =[self.max_accel, -self.max_delta_yaw_rate] #m/s max rad/ss
        
        return best_u

    def calc_obstacle_cost(self, trajectory):
        dx = np.array([])
        dy = np.array([])
        min_r = 100000

        for i in range(len(trajectory)):
            for j in range(len(self.obstacles)):
                dx = trajectory[i, 0] - self.obstacles[j, 0]
                dy = trajectory[i, 1] - self.obstacles[j, 1]
                r = np.hypot(dx, dy)
                if min_r >= r:
                    min_r = r

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        
        local_ob = self.obstacles[:, None] - trajectory[:, 0:2]

        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob.dot(x) for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])

        upper_check = local_ob[:, 0] <= 0.3  #set with param?????????????
        right_check = local_ob[:, 1] <= self.boat_width / 2
        bottom_check = local_ob[:, 0] >= -self.boat_length
        left_check = local_ob[:, 1] >= -self.boat_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

        return 1.0 / min_r

    def calc_to_goal_cost(self, trajectory):
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def dwa_control(self):
        dw = self.calc_dynamic_window()
        self.best_u = self.calc_control_and_trajectory(dw)

    def predict_trajectory(self, boat_init, v, yaw_rate):
        boat_pos = np.array(boat_init)
        trajectory = np.array(boat_pos)
        time = 0

        while time <= self.predict_time:
            temp_boat_pos = boat_pos
            temp_boat_pos[2] += yaw_rate * self.dt # yaw_rate yaw
            temp_boat_pos[0] += v * math.cos(temp_boat_pos[2]) * self.dt # v x
            temp_boat_pos[1] += v * math.sin(temp_boat_pos[2]) * self.dt # v y
            temp_boat_pos[3] = v  # v v
            temp_boat_pos[4] = yaw_rate  # yaw_rate yaw_rate

            if self.check_line_in([boat_pos[0], boat_pos[1]]): #go out
                trajectory = np.vstack((trajectory, temp_boat_pos))
                boat_pos = temp_boat_pos

            time += self.dt

        return trajectory

    def check_line_in(self, point):
        crossed = 0
        d_line_point = float("Inf")

        for i in range(4):
            j = (i+1)%4
            if(self.edge_points[i][1] > point[1]) != (self.edge_points[j][1] > point[1]):
                intersection = float((self.edge_points[j][0] - self.edge_points[i][0]) * (point[1]-self.edge_points[i][1]) \
                     / (self.edge_points[j][1] - self.edge_points[i][1]) + self.edge_points[i][0])
                if point[0] < intersection:
                    crossed += 1
                area = abs((self.edge_points[i][0] - point[0]) * (self.edge_points[j][1] - point[1]) \
                    - (self.edge_points[i][1] - point[1]) * (self.edge_points[j][0] - point[0]))
                p_to_p = math.sqrt((self.edge_points[i][0] - self.edge_point[j][0]) ** 2 + (self.edge_points[i][1] - self.edge_points[j][1]) ** 2 )
                d = area / p_to_p
                if d < d_line_point:
                    d_line_point = d

        if (crossed % 2) == 0 or d < self.safey_distance:
            return True #inside the line
        else :
            return False
    
    def prt(self):
        print("------------------dwa------------------")
        print("current goal : " + str(round(self.goal[0],3)) + " , " + str(round(self.goal[1], 3)) + " / x, y : " + str(round(self.boat.boat_status()[0],2)) + " , " + str(round(self.boat.boat_status()[1],2)))
        print("boat_status : " + str(round(self.boat.boat_status()[2]* 180 / math.pi,1)) + " deg , " + str(round(self.boat.boat_status()[3],2)) + "m/s , " + str(round(self.boat.boat_status()[4] * 180 / math.pi,2)) + " deg/s")
        print("Best U : " + str(round(self.best_u[0], 2)) + " m/s , " + str(round(self.best_u[1] * 180 / math.pi,2)) + " deg/s")

def shutdownhook():
    print("End the program") #no meaning


def main():
    rospy.init_node('DWA', anonymous=False)
    dwa_path = DWA_Calc()
    hz = 10
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        hz = hz -1

        dwa_path.obstacle_init() 

        if dwa_path.arrival_check():
            if len(dwa_path.class_goal.way_list) == 0:
                print("yeah~ last goal arrived")
                dwa_path.PausePublisher()
                rospy.on_shutdown(shutdownhook)
                break  #arrived final goal
            elif len(dwa_path.class_goal.way_list) == 1:
                print("waypoint arrived! go to the LAST GOAL!")
                dwa_path.goal_update()
            else:
                print("waypoint arrived! go to the next goal!")
                dwa_path.goal_update()
                

        dwa_path.obstacle_init()
        
        dwa_path.dwa_control()
        #print("calculation complete")
        dwa_path.DWAPublisher()
        
        if hz < 1:
            dwa_path.prt()
            hz = 10

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
