#!/usr/bin/env python

# import numpy as np
# import rospy
# import math
# import sys
# from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle
# from tricat_211.msg import FilteringObstacles, CircleObstacle, CircleObstacleDWA, FilteringObstaclesDWA
# from tricat_211.msg import FilteringWalls, WallObstacle, FilteringWallsDWA, WallParticle


# class ObstacleFilter:
#     def __init__(self):
#         ob_config = rospy.get_param("OB")
#         self.decision_range = ob_config['decision_range']
#         self.min_wall_length = ob_config['min_wall_length']

#         self.obstacle_list = [] #get list from lidar
#         self.segment_list = []
#         self.filter_obstacle_list = np.empty((0,5), float)  # len(self.obstacle_list)//obstacle list - distance from boat(x, y, d), theta # (obstacle number)*4
#         # self.filter_wall_list = np.empty((0,5), float)
#         self.filter_wall_list_DWA = np.empty((0,2), float) #(x, y)

#         rospy.Subscriber('/obstacles', Obstacles, self.obstacle_callback)

#         self.filter_obstacles_pub = rospy.Publisher("/filtering_obstacles", FilteringObstaclesDWA, queue_size=10)
#         # self.filter_walls_pub = rospy.Publisher("/filtering_walls", FilteringWalls, queue_size=10)
#         self.filter_walls_pub = rospy.Publisher("/filtering_walls", FilteringWallsDWA, queue_size=10)

#     def obstacle_callback(self, msg):
#         self.obstacle_list = msg.circles #.center.x/.center.y/.center.z/.radius
#         self.segment_list = msg.segments #.first_point.x/.first_point.y/.first_point.z/last_point

#     # def obstacle_filtering(self):
#     #     obstacles = self.obstacle_list
#     #     self.filter_obstacle_list = np.empty((0,5), float)

#     #     if len(self.obstacle_list) == 0:
#     #         return

#     #     for i in range(len(self.obstacle_list)):
#     #         ob_x = -obstacles[i].center.x
#     #         ob_y = obstacles[i].center.y
#     #         ob_d = math.sqrt(ob_x ** 2 + ob_y ** 2)
#     #         ob_R = obstacles[i].radius
#     #         ob_theta = math.atan2(ob_y,ob_x) * 180 / math.pi
#     #         if self.distance_decision(ob_d):
#     #             ob_info = np.array([[ob_x, ob_y, ob_d, ob_theta, ob_R]])
#     #             self.filter_obstacle_list = np.append(self.filter_obstacle_list, ob_info, axis=0)
#     #         else:
#     #             continue

#     #     return self.filter_obstacle_list

#     def obstacle_filtering(self):
#         obstacles = self.obstacle_list
#         self.filter_obstacle_list = np.empty((0,2), float)

#         if len(self.obstacle_list) == 0:
#             return

#         for i in range(len(self.obstacle_list)):
#             ob_x = -obstacles[i].center.x
#             ob_y = obstacles[i].center.y
#             ob_d = math.sqrt(ob_x ** 2 + ob_y ** 2)
#             #ob_R = obstacles[i].radius
#             #ob_theta = math.atan2(ob_y,ob_x) * 180 / math.pi
#             if self.distance_decision(ob_d):
#                 ob_info = np.array([[ob_x, ob_y]]) #, ob_d, ob_theta, ob_R]])
#                 self.filter_obstacle_list = np.append(self.filter_obstacle_list, ob_info, axis=0)
#             else:
#                 continue

#         return self.filter_obstacle_list

#     # def wall_filtering(self):
#     #     walls = self.segment_list
#     #     self.filter_wall_list = np.empty((0,5), float)

#     #     if len(self.segment_list) == 0:
#     #         return

#     #     for i in range(len(self.segment_list)): # 12?
#     #         w_start_x = walls[i].first_point.x
#     #         w_start_y = walls[i].first_point.y
#     #         w_end_x = walls[i].last_point.x
#     #         w_end_y = walls[i].last_point.y
#     #         w_distance = self.distance_boat_to_wall(w_start_x, w_start_y, w_end_x, w_end_y)
#     #         w_length = (w_end_x - w_start_x) ** 2 + (w_end_y - w_start_y) ** 2
#     #         if self.distance_decision(w_distance) and abs(w_length) > self.min_wall_length:
#     #             w_info = np.array([[w_start_x, w_start_y, w_end_x, w_end_y, w_distance]])
#     #             self.filter_wall_list = np.append(self.filter_wall_list, w_info, axis = 0)
                
#     #         else:
#     #             continue            

#     #     return self.filter_wall_list

#     def wall_filtering(self):
#         walls = self.segment_list
#         self.filter_wall_list_DWA = np.empty((0, 2), float)

#         if len(self.segment_list) == 0:
#             return

#         for i in range(len(self.segment_list)):
#             start_x = walls[i].first_point.x
#             start_y = walls[i].first_point.y
#             end_x = walls[i].last_point.x
#             end_y = walls[i].last_point.y
#             distance = self.distance_boat_to_wall(start_x, start_y, end_x, end_y)
#             length = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)

#             if self.distance_decision(distance) and abs(length) > self.min_wall_length:
#                 particles = np.zeros((0, 2), float)
#                 div_num = int(length/2 - 1)
#                 for i in range(div_num):
#                     mid_point = np.array([[start_x + (end_x-start_x)*(i/div_num), start_y + (end_y-start_y)*(i/div_num)]])
#                     self.filter_wall_list_DWA = np.append(self.filter_wall_list_DWA, mid_point, axis = 0)                
#                 w_info = np.array([[end_x, end_y]])
#                 self.filter_wall_list_DWA = np.append(self.filter_wall_list_DWA, w_info, axis = 0)
#             else:
#                 continue
    
#     def filteringObstaclesPublisher(self):
#         filtering_ob_list = FilteringObstaclesDWA()
        
#         for i in range(len(self.filter_obstacle_list)):
#             filtering_ob_list.circles.append(CircleObstacle(x = self.filter_obstacle_list[i][0],
#             y = self.filter_obstacle_list[i][1]))
#             #d = self.filter_obstacle_list[i][2],
#             #theta = self.filter_obstacle_list[i][3],
#             #radius = self.filter_obstacle_list[i][4]))
            
#         self.filter_obstacles_pub.publish(filtering_ob_list)
    
#     # def filteringWallsPublisher(self):
#     #     filtering_w_list = FilteringWalls()

#     #     for i in range(len(self.filter_wall_list)):
#     #         filtering_w_list.walls.append(WallObstacle(start_x = self.filter_wall_list[i][0], 
#     #         start_y = self.filter_wall_list[i][1], 
#     #         end_x = self.filter_wall_list[i][2],
#     #         end_y = self.filter_wall_list[i][3],
#     #         distance = self.filter_wall_list[i][4]))

#     #     self.filter_walls_pub.publish(filtering_w_list)

#     def filteringWallsPublisherForDWA(self):
#         filtering_w_list = FilteringWallsDWA()

#         for i in range(len(self.filter_wall_list_DWA)):
#             filtering_w_list.particle.append(WallParticle(x = self.filter_wall_list_DWA[i][0], 
#             y = self.filter_wall_list_DWA[i][1]))

#         self.filter_walls_pub.publish(filtering_w_list)
    
#     def distance_decision(self, distance):
#         if distance < self.decision_range :
#             return True
#         else:
#             return False

#     def distance_boat_to_wall(self, start_x, start_y, end_x, end_y):
#         boat_to_start = start_x**2 + start_y**2 #square value
#         boat_to_end = end_x**2 + end_y**2 # square value
#         start_to_end = (start_x-end_x)**2 + (start_y-end_y)**2 #square value
#         cosin_value = (boat_to_start + start_to_end - boat_to_end)/(2*math.sqrt(boat_to_start)*math.sqrt(start_to_end))
#         angle_value = math.acos(cosin_value)
#         boat_to_line = abs(math.sqrt(boat_to_start)*math.sin(angle_value))
#         return boat_to_line


# def main():
#     rospy.init_node('Obstacle_Filter', anonymous=False)

#     f = ObstacleFilter()

#     #rospy.sleep(1)
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         f.obstacle_filtering()
#         f.wall_filtering()

#         f.filteringObstaclesPublisher()
#         # f.filteringWallsPublisher()
#         f.filteringWallsPublisherForDWA()

#         rate.sleep()

#         #rospy.sleep(0.5) # 1 / Hz
#     rospy.spin()

# if __name__ == '__main__':
#     main()


import numpy as np
import rospy
import math
import sys
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle
from tricat_211.msg import FilteringObstacles, CircleObstacle, CircleObstacleDWA, FilteringObstaclesDWA
from tricat_211.msg import FilteringWalls, WallObstacle, FilteringWallsDWA, WallParticle


class ObstacleFilter:
    def __init__(self):
        ob_config = rospy.get_param("OB")
        self.decision_range = ob_config['decision_range']
        self.min_wall_length = ob_config['min_wall_length']

        self.obstacle_list = [] #get list from lidar
        self.segment_list = []
        self.filter_obstacle_list = np.empty((0,3), float)  # len(self.obstacle_list)//obstacle list - distance from boat(x, y, d), theta # (obstacle number)*4
        # self.filter_wall_list = np.empty((0,5), float)
        self.filter_wall_list_DWA = np.empty((0,3), float) #(x, y)

        rospy.Subscriber('/obstacles', Obstacles, self.obstacle_callback)

        self.filter_obstacles_pub = rospy.Publisher("/filtering_obstacles", FilteringObstaclesDWA, queue_size=10)
        # self.filter_walls_pub = rospy.Publisher("/filtering_walls", FilteringWalls, queue_size=10)
        self.filter_walls_pub = rospy.Publisher("/filtering_walls", FilteringWallsDWA, queue_size=10)

    def obstacle_callback(self, msg):
        self.obstacle_list = msg.circles #.center.x/.center.y/.center.z/.radius
        self.segment_list = msg.segments #.first_point.x/.first_point.y/.first_point.z/last_point

    '''def obstacle_filtering(self):
        obstacles = self.obstacle_list
        self.filter_obstacle_list = np.empty((0,5), float)

        if len(self.obstacle_list) == 0:
            return

        for i in range(len(self.obstacle_list)):
            ob_x = -obstacles[i].center.x
            ob_y = obstacles[i].center.y
            ob_d = math.sqrt(ob_x ** 2 + ob_y ** 2)
            ob_R = obstacles[i].radius
            ob_theta = math.atan2(ob_y,ob_x) * 180 / math.pi
            if self.distance_decision(ob_d):
                ob_info = np.array([[ob_x, ob_y, ob_d, ob_theta, ob_R]])
                self.filter_obstacle_list = np.append(self.filter_obstacle_list, ob_info, axis=0)
            else:
                continue

        return self.filter_obstacle_list'''

    def obstacle_filtering(self):
        obstacles = self.obstacle_list
        self.filter_obstacle_list = np.empty((0,3), float)

        if len(self.obstacle_list) == 0:
            return

        for i in range(len(self.obstacle_list)):
            ob_x = -obstacles[i].center.x
            ob_y = obstacles[i].center.y
            ob_d = math.sqrt(ob_x ** 2 + ob_y ** 2)
            #ob_R = obstacles[i].radius
            ob_theta = math.atan2(ob_y,ob_x) * math.pi / 180
            if self.distance_decision(ob_d):
                ob_info = np.array([[ob_x, ob_y, ob_theta]]) #, ob_d, ob_theta, ob_R]])
                self.filter_obstacle_list = np.append(self.filter_obstacle_list, ob_info, axis=0)
            else:
                continue

        return self.filter_obstacle_list

    # def wall_filtering(self):
    #     walls = self.segment_list
    #     self.filter_wall_list = np.empty((0,5), float)

    #     if len(self.segment_list) == 0:
    #         return

    #     for i in range(len(self.segment_list)): # 12?
    #         w_start_x = walls[i].first_point.x
    #         w_start_y = walls[i].first_point.y
    #         w_end_x = walls[i].last_point.x
    #         w_end_y = walls[i].last_point.y
    #         w_distance = self.distance_boat_to_wall(w_start_x, w_start_y, w_end_x, w_end_y)
    #         w_length = (w_end_x - w_start_x) ** 2 + (w_end_y - w_start_y) ** 2
    #         if self.distance_decision(w_distance) and abs(w_length) > self.min_wall_length:
    #             w_info = np.array([[w_start_x, w_start_y, w_end_x, w_end_y, w_distance]])
    #             self.filter_wall_list = np.append(self.filter_wall_list, w_info, axis = 0)
                
    #         else:
    #             continue            

    #     return self.filter_wall_list

    def wall_filtering(self):
        walls = self.segment_list
        self.filter_wall_list_DWA = np.empty((0, 3), float)

        if len(self.segment_list) == 0:
            return

        for i in range(len(self.segment_list)):
            start_x = walls[i].first_point.x
            start_y = walls[i].first_point.y
            end_x = walls[i].last_point.x
            end_y = walls[i].last_point.y
            distance = self.distance_boat_to_wall(start_x, start_y, end_x, end_y)
            length = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)

            if self.distance_decision(distance) and abs(length) > self.min_wall_length:
                particles = np.zeros((0, 2), float)
                div_num = int(length/2 - 1)
                for i in range(div_num):
                    mid_x = start_x + (end_x-start_x)*(i/div_num)
                    mid_y = start_y + (end_y-start_y)*(i/div_num)
                    w_theta = math.atan2(mid_y,mid_x) * math.pi / 180
                    mid_point = np.array([[mid_x, mid_y, w_theta]])
                    self.filter_wall_list_DWA = np.append(self.filter_wall_list_DWA, mid_point, axis = 0)
                                
                w_info = np.array([[end_x, end_y, (math.atan2(end_y, end_x) * math.pi / 180)]])
                self.filter_wall_list_DWA = np.append(self.filter_wall_list_DWA, w_info, axis = 0)
            else:
                continue
    
    def filteringObstaclesPublisher(self):
        filtering_ob_list = FilteringObstaclesDWA()
        
        for i in range(len(self.filter_obstacle_list)):
            filtering_ob_list.circles.append(CircleObstacle(x = self.filter_obstacle_list[i][0],
            y = self.filter_obstacle_list[i][1],
            theta = self.filter_obstacle_list[i][2]))
            #d = self.filter_obstacle_list[i][2],
            #theta = self.filter_obstacle_list[i][3],
            #radius = self.filter_obstacle_list[i][4]))
            
        self.filter_obstacles_pub.publish(filtering_ob_list)
    
    # def filteringWallsPublisher(self):
    #     filtering_w_list = FilteringWalls()

    #     for i in range(len(self.filter_wall_list)):
    #         filtering_w_list.walls.append(WallObstacle(start_x = self.filter_wall_list[i][0], 
    #         start_y = self.filter_wall_list[i][1], 
    #         end_x = self.filter_wall_list[i][2],
    #         end_y = self.filter_wall_list[i][3],
    #         distance = self.filter_wall_list[i][4]))

    #     self.filter_walls_pub.publish(filtering_w_list)

    def filteringWallsPublisherForDWA(self):
        filtering_w_list = FilteringWallsDWA()

        for i in range(len(self.filter_wall_list_DWA)):
            filtering_w_list.particle.append(WallParticle(x = self.filter_wall_list_DWA[i][0], 
            y = self.filter_wall_list_DWA[i][1],
            theta = self.filter_wall_list_DWA[i][2]))

        self.filter_walls_pub.publish(filtering_w_list)
    
    def distance_decision(self, distance):
        if distance < self.decision_range :
            return True
        else:
            return False

    def distance_boat_to_wall(self, start_x, start_y, end_x, end_y):
        boat_to_start = start_x**2 + start_y**2 #square value
        boat_to_end = end_x**2 + end_y**2 # square value
        start_to_end = (start_x-end_x)**2 + (start_y-end_y)**2 #square value
        cosin_value = (boat_to_start + start_to_end - boat_to_end)/(2*math.sqrt(boat_to_start)*math.sqrt(start_to_end))
        angle_value = math.acos(cosin_value)
        boat_to_line = abs(math.sqrt(boat_to_start)*math.sin(angle_value))
        return boat_to_line


def main():
    rospy.init_node('Obstacle_Filter', anonymous=False)

    f = ObstacleFilter()

    #rospy.sleep(1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        f.obstacle_filtering()
        f.wall_filtering()

        f.filteringObstaclesPublisher()
        # f.filteringWallsPublisher()
        f.filteringWallsPublisherForDWA()

        rate.sleep()

        #rospy.sleep(0.5) # 1 / Hz
    rospy.spin()

if __name__ == '__main__':
    main()

