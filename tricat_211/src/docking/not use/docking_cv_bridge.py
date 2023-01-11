#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
import time
import numpy as np
import pymap3d as pm
from cv_bridge import CvBridge, CvBridgeError 

from tricat_211.msg import HeadingAngle, Control
from std_msgs.msg import UInt16
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox, ObjectCount 
    #if error, 'darknet_ros_msg.msg'
from sensor_msgs.msg import Image, LaserScan

#<BoundingBox> -- axis check!!!!!
#[0]float64 probability. [1]int64 xmin. [2]int64 ymin. [3]int64 xmax. 
#[4]int64 ymax. [5]int16 id. [6]string Class

#<Image>
# color/image_raw (sensor_msgs/Image) -->     Color rectified image. RGB format. 
#[0]std_msgs/Header header  [1]uint32 height        [2]uint32 width
#[3]string encoding         [4]uint8 is_bigendian   [5]uint32 step
#[6]uint8[] data


class Docking:
    def __init__(self):
        ### ROS Pub
        self.servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

        ### Hyper-Parameters
        docking_config = rospy.get_param("docking_param")
        self.target_class = docking_config['target_class']

        ### Camera(mark)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_callback)
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.convert_depth_image)
        
        self.classes = []
        self.img_width = 0
        self.img_height = 0
        self.box_center_x = 0
        self.box_center_y = 0

        self.box_depth = 0

        self.extra_span = docking_config['extra_span'] # 0~1
        self.close_distance = docking_config['close_distance'] #5m default -- too far??
        docking_zone_gps = docking_config['docking_zone_gps']
        lat_00, lon_00, alt_00 = docking_config['map_00_lat'], docking_config['map_00_lon'], docking_config['map_00_alt']
        #convert docking_zone_gps -> coordinate (x, y)
        self.docking_zone_xy = pm.geodetic2enu(docking_zone_gps[0], docking_zone_gps[1], docking_zone_gps[2], lat_00, lon_00, alt_00)[0:2]

        self.mark_state = 0 
            #0:None, 1:mark exist&not target, 2:target far, 3:target close
        self.target_loc = "None"
        
        ### LiDAR(obstacle)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.ranges = []
        self.ob_state = 0
            #0:None or far, 1:ob close

        ### Control
        self.servo = 94
        self.thruster = 1600

    def boxes_callback(self, msg):
        boxes = msg.bounding_boxes
        if len(boxes)==0:
            self.mark_state = 0
        else:
            self.classes = []
            for i in range(len(boxes)):
                self.classes.append(boxes[i].Class)
                
                if boxes[i].Class == self.target_class:
                    self.box_size = (boxes[i].xmax-boxes[i].xmin) * (boxes[i].ymax-boxes[i].ymin)
                    self.box_center_x = float((boxes[i].xmax+boxes[i].xmin)/2)
                    self.box_center_y = float((boxes[i].ymax+boxes[i].ymin)/2)

                    self.target_distance = self.box_depth * 0.001 #mm -> m
                    if self.target_distance <= self.close_distance:
                        self.mark_state = 3
                    else:
                        self.mark_state = 2
                    self.target_center_x = abs(boxes[i].xmax + boxes[i].xmin)/2
                elif i == len(boxes)-1 and self.mark_state>1:
                    self.mark_state = 1

    def image_callback(self, msg):
        self.img_width = msg.width
        self.img_height = msg.height

    def lidar_callback(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.ranges = msg.ranges
        if len(self.ranges)!=0 and min(self.ranges) <= self.close_distance:
            # close_obs = np.sort(self.ranges) #[0:5] #check if error erupted when array length is 0 or under 5
            idx = self.ranges.index(min(self.ranges))
            self.ob_angle = math.degrees(self.angle_min + self.angle_increment * self.idx)
            triangle_area = abs((min(self.ranges)**2) * math.tan(self.ob_angle))
            if triangle_area <= 35.8: #angle=20, d=2
                self.ob_state = 1
            else:
                self.ob_state = 0
        else:
            self.ob_state = 0

    def convert_depth_image(self, ros_image):
        bridge = CvBridge() # Use cv_bridge() to convert the ROS image to OpenCV format
        if self.mark_state==0 or self.mark_state==1:
            pass
        else:
            depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
                #Convert the depth image using the default passthrough encoding
            depth_array = np.array(depth_image, dtype=np.float32)
                #Convert the depth image to a Numpy array
            # idx_x = int(self.map_func(self.box_center_x, 0, self.img_width, 0, depth_array.shape[0]))
            # idx_y = int(self.map_func(self.box_center_y, 0, self.img_height, 0, depth_array.shape[1]))
            idx_x = int(self.box_center_x)
            idx_y = int(self.box_center_y)
            self.box_depth = depth_array[idx_x, idx_y]
            print("-------------------size", depth_array.shape)
            print("-------------------box_center_x", idx_x)
            print("-------------------box_center_y", idx_y)
            print("-------------------box_depth", self.box_depth)
        
    def state_judge(self):
        if self.ob_state==0:
            if self.mark_state==0:
                self.control_publish(thruster=1700)
            elif self.mark_state==1:
                self.control_publish(thruster=1400)
            else:
                self.target_tracking()
        else:
            self.obstacle_avoid()

    def target_tracking(self):
        line1 = self.img_width/2 - self.img_width*(1/6 + self.extra_span)
        line2 = self.img_width/2 - self.img_width*(1/6 + self.extra_span)
        servo_line1 = 94 - (8 + 48*self.extra_span)
        servo_line2 = 94 + (8 + 48*self.extra_span)
        if self.target_center_x < line1:
            angle_servo = self.map_func(self.target_center_x, 0, line1, 70, servo_line1)
            self.target_loc = "Left"
        elif self.target_center_x > line2:
            angle_servo = self.map_func(self.target_center_x, line2, 1280, 94, servo_line2)
            self.target_loc = "Right"
        else:
            angle_servo = self.map_func(self.target_center_x, line1, line2, servo_line1, servo_line2)
            self.target_loc = "Center"
        self.control_publish(servo=angle_servo)

    def obstacle_avoid(self):
        angle_servo = map_func(self.ob_angle, self.angle_min, self.angle_max, 70, 118)
            #check if it works well
        self.control_publish(servo=angle_servo)

    def map_func(self, x, input_min, input_max, output_min, output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min

    def control_publish(self, servo=94, thruster=1600):
        self.servo = servo
        self.thruster = thruster
        control = Control()
        control.thruster = thruster
        control.servo = round(servo) #PID control add?
        self.thruster_pub.publish(control.thruster)
        self.servo_pub.publish(control.servo)

    def dock_finished(self):
        return False
        # if self.mark_state == 0:
        #     return False
        # else:
        #     return (self.target_distance <= 1) # finished-> True
    
    def reset_state(self):
        pass
    
    def board_print(self):
        print("{0:*<27}".format("State"))
        print("# State")
        print("{0:^5} | {1:^13} | {2:^5}".format("", "target", ""))
        print("{0:-^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("", "cls", "far", "not", "none"))
        print("{0:-^5}-+-{1:-^13}-+-{2:-^5}".format("", "", ""))

        if self.mark_state==0:
            if self.ob_state==0:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("ob", "", "", "", ""))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format("none", "", "", "", "O"))
            else:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", "O"))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
        elif self.mark_state==1:
            if self.ob_state==0:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "O", ""))
            else:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "O", ""))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
        elif self.mark_state==2:
            if self.ob_state==0:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "O", "", ""))
            else:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "O", "", ""))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))
        elif self.mark_state==3:
            if self.ob_state==0:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "", "", "", ""))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "O", "", "", ""))
            else:
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" ob", "O", "", "", ""))
                print("{0:^5} | {1:^3} | {2:^3} | {3:^3} | {4:^5}".format(" none", "", "", "", ""))        

        if self.mark_state > 1:
            print("# {0:10} : {1}".format("tgt dist", round(self.target_distance, 2)))
            print("# {0:10} : {1}".format("tgt loc", self.target_loc))
        elif self.mark_state == 1:
            print("# {0:10} : {1}".format("watching", self.classes))
        if self.ob_state==1:
            print("# {0:10} : {1}".format("ob dist", round(min(self.ranges), 2)))
        
        print("# {0:10} : {1}".format("servo", self.servo))
        print("# {0:10} : {1}".format("thruster", self.thruster))


def main():   
    rospy.init_node('DockingMission', anonymous=False)
    rate = rospy.Rate(10)
    docking = Docking()
    print("{:=^40}".format(" Start "))

    # tick = 0
    docking.board_print()
    
    docking.state_judge()
    docking.reset_state()

    while not rospy.is_shutdown():
        if docking.dock_finished():
            print("{:=^40}".format(" Finished "))
            break
        else:
            print("{:=^40}".format(" Not Finished "))
            # if tick>=10:
                # docking.board_print()
                # tick=0
            docking.board_print()
            docking.state_judge()
        # tick+=1
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()




