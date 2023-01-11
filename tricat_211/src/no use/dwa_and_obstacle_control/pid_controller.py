#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import Imu, NavSatFix
from ublox_msgs.msg import NavPVT
from tricat_211.msg import Control, DWA  # thruster, servo, v, yaw_rate float64
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

class PID:
    def __init__(self):
        PID_config = rospy.get_param("PID")
        self.thruster_pwm = 1520
        self.servo_control = 93

        self.cur_v = 0.0
        self.cur_yaw_rate = 0.0
    
        self.optimal_v = 0.0
        self.optimal_yaw_rate = 0.0

        self.kp_thruster = PID_config['kp_thruster']
        self.ki_thruster = PID_config['ki_thruster']
        self.kd_thruster = PID_config['kd_thruster']
        self.error_v = 0.0
        self.error_v_sum = 0.0
        self.prev_v = 0.0 # what is this

        self.kp_servo = PID_config['kp_servo']
        self.ki_servo = PID_config['ki_servo']
        self.kd_servo = PID_config['kd_servo']
        self.error_yaw_rate = 0.0
        self.error_yaw_rate_sum = 0.0
        self.prev_yaw_rate = 0.0 # what is this

        self.rate = PID_config['rate']   # ex) 0.05 => 1 / rate

        rospy.Subscriber("/imu/data", Imu, self.yaw_rate_callback)
        rospy.Subscriber("/ublox_gps/navpvt", NavPVT, self.velocity_callback)
        rospy.Subscriber("/best_U", DWA, self.DWA_data_callback)

        self.Servo_pub = rospy.Publisher("/Servo", UInt16, queue_size=10)
        self.thruster_pub = rospy.Publisher("/thruster", UInt16, queue_size=10)

    def yaw_rate_callback(self, data):
        self.cur_yaw_rate = data.angular_velocity.z  # yaw_rate [rad/s]

    def velocity_callback(self, data):
        self.cur_v = data.gSpeed * 0.001 # mm/s -> m/s  velocity

    def DWA_data_callback(self, data):
        self.optimal_v = data.v
        self.optimal_yaw_rate = data.yaw_rate

    def thruster_pid_controller(self):
        self.error_v = self.optimal_v - self.cur_v

        self.cp_thruster = self.kp_thruster * self.error_v

        self.ci_thruster = 0

        v_derivative = (self.cur_v - self.prev_v) / self.rate # division 1 / self.rate  # replaced with accel for drivative kick
        self.prev_v = self.cur_v
        self.cd_thruster = self.kd_thruster * -v_derivative

        thruster_pd = self.cp_thruster + self.ci_thruster + self.cd_thruster
        self.thruster_pwm = self.thruster_pwm + thruster_pd

        if self.thruster_pwm > 1600:
            self.thruster_pwm = 1600
        elif self.thruster_pwm < 1500:
            self.thruster_pwm = 1500
        else:
            pass

        return 1600 #self.thruster_pwm

    def servo_pid_controller(self):
        '''
        self.error_yaw_rate = self.optimal_yaw_rate - self.cur_yaw_rate

        self.cp_servo = self.kp_servo * self.error_yaw_rate

        #self.error_yaw_rate_sum = self.error_yaw_rate_sum + self.error_yaw_rate * (1 / self.rate)
        #if motor is off :
        #    self.error_yaw_rate_sum = 0
        #self.ci_servo =  self.kd_servo * self.error_yaw_rate_sum
 
        yaw_rate_derivative = (self.cur_yaw_rate - self.prev_yaw_rate) / self.rate # division 1 / self.rate
        self.prev_yaw_rate = self.cur_yaw_rate
        self.cd_servo = self.kd_servo * -yaw_rate_derivative
        self.ci_servo = 0
        
        servo_pd = -(self.cp_servo + self.ci_servo + self.cd_servo)
        self.servo_control = (self.servo_control + servo_pd)
        if self.servo_control > 118: # 93+25  right
            self.servo_control = 118
        elif self.servo_control < 68: # 93-25 left
            self.servo_control = 68
        else:
            pass
        '''
        x = self.optimal_yaw_rate
        y = -228.92 * x**3 - 205.16 * x**2 -84.808*x + 87.947
        self.servo_control = int(y)
        if self.servo_control > 93+20: # 93+25  right
            self.servo_control = 93+20
        elif self.servo_control < 93-20: # 93-25 left
            self.servo_control = 93-20
        else:
            pass
        return self.servo_control

    def control_publisher(self):
        output_msg = Control()
        output_msg.thruster = round(self.thruster_pid_controller())
        output_msg.servo = round(self.servo_pid_controller())
        self.thruster_pub.publish(output_msg.thruster)
        self.Servo_pub.publish(output_msg.servo)
    
    def prt(self):
        print("-------------------arduino-------------------")
        print("thruster:", self.thruster_pwm, "servo : ", self.servo_control)
        
        
        

def main():
    rospy.init_node('PID_controller', anonymous=False)
    pid = PID()
    hz = 10
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        hz = hz -1
        
        pid.control_publisher()

        if hz < 1:
            pid.prt()
            hz = 10

        rate.sleep()
    rospy.spin()
if __name__ == '__main__':
    main()
