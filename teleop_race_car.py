#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class TeleopRaceCar:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd/cmd_vel', Twist, queue_size=1)
        self.speed = 0.0
        self.turn = 0.0

    def move(self):
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.turn
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.init_node('teleop_race_car')
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            self.move()
            rate.sleep()

if __name__ == '__main__':
    teleop_race_car = TeleopRaceCar()
    try:
        while not rospy.is_shutdown():
            # Read key inputs
            key = input()
            if key == 'w':
                teleop_race_car.speed = 0.5
            elif key == 's':
                teleop_race_car.speed = -0.5
            elif key == 'a':
                teleop_race_car.turn = 0.5
            elif key == 'd':
                teleop_race_car.turn = -0.5
            else:
                teleop_race_car.speed = 0.0
                teleop_race_car.turn = 0.0
            # Move the race car
            teleop_race_car.move()
    except rospy.ROSInterruptException:
        pass
