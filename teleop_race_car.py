#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class TeleopCar:
    def __init__(self):
        rospy.init_node('teleop_car')
        self.pub = rospy.Publisher('~/cmd/cmd_vel', Twist, queue_size=1)
        self.move_bindings = {
            'w': (1, 0, 0, 1),
            'a': (1, 0, 0, -1),
            's': (-1, 0, 0, -1),
            'd': (-1, 0, 0, 1),
        }
        self.speed = rospy.get_param('~speed', 0.5)
        self.turn = rospy.get_param('~turn', 1.0)
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.key = ''
        rospy.loginfo("Ready to move")

    def getKey(self):
        self.key = raw_input()
        if self.key in self.move_bindings.keys():
            self.x = self.move_bindings[self.key][0]
            self.y = self.move_bindings[self.key][1]
            self.z = self.move_bindings[self.key][2]
            self.th = self.move_bindings[self.key][3]
        else:
            self.x = 0
            self.y = 0
            self.z = 0
            self.th = 0
            if self.key == '\x03':
                rospy.signal_shutdown("KeyboardInterrupt")

    def move(self):
        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.th * self.turn
        self.pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.getKey()
            self.move()
            rate.sleep()

if __name__ == '__main__':
    try:
        teleop_car = TeleopCar()
        teleop_car.run()
    except rospy.ROSInterruptException:
        pass
