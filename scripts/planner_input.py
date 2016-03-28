#!/usr/bin/python
#added for ROS
import rospy
from ros_planner.msg import CoordinateMsgs
from std_msgs.msg import Int32,Int8

class Position_input():

    def __init__(self):
        self.pub = rospy.Publisher('modules_position', CoordinateMsgs, queue_size=10)
        rospy.init_node('HMI_Planner',anonymous=True)
        self.rate = rospy.Rate(10) #10hz
        self.msg = CoordinateMsgs()


    def setCoord(self, name):
        # Function asking user to enter position of each modules
        print('position for ', self.m_id)
        self.Fx = raw_input('Enter x coord: ')
        self.Fy = raw_input('Enter y coord: ')
        self.Fz = raw_input('Enter z coord: ')
        # Got some error with JSON file, make sure raw_input is an integer so
        # it can be concanated later.
        return (int(self.Fx),int(self.Fy),int(self.Fz))

    def getPosition(self):

        while not rospy.is_shutdown():
            self.go = int(raw_input(" Create a module (1) or Launch the protocol (0) "))
            if self.go:

                self.name = raw_input('Enter module name: ')
                self.type = raw_input('Enter module type: ')
                self.m_id = raw_input('Enter module ID: ')
                self.coord_x, self.coord_y, self.coord_z = p_i.setCoord(self.m_id)

                self.msg.run = 0
                self.msg.name = self.name
                self.msg.m_id = self.m_id
                self.msg.type = self.type
                self.msg.coord_x = self.coord_x
                self.msg.coord_y = self.coord_y
                self.msg.coord_z = self.coord_z

                rospy.loginfo(self.msg)
                self.pub.publish(self.msg)
                self.rate.sleep()

            else:
                self.msg.run = 1
                self.msg.name = "NA"
                self.msg.m_id = "NA"
                self.msg.type = "NA"
                self.msg.coord_x = 0
                self.msg.coord_y = 0
                self.msg.coord_z = 0

                rospy.loginfo(self.msg)
                self.pub.publish(self.msg)
                self.rate.sleep()


if __name__ == '__main__':
    try:
        p_i = Position_input()
        p_i.getPosition()
    except rospy.ROSInterruptException:
        pass
