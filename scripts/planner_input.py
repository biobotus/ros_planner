#!/usr/bin/python
#added for ROS
import rospy
import CoordinateMsgs.msg
from std_msgs.msg import integer
def setCoord(name):
    # Function asking user to enter position of each modules
    print('position for ', name)
    Fx = raw_input('Enter x coord dumbass: ')
    Fy = raw_input('Enter y coord dumbass: ')
    Fz = raw_input('Enter z coord dumbass: ')
    # Got some error with JSON file, make sure raw_input is an integer so
    # it can be concanated later.
    return (int(Fx),int(Fy),int(Fz))

def getPosition():
    coord_x, coord_y, coord_z = setCoord(name)
    pub = rospy.Publisher('chatter', CoordinateMsgs)
    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(10) #10hz
    msg = CoordinateMsgs()
    msg.name = name
    msg.coord_x = coord_x
    msg.coord_y = coord_y
    msg.coord_z = coord_z

    while not rospy.is_shutdownd():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
