#!/usr/bin/python
#added for ROS
import rospy
from ros_planner.msg import CoordinateMsgs # TODO : Change coords to float
from std_msgs.msg import String
from biobot_ros_msgs.msg import CoordinateMsgs

class HMI_Planner():

    def __init__(self):
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name ,anonymous=True)
        self.rate = rospy.Rate(10)  #10 Hz

        self.pub = rospy.Publisher('Deck_Item', CoordinateMsgs, queue_size=10)
        self.pub_start_protcol = rospy.Publisher('Start_Protocol', String, queue_size=10)
        self.msg = CoordinateMsgs()
        self.deck = []
        self.deck_import_file = '/home/linux/catkin_ws/src/ros_planner/deck/test.txt'
        self.deck_file = '/home/linux/catkin_ws/src/ros_planner/deck/deck.txt'
        self.default_protocol = '/home/linux/catkin_ws/src/ros_planner/json/pipette.json'

    def setCoord(self, name):
        # Function asking user to enter position of each modules
        print('position for ', name)
        Fx = float(raw_input('Enter x coord: '))
        Fy = float(raw_input('Enter y coord: '))
        Fz = float(raw_input('Enter z coord: '))
        # Got some error with JSON file, make sure raw_input is float so
        # it can be concanated later.
        return (Fx, Fy, Fz)

    def getPosition(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            print("Current items on the deck: \n{0}\n".format('\n'.join(self.deck)))

            go = raw_input("Load deck from file (0),\nAdd module (1),\nClear (2),\nSend deck to Planner (3),\nStart protocol (4) or\nExit HMI (5)\n")

            if go == '0':
                with open(self.deck_import_file, 'r') as f:
                    x = f.readlines()

                with open(self.deck_file, 'w') as f:
                    f.writelines(x)

                x = [i.rstrip() for i in x]

                nb_mod = len(x) / 6
                self.deck = []
                for i in range(nb_mod):
                    self.deck.append(x[i*6+0])

            elif go == '1':
                m_name = raw_input('Enter module name: ')
                m_type = raw_input('Enter module type: ')
                m_id = raw_input('Enter module ID: ')
                coord_x, coord_y, coord_z = p_i.setCoord(m_id)
                self.deck.append(m_name)
                with open(self.deck_file, 'a') as f:
                    write_module = ["{0}\n".format(m_name),
                                    "{0}\n".format(m_type),
                                    "{0}\n".format(m_id),
                                    "{0}\n".format(coord_x),
                                    "{0}\n".format(coord_y),
                                    "{0}\n".format(coord_z)]
                    f.writelines(write_module)

            elif go == '2':
                self.deck = []
                with open (self.deck_file, 'w'):
                    pass

            elif go == '3':

                if raw_input('Coordinates measured with pipette_s ? (1=yes/0=no)'):
                    with open(self.deck_file, 'r') as f:
                        x = f.readlines()
                    x = [i.rstrip() for i in x]

                    verify = len(x) % 6
                    if verify:
                        print("Error - deck file has incorrect length")
                        return

                    nb_mod = len(x) / 6
                    for i in range(nb_mod):
                        self.msg.m_name = x[i*6+0]
                        self.msg.m_type = x[i*6+1]
                        self.msg.m_id = x[i*6+2]
                        self.msg.coord_x = float(x[i*6+3])+80
                        self.msg.coord_y = float(x[i*6+4])+275
                        self.msg.coord_z = float(x[i*6+5])+120
                        print self.msg.coord_x
                        print self.msg.coord_y
                        print self.msg.coord_z
                        self.pub.publish(self.msg)
                        self.rate.sleep()

                else:

                    with open(self.deck_file, 'r') as f:
                        x = f.readlines()
                    x = [i.rstrip() for i in x]

                    verify = len(x) % 6
                    if verify:
                        print("Error - deck file has incorrect length")
                        return

                    nb_mod = len(x) / 6
                    for i in range(nb_mod):
                        self.msg.m_name = x[i*6+0]
                        self.msg.m_type = x[i*6+1]
                        self.msg.m_id = x[i*6+2]
                        self.msg.coord_x = float(x[i*6+3])
                        self.msg.coord_y = float(x[i*6+4])
                        self.msg.coord_z = float(x[i*6+5])
                        self.pub.publish(self.msg)
                        self.rate.sleep()


            elif go == '4':
                protocol_path = raw_input('Enter protocol path (0 for default): ')
                if protocol_path == '0':
                    protocol_path = self.default_protocol
                self.pub_start_protcol.publish(protocol_path)
                self.rate.sleep()


            else:
                print('Exiting HMI')
                return


if __name__ == '__main__':
    try:
        p_i = HMI_Planner()
        p_i.getPosition()

    except (rospy.ROSInterruptException, EOFError) as e:
        print(e)
