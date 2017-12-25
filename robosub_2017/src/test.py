#!/usr/bin/python2
import rospy, math, tf 
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

class Zeabuscontrol:

    def __init__(self):
        rospy.init_node('autorun',anonymous=True)
        self.drive = Twist()
        self.pose = Pose()
        self.pub_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.pub_yaw = rospy.Publisher('/fix/rel/yaw',Float64,queue_size=10)
        self.pub_abs_yaw = rospy.Publisher('/fix/abs/yaw',Float64,queue_size=10)
        self.pub_abs_depth = rospy.Publisher('/fix/abs/depth',Float64,queue_size=10) 
        self.auv_state = [0,0,0,0,0,0] #position
        rospy.Subscriber('/auv/state',Odometry,self.getState) #recive position

if __name__ == '__main__':
    #position = [int(input('x: ')),int(input('y: '))]#,int(input())]
    control=Zeabuscontrol()
    # control.turn_abs_yaw(0)
    # control.run(position)
    # control.stop()
    # print control.auv_state
    
print(numpy())