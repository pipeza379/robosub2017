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

    def getState(self,data):
        self.pose=data.pose.pose
        pose = self.pose
        temp = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        euler_angular =tf.transformations.euler_from_quaternion(temp)
        
        self.auv_state[0] = pose.position.x
        self.auv_state[1] = pose.position.y
        self.auv_state[2] = pose.position.z
        self.auv_state[3] = euler_angular[0]
        self.auv_state[4] = euler_angular[1]
        self.auv_state[5] = euler_angular[2]

    def stop(self,time):
        self.drive.linear.x=0 
        self.drive.linear.y=0 
        self.drive.linear.z=0
        
        self.drive.angular.x=0 
        self.drive.angular.y=0 
        self.drive.angular.z=0
        
        # for i in xrange(5):
        self.pub_vel.publish(self.drive)
        rospy.sleep(time)    

    def turnYaw(self,rad):
        self.stop(2)
        self.pub_yaw.publish(Float64(math.radians(rad)))
        rospy.sleep(2)

    def turnAbsYaw(self,rad):
        self.stop(2)
        self.pub_abs_yaw.publish(Float64(math.radians(rad)))
        rospy.sleep(2)

    def depth(self,high):
        self.stop(2)
        self.pub_abs_depth.publish(high)
    
    def run(self,position,turn=False):
        self.stop(2)
        here = (self.auv_state[0],self.auv_state[1],self.auv_state[2])
        plus=1
        if turn == True:
            plus = -1
        for pose in range(len(position)):
            if position[pose] == 777: #not set position
                continue    
            while self.auv_state[pose] < position[pose]-(0.15*position[pose]):
                # print self.auv_state
                if pose == 0:
                    self.drive.linear.x+=plus
                elif pose == 1:                    
                    self.drive.linear.y+=plus
                elif pose == 2:
                    self.drive.linear.z+=plus
                self.pub_vel.publish(self.drive)
            while self.auv_state[pose] > position[pose]+(0.15*position[pose]):
                # print self.auv_state
                if pose == 0:
                    self.drive.linear.x-=plus
                elif pose == 1:
                    self.drive.linear.y-=plus
                elif pose == 2:
                    self.drive.linear.z-=plus
                self.pub_vel.publish(self.drive)
            print self.auv_state
    
    # def runTriangle(self,position):
    #     degree = math.degrees(math.atan(position[1]/position[0]))
    #     pose = self.auv_state[:3]
    #     # here = (pose[0],pose[1],pose[2])
    #     path = (position[0]**2+position[1]**2)**(1/2)
    #     rospy.Subscriber('/auv/state',Odometry,self.getState)
    #     if position[0] >= 0 :
    #         if position[1] < 0:
    #             degree = degree+90
    #     elif position[0] < 0:
    #         if positionp[1] >=0:
    #             degree += 270
    #         else:
    #             degree = 270 - degree 
    #     self.turnYaw(degree)
    #     self.stop(4)
    #     while math.sqrt((x-round(pose[0],2))**2+(y-round(pose[1]))**2)>1:
    #         if position[0]-pose[0]
        
    #     self.pub_vel.publish(self.drive)
 
    def missionGate(self,position):
        set_here = (self.auv_state[0],self.auv_state[1]) 
        print('sethere',set_here)
        self.turnAbsYaw(0)
        self.run(position)
        self.stop(2)
        print self.auv_state
        self.turnYaw(180)
        self.stop(5)
        position[0] = set_here[0];position[1] = set_here[1]
        print('pose',position)
        set_here = (self.auv_state[0],self.auv_state[1])
        print('set_here',set_here)
        self.run(position,True)
        self.stop(0.1)
        print self.auv_state

if __name__ == '__main__':
    # while not rospy.is_shutdown():  
        control=Zeabuscontrol()
        position = [int(input('x: ')),int(input('y: '))]#,int(input())]
        control.missionGate(position)
        print control.auv_state
    
    # control.run(position)
    # control.stop()
    # control.turnYaw(-90)
    # control.depth(0.5)