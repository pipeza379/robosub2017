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
        rospy.sleep(time)    
        self.drive.linear.x=0 
        self.drive.linear.y=0 
        self.drive.linear.z=0
        
        self.drive.angular.x=0 
        self.drive.angular.y=0 
        self.drive.angular.z=0
        
        for i in xrange(3):
            self.pub_vel.publish(self.drive)
        
    def auvMove(self,move,speed=1):
        if move == 'forward':
            self.drive.linear.x+=speed
        elif move == 'backward':
            self.drive.linear.x-=speed
        elif move == 'up':
            self.drive.linear.z+=speed
        elif move == 'down':
            self.drive.linear.z-=speed
        elif move == 'left':
            self.drive.linear.y+=speed
        elif move == 'right':
            self.drive.linear.y-=speed
        for i in xrange(3):
            self.pub_vel.publish(self.drive)
        
    def turnRealYaw(self,degree):
        self.stop(2)
        if degree > 180:
            degree = degree%360-360
        elif degree < -180:
            degree = degree%360+360
        self.pub_yaw.publish(Float64(math.radians(degree)))
        rospy.sleep(5.3*degree/180)

    def turnAbsYaw(self,degree=0):
        self.stop(2)                    
        self.pub_abs_yaw.publish(Float64(math.radians(degree)))
        rospy.sleep(3)

    def depth(self,high):
        self.stop(2)
        self.pub_abs_depth.publish(high)
    
    # def run(self,position,turn=False,move=1):
    #     x2=position[0];y2=position[1] #;z2=position[2]
    #     x1=self.auv_state[0];y1=self.auv_state[1];z1=self.auv_state[2]
    #     if turn == True:
    #         move = -1
    #     for pose in range(len(position)):
    #         if position[pose] == 777: #not set position
    #             continue    
    #         while math.sqrt((x2-round(x1,2))**2+(y2-round(y1,2)**2)) >1:
    #             print self.auv_state
    #             if pose == 0:
    #                 if x1>x2:
    #                     self.drive.linear.x-=move
    #                 else:
    #                     self.drive.linear.x+=move
    #             elif pose == 1:
    #                 if y1>y2:
    #                     self.drive.linear.y-=move
    #                 else:
    #                     self.drive.linear.y+=move
    #             # elif pose == 2:
    #             #     self.drive.linear.z-=move
    #             #     self.drive.linear.z+=move
    #             self.pub_vel.publish(self.drive)
    #         rospy.sleep(2)
    #     self.stop(2)
            
    def runTriangle(self,position,move=1):
        start = (self.auv_state[0],self.auv_state[1],self.auv_state[2])
        x2=position[0];y2=position[1] #;z2=position[2]
        while math.sqrt((x2-self.auv_state[0])**2+(y2-self.auv_state[1])**2) >0.5:
            degree = math.degrees(math.atan2(position[1]-self.auv_state[1],position[0]-self.auv_state[0])-self.auv_state[5])
            print 'degree: ',degree
            if degree > 0.5:
                self.turnRealYaw(degree)
                rospy.sleep(0.3)
            print 'distance',math.sqrt((x2-self.auv_state[0])**2+(y2-self.auv_state[1])**2)
            # print self.auv_state
            self.auvMove('forward')
            rospy.sleep(0.3)
        self.stop(0.1)

if __name__ == '__main__':
    # while not rospy.is_shutdown():  
        control=Zeabuscontrol()
        position = [int(input('x: ')),int(input('y: '))]#,int(input())]
        control.runTriangle(position)
        control.stop(2)
        # angle = int(input('angle:'))
        # control.turnRealYaw(angle)
        print control.auv_state
    
    # control.run(position)
    # control.stop()
    # control.turnRealYaw(-90)
    # control.depth(0.5)