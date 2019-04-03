#!/usr/bin/env python

import rospy
import numpy as np
from turtlesim.srv import Spawn,SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sin, cos, sqrt
from turtle.srv import AssignMission
from turtle.msg import RobotMission
from random import randint
import time


class SingleTurtleControl():
    def __init__(self):
        # Start a new node
        rospy.init_node('SingleTurtleControl',anonymous=True)

        #############################
        ######   Service       ######
        #############################
        
        # use the 'spawn' service to create more turtles
        rospy.wait_for_service('spawn')
        try:
            turtle_spawn = rospy.ServiceProxy('spawn',Spawn)
            pass
        except rospy.ServiceException as ex:
            rospy.logerr('Couldnt spawn! Exception: {}'.format(ex))
            rospy.signal_shutdown('Couldnt spawn!')

        try:
            self.turtle_name = turtle_spawn(randint(3,7),randint(3,7),0,None).name
            pass
        except rospy.ServiceException as ex:
            rospy.logerr('Couldnt spawn! Exception: {}'.format(ex))
            rospy.signal_shutdown('Couldnt spawn!')
        
        # use service '{}/set_pen'
        rospy.wait_for_service('{}/set_pen'.format(self.turtle_name))
        try:
            self.setPen = rospy.ServiceProxy('{}/set_pen'.format(self.turtle_name),SetPen)
            pass
        except rospy.ServiceException as ex:
            rospy.logerr('Couldnt set pen! Exception: {}'.format(ex))
            rospy.signal_shutdown('Couldnt set pen!')

        # subscribe to /turtle1/pose with callback, self.updatePose
        rospy.pose_subscriber = rospy.Subscriber('{}/pose'.format(self.turtle_name),Pose,self.updatePose)
        self.pose = Pose()

        # Create a publisher to /turtle1/cmd_vel
        self.velocity_publisher = rospy.Publisher('{}/cmd_vel'.format(self.turtle_name),Twist,queue_size=10)
        self.twist = Twist()

        self.rate = rospy.Rate(10)

        # use service "assign_mission"
        rospy.wait_for_service('assign_mission')
        try:
            self.get_mission = rospy.ServiceProxy('assign_mission',AssignMission)
            pass
        except rospy.ServiceException as ex:
            rospy.logerr('Couldnt get mission! Exception: {}'.format(ex))
            rospy.signal_shutdown('Couldnt get mission!')

        self.run = False
        while not self.run:
            rospy.sleep(0.001)

        self.ControlTurtle()
       


    def ControlTurtle(self):
        mission = self.get_mission()
        print(mission)
        for i in range(len(mission.mission)):
            robot_id = 'turtle' + str(mission.mission[i].robotid+1)
            r = mission.mission[i].r
            g = mission.mission[i].g
            b = mission.mission[i].b
            width = mission.mission[i].width
            edge = np.array(mission.mission[i].data)\
                   .reshape((len(mission.mission[i].data)/2,2))
            distBetweenPoints = 0.3
            if robot_id == self.turtle_name:
                self.FullfillShape(edge,r,g,b,width,distBetweenPoints)


    # control robot to move to fullfill a shape
    def FullfillShape(self, edge, r,g,b,penWidth, distBetweenPoints):
        # Add more points to the given edge
        edge = self.AddPoint(edge, distBetweenPoints)

        self.target = Pose()
        self.target.x = edge[0][0]
        self.target.y = edge[0][1]

        self.setPen(r,g,b,penWidth,1)

        dist = sqrt(pow(self.target.x-self.pose.x,2)+\
                    pow(self.target.y-self.pose.y,2))

        while not rospy.is_shutdown() and dist > 0.1:
            cmdVx = 5 * (self.target.x - self.pose.x)
            cmdVy = 5 * (self.target.y - self.pose.y)
            self.Feedback(cmdVx,cmdVy,0.4)
            self.LimitCmds(0.5,1)
            print self.turtle_name

            self.velocity_publisher.publish(self.twist)
            dist = sqrt(pow(self.target.x-self.pose.x,2)+\
                        pow(self.target.y-self.pose.y,2))
            self.rate.sleep()         

        # begin to draw
        self.setPen(r,g,b,penWidth,0)

        numOfEntry = 0
        leftDisplay = numOfEntry
        rightDisplay = numOfEntry + 1
        for i in range(edge.shape[0]):

            if i % 4 == 0:
                self.target.x = edge[leftDisplay][0]
                self.target.y = edge[leftDisplay][1]
                leftDisplay = leftDisplay - 1
                if leftDisplay < 0:
                    leftDisplay = edge.shape[0] - 1
            elif i % 4 == 1:
                self.target.x = edge[leftDisplay][0]
                self.target.y = edge[leftDisplay][1]
                leftDisplay = leftDisplay - 1
                if leftDisplay < 0:
                    leftDisplay = edge.shape[0] - 1
            elif i % 4 == 2:
                self.target.x = edge[rightDisplay][0]
                self.target.y = edge[rightDisplay][1]
                rightDisplay = rightDisplay + 1
                if rightDisplay > edge.shape[0] - 1:
                    rightDisplay = 0
            elif i % 4 == 3:
                self.target.x = edge[rightDisplay][0]
                self.target.y = edge[rightDisplay][1]
                rightDisplay = rightDisplay + 1
                if rightDisplay > edge.shape[0] - 1:
                    rightDisplay = 0

            dist = sqrt(pow(self.target.x-self.pose.x,2)+\
                        pow(self.target.y-self.pose.y,2))

            while not rospy.is_shutdown() and dist > 0.1:
                cmdVx = 5 * (self.target.x - self.pose.x)
                cmdVy = 5 * (self.target.y - self.pose.y)
                self.Feedback(cmdVx,cmdVy,0.4)
                self.LimitCmds(0.5,1)
                print self.turtle_name

                self.velocity_publisher.publish(self.twist)
                dist = sqrt(pow(self.target.x-self.pose.x,2)+\
                            pow(self.target.y-self.pose.y,2))
                self.rate.sleep()

    # Add points to each edge of the shape
    def AddPoint(self,edge, lenBetweenPoints):
        newedge = np.array([edge[0]])

        for i in range (edge.shape[0]-1):
            # compute the length of each edge
            lenOfEdge = sqrt(pow(edge[i+1][0]-edge[i][0],2)+\
                             pow(edge[i+1][1]-edge[i][1],2))
            numOfPoints = np.ceil(lenOfEdge/lenBetweenPoints)+1
            
            for j in range (int(numOfPoints) - 1):
                xOfPoint = edge[i][0] + \
                           (edge[i+1][0]-edge[i][0]) / numOfPoints *\
                           (j + 1)
                yOfPoint = edge[i][1] + \
                           (edge[i+1][1]-edge[i][1]) / numOfPoints *\
                           (j + 1)
                newedge = np.append(newedge, [[xOfPoint,yOfPoint]],axis=0)

            newedge = np.append(newedge, [edge[i+1]], axis=0)

        return newedge

    # feedback controller
    def Feedback(self,cmdVx,cmdVy,epsilon):
        self.twist.linear.x = cos(self.pose.theta) * cmdVx + \
                              sin(self.pose.theta) * cmdVy
        self.twist.angular.z = (cos(self.pose.theta) * cmdVy - \
                                sin(self.pose.theta) * cmdVx) / epsilon

        pass
    
    # function used to limit the linear velocity and angular velocity
    def LimitCmds(self,maxVel,wheel2Center):
        leftVel = self.twist.linear.x - self.twist.angular.z/wheel2Center
        rightVel = self.twist.linear.x + self.twist.angular.z/wheel2Center

        if leftVel>maxVel or leftVel<-maxVel:
            rightVel = rightVel * maxVel / abs(leftVel)
            leftVel = leftVel * maxVel / abs(leftVel)
            self.twist.linear.x = (leftVel + rightVel)/2.0
            self.twist.angular.z = (rightVel - leftVel)/wheel2Center

        if rightVel>maxVel or rightVel<-maxVel:
            leftVel = leftVel * maxVel / abs(rightVel)
            rightVel = rightVel * maxVel / abs(rightVel)
            self.twist.linear.x = (leftVel + rightVel)/2.0
            self.twist.angular.z = (rightVel - leftVel)/wheel2Center

        pass

    # callback function: updatePose
    def updatePose(self,data):
        self.pose = data
        self.run = True
        pass

if __name__ == '__main__':
    start = SingleTurtleControl()
