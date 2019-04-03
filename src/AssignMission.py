#!/usr/bin/env python

from turtle.srv import AssignMission
from turtle.msg import RobotMission
import rospy


def handler_assign_mission(req):
    all_mission = []
    mission = RobotMission()
    mission.robotid = 1
    mission.r = 100
    mission.g = 100
    mission.b = 100
    mission.width = 5
    mission.data = [3, 3,\
                    2, 4,\
                    3, 5,\
                    4, 5,\
                    5, 4,\
                    4, 3,\
                    3, 3]
    all_mission.append(mission)

    mission = RobotMission()
    mission.robotid = 2
    mission.r = 100
    mission.g = 150
    mission.b = 200
    mission.width = 5
    mission.data = [5, 5,\
                    5, 10,\
                    9, 10,\
                    9, 5,\
                    5, 5]
    all_mission.append(mission)

    mission = RobotMission()
    mission.robotid = 3
    mission.r = 50
    mission.g = 150
    mission.b = 100
    mission.width = 5
    mission.data = [1, 8,\
                    1, 10,\
                    10, 10,\
                    10, 8,\
                    1, 8]
    all_mission.append(mission)

    mission = RobotMission()
    mission.robotid = 4
    mission.r = 30
    mission.g = 30
    mission.b = 30
    mission.width = 5
    mission.data = [7, 8,\
                    7, 3,\
                    5, 3,\
                    5, 8,\
                    7, 8]
    all_mission.append(mission) 
    
    return [all_mission]


def assign_mission_server():
    rospy.init_node('assign_mission')
    am = rospy.Service('assign_mission', AssignMission, handler_assign_mission)
    print "Ready for mission assignment"
    rospy.spin()

if __name__ == "__main__":
    assign_mission_server()
