#!/usr/bin/env python

from random import randint

import roslaunch
import rospy
import numpy as np
from turtlesim.srv import Kill
from std_srvs.srv import Empty


def Launcher():
    rospy.init_node('turtlesim_launcher')

    # launch node turtlesim
    sim = roslaunch.Node('turtlesim','turtlesim_node')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    launch.launch(sim)
    rospy.loginfo('turtlesim')
    rospy.sleep(1)

    # use the 'clear' service to clear background color
    rospy.wait_for_service('clear')
    try:
        clear = rospy.ServiceProxy('clear',Empty)
        pass
    except rospy.ServiceException as ex:
        rospy.logerr('Couldnt clear! Exception: {}'.format(ex))
        rospy.signal_shutdown('Couldnt clear!')
    
    # set background color to white
    rospy.set_param('background_r',0)
    rospy.set_param('background_g',0)
    rospy.set_param('background_b',0)
    clear()

    # kill the first turtle
    rospy.wait_for_service('kill')
    try:
        turtleKill = rospy.ServiceProxy('kill',Kill)
        pass
    except rospy.ServiceException as ex:
        rospy.logerr('Couldnt kill! Exception: {}'.format(ex))
        rospy.signal_shutdown('Couldnt kill!')
    turtleKill('turtle1')

    genTurtle = roslaunch.Node('turtle','SingleTurtleControl.py',output="screen")
    num_turtle = 4
    procs = []
    rospy.loginfo('Launching {} turtles'.format(num_turtle))
    for i in range(num_turtle):
        genTurtle.name = 'gen_turtle{}'.format(i)
        procs.append(launch.launch(genTurtle))
        rospy.loginfo(genTurtle.name)
        rospy.sleep(1)

    rospy.loginfo('spinning!')
    while not rospy.is_shutdown():
        continue


if __name__ == '__main__':
    laucher = Launcher()
    
