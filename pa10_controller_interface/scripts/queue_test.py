#!/usr/bin/env python

import roslib; roslib.load_manifest('industrial_robot_simulator')
import rospy
import copy
import threading
import Queue
import utils

# Publish
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback 

# Subscribe
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Reference
from industrial_msgs.msg import TriState
import trajectory_msgs

# Services from pa10_server
from pa10controller.srv import *


"""
Class for calling services from pa10server application

"""
class PA10ServiceCaller():

    def getJointPositions(self):
        serviceName = '/pa10/getJointConfig'

        rospy.wait_for_service(serviceName)

        try:
            service = rospy.ServiceProxy(serviceName, getJointConfig)
            return utils.tupleToRad(service().positions)
        except rospy.ServiceException, e:
            rospy.logerr("Service call %s failed: %s", serviceName, e)

        return False

    def addItemToQueue(self, endOfQueue, jointPositions):
        serviceName = '/pa10/addItemtoQueue'
        gripperArg = False # TODO: check what True does
        commandArg = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        rospy.wait_for_service(serviceName)
        
        try:
            addItmToQueueService = rospy.ServiceProxy(serviceName, addToQueue)
            return addItmToQueueService(gripperArg, endOfQueue, jointPositions, commandArg)
        except rospy.ServiceException, e:
            rospy.logerr("Service call %s failed: %s", serviceName, e)

        return False

    def setJointPositions(self, jointPositions):
        serviceName = '/pa10/setJointsConfig'
        gripperArg = False # TODO: check what True does
        commandArg = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # print utils.tupleToDeg(jointPositions)
        # return

        rospy.wait_for_service(serviceName)
        
        try:
            setJointsConfigService = rospy.ServiceProxy(serviceName, setJointConfig)
            return setJointsConfigService(gripperArg, jointPositions, commandArg)
        except rospy.ServiceException, e:
            rospy.logerr("Service call %s failed: %s", serviceName, e)

        return False 


if __name__ == '__main__':
    pa10 = PA10ServiceCaller()
    rospy.init_node('pa10_queue_test')

    numberOfPoints = 50
    multiplier = 1.5

    for i in range(1, numberOfPoints + 1):
        joints = (0.0, 0.0, 0.0, 0.0, 0.0, multiplier*i*1.0, 0.0)
        pa10.addItemToQueue(False, joints)
        print joints

    print "backwards"

    for i in range(1, numberOfPoints + 1):
        joints = (0.0, 0.0, 0.0, 0.0, 0.0, numberOfPoints*multiplier-i*multiplier*1.0, 0.0)
        pa10.addItemToQueue(False, joints)
        print joints

    joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    print joints
    pa10.addItemToQueue(True, joints)

    print "done"