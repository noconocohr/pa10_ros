#!/usr/bin/env python

import roslib; roslib.load_manifest('pa10_controller_interface')
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
            return addItmToQueueService(gripperArg, endOfQueue, utils.tupleToDeg(jointPositions), commandArg)
        except rospy.ServiceException, e:
            rospy.logerr("Service call %s failed: %s", serviceName, e)

        return False

    def setJointPositions(self, jointPositions):
        serviceName = '/pa10/setJointsConfig'
        gripperArg = False # TODO: check what True does
        commandArg = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        rospy.wait_for_service(serviceName)
        
        try:
            setJointsConfigService = rospy.ServiceProxy(serviceName, setJointConfig)
            return setJointsConfigService(gripperArg, utils.tupleToDeg(jointPositions), commandArg)
        except rospy.ServiceException, e:
            rospy.logerr("Service call %s failed: %s", serviceName, e)

        return False 




"""
MotionControllerSimulator

This class simulates the motion controller for an industrial robot.

This class IS threadsafe

"""
class MotionControllerSimulator():

    """
    Constructor of motion controller simulator
    """
    def __init__(self, num_joints, initial_joint_state, update_rate = 100, buffer_size = 0):
        
        # PA10 service caller class
        self.PA10ServiceCaller = PA10ServiceCaller()

        # Class lock
        self.lock = threading.Lock()

        self.queueCounter = 0
        
        # Motion loop update rate (higher update rates result in smoother simulated motion)
        self.update_rate = update_rate
        rospy.loginfo("Setting motion update rate (hz): %f", self.update_rate)
        
        # Initialize joint position
        self.joint_positions = initial_joint_state
        rospy.loginfo("Setting initial joint state: %s", str(initial_joint_state))
        
        # Initialize motion buffer (contains joint position lists)
        self.motion_buffer = Queue.Queue(buffer_size)
        rospy.loginfo("Setting motion buffer size: %i", buffer_size)
        
        # Shutdown signal
        self.sig_shutdown = False
        
        # Stop signal
        self.sig_stop = False
        
        # Motion thread
        self.motion_thread = threading.Thread(target=self._motion_worker)
        self.motion_thread.daemon = True
        self.motion_thread.start()
    

    """
    """
    def add_motion_waypoint(self, point):
        self.motion_buffer.put(point)
            
                
    """
    """
    def get_joint_positions(self):
        positions = self.PA10ServiceCaller.getJointPositions()
        # positions = (0.0,0.0,0.0,0.0,0.0,0.0,0.0)
        if positions == False:
            self.stop()
            return

        with self.lock:
            self.joint_positions = positions
            return self.joint_positions[:]
            
    """
    """
    def is_in_motion(self):
        return not self.motion_buffer.empty()
        
    """
    """
    def shutdown(self): 
        self.sig_shutdown = True
        rospy.loginfo('Motion_Controller shutdown signaled')
        
    """
    """
    def stop(self):
        rospy.loginfo('Motion_Controller stop signaled')
        with self.lock:
            self._clear_buffer()
            self.sig_stop = True
        
    """
    """
    def interpolate(self, last_pt, current_pt, alpha):
        intermediate_pt = JointTrajectoryPoint()
        for last_joint, current_joint in zip(last_pt.positions, current_pt.positions):
            intermediate_pt.positions.append(last_joint + alpha*(current_joint-last_joint))
        intermediate_pt.time_from_start = last_pt.time_from_start + rospy.Duration(alpha*(current_pt.time_from_start.to_sec() - last_pt.time_from_start.to_sec()))
        return intermediate_pt
        
    """
    """
    def _clear_buffer(self):
        with self.motion_buffer.mutex:
            self.motion_buffer.queue.clear()
            
    """
    """
    def _move_to(self, point, dur):
            
        #rospy.sleep(dur)
        
        with self.lock:
            if not self.sig_stop:
                if self.PA10ServiceCaller.addItemToQueue(False, point.positions) == False:
                    rospy.logerr("Error when moveing to new position")
                    self.stop()
                    self.queueCounter = 0
                else:
                    self.queueCounter = self.queueCounter + 1
                #rospy.loginfo('Moved to position: %s in %s', str(self.joint_positions), str(dur))
                
            else:
                rospy.loginfo('Stoping motion immediately, clearing stop signal')
                self.sig_stop = False
      
    def _execute_trajectory(self, last_goal_point):
        with self.lock:
            self.PA10ServiceCaller.addItemToQueue(True, last_goal_point.positions)
            rospy.loginfo('Executing queue with %d points', self.queueCounter)
            self.queueCounter = 0
            

    """
    """
    def _motion_worker(self):
    
        rospy.loginfo('Starting motion worker in motion controller simulator')
        first_run = False
        move_duration = rospy.Duration()
        if self.update_rate <> 0.:
            update_duration = rospy.Duration(1./self.update_rate)
        last_goal_point = JointTrajectoryPoint()
        
        with self.lock:
            last_goal_point.positions = self.joint_positions[:]
            
        while not self.sig_shutdown:
        
            try:

                # Execute trajectory when queue empty
                if first_run and self.motion_buffer.empty(): #TODO do this with inherited class (check for execute=queue)
                    self._execute_trajectory(last_goal_point)

                current_goal_point = self.motion_buffer.get()
                
                # If the current time from start is less than the last, then it's a new trajectory
                if current_goal_point.time_from_start < last_goal_point.time_from_start:
                    move_duration = current_goal_point.time_from_start
                # Else it's an existing trajectory and subtract the two
                else:
                    # If current move duration is greater than update_duration, move arm to interpolated joint position
                    # Provide an exception to this rule: if update rate is <=0, do not add interpolated points
                    move_duration = current_goal_point.time_from_start - last_goal_point.time_from_start
                    if self.update_rate > 0:
                        while update_duration < move_duration:
                            intermediate_goal_point = self.interpolate(last_goal_point, current_goal_point, update_duration.to_sec()/move_duration.to_sec())
                            self._move_to(intermediate_goal_point, update_duration.to_sec()) #TODO should this use min(update_duration, 0.5*move_duration) to smooth timing?
                            last_goal_point = copy.deepcopy(intermediate_goal_point)
                            move_duration = current_goal_point.time_from_start - intermediate_goal_point.time_from_start

                self._move_to(current_goal_point, move_duration)
                    
                last_goal_point = copy.deepcopy(current_goal_point)
                first_run = True
                
            except Exception as e:
                rospy.logerr('Unexpected exception: %s', e)
   
    
        rospy.loginfo("Shutting down motion controller")
        
        
        

"""
IndustrialRobotSimulator

This class simulates an industrial robot controller.  The simulator
adheres to the ROS-Industrial robot driver specification: 

http://www.ros.org/wiki/Industrial/Industrial_Robot_Driver_Spec

TODO: Currently the simulator only supports the bare minimum motion
interface.

TODO: Interfaces to add:
Robot status
Joint streaming
All services

"""
class IndustrialRobotSimulatorNode():


    """
    Constructor of industrial robot simulator
    """
    def __init__(self):
        rospy.init_node('pa10_controller_interface')

        # Class lock
        self.lock = threading.Lock()
        
        # Publish rate (hz)
        self.pub_rate = rospy.get_param('pub_rate', 10.0)
        rospy.loginfo("Setting publish rate (hz) based on parameter: %f", self.pub_rate)
        
        # Joint names
        def_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"] 
        self.joint_names = rospy.get_param('controller_joint_names', def_joint_names)
        if len(self.joint_names) == 0:
            rospy.logwarn("Joint list is empty, did you set controller_joint_name?")
        rospy.loginfo("Setting joint names based on parameter: %s", str(self.joint_names))
        
        # Setup initial joint positions
        num_joints = len(self.joint_names)
        initial_joint_state = rospy.get_param('initial_joint_state', [0]*num_joints)
        same_len = len(initial_joint_state) == num_joints
        all_num  = all(type(x) is int or type(x) is float for x in initial_joint_state)
        if not same_len or not all_num:
            initial_joint_state = [0]*num_joints
            rospy.logwarn("Invalid initial_joint_state parameter, defaulting to all-zeros "
                "(len: %s, types: %s).", same_len, all_num)

        # retrieve update rate
        motion_update_rate = rospy.get_param('motion_update_rate', 15.);  #set param to 0 to ignore interpolated motion
        self.motion_ctrl = MotionControllerSimulator(num_joints, initial_joint_state, update_rate = motion_update_rate)

        # Published to joint states
        rospy.loginfo("Creating joint state publisher")
        self.joint_state_pub = rospy.Publisher('joint_states', JointState)
        
        # Published to joint feedback
        rospy.loginfo("Creating joint feedback publisher")
        self.joint_feedback_pub = rospy.Publisher('feedback_states', FollowJointTrajectoryFeedback)

        # Subscribe to a joint trajectory
        rospy.loginfo("Creating joint trajectory subscriber")
        self.joint_path_sub = rospy.Subscriber('joint_path_command', JointTrajectory, self.trajectory_callback)

        # Timed task (started automatically)
        period = rospy.Duration(1.0/self.pub_rate)
        rospy.loginfo('Setting up publish worker with period (sec): %s', str(period.to_sec()))
        rospy.Timer(period, self.publish_worker)
        
        # Clean up init
        rospy.on_shutdown(self.motion_ctrl.shutdown)




    """
    The publish worker is executed at a fixed rate.  The publishes the various
    state and status information for the robot.
    """
    def publish_worker(self, event):
        self.joint_state_publisher()
            
       
    """
    The joint state publisher publishes the current joint state and the current
    feedback state (as these are closely releated)
    """       
    def joint_state_publisher(self):
    
    
        try:
    
            joint_state_msg = JointState()
            joint_fb_msg = FollowJointTrajectoryFeedback()
            time = rospy.Time.now()
            
            with self.lock:
                
                #Joint states
                    
                joint_state_msg.header.stamp = time
                joint_state_msg.name = self.joint_names
                joint_state_msg.position = self.motion_ctrl.get_joint_positions()
                
                self.joint_state_pub.publish(joint_state_msg)
                
                #Joint feedback
                joint_fb_msg.header.stamp = time
                joint_fb_msg.joint_names = self.joint_names
                joint_fb_msg.actual.positions = self.motion_ctrl.get_joint_positions()
                
                self.joint_feedback_pub.publish(joint_fb_msg)
                         
        except Exception as e:
            rospy.logerr('Unexpected exception in joint state publisher: %s', e)
      
                        
    """
    Trajectory subscription callback (gets called whenever a joint trajectory
    is received).

    @param msg_in: joint trajectory message
    @type  msg_in: JointTrajectory
    """
    def trajectory_callback(self, msg_in):
    
        try:
    
            rospy.loginfo('Received trajectory with %s points, executing callback', str(len(msg_in.points)))
            
            if self.motion_ctrl.is_in_motion():
                if len(msg_in.points) > 0:
                    rospy.logerr('Received trajectory while still in motion, trajectory splicing not supported')
                else:
                    rospy.loginfo('Received empty trajectory while still in motion, stopping current trajectory')
                self.motion_ctrl.stop()
                
            else:
        
                for point in msg_in.points:
                    #point = self._to_controller_order(msg_in.joint_names, point)
                    self.motion_ctrl.add_motion_waypoint(point)
                    
        except Exception as e:
            rospy.logerr('Unexpected exception: %s', e)

        
        rospy.loginfo('Exiting trajectory callback')


    """
    Remaps point to controller joint order

    @param keys:   keys defining joint value order
    @type  keys:   list
    @param point:  joint trajectory point
    @type  point:  JointTrajectoryPoint
    
    @return point: reorder point
    @type point: JointTrajectoryPoint
    """
    def _to_controller_order(self, keys, point):
    
        #rospy.loginfo('to controller order, keys: %s, point: %s', str(keys), str(point))
        pt_rtn = copy.deepcopy(point)
        pt_rtn.positions = self._remap_order(self.joint_names, keys, point.positions)
        
        return pt_rtn
        
    def _remap_order(self, ordered_keys, value_keys, values):
        
        #rospy.loginfo('remap order, ordered_keys: %s, value_keys: %s, values: %s', str(ordered_keys), str(value_keys), str(values))
        ordered_values = []

        
        ordered_values = [0]*len(ordered_keys)
        mapping = dict(zip(value_keys, values))
        #rospy.loginfo('maping: %s', str(mapping))
        
        for i in range(len(ordered_keys)):
            ordered_values[i] = mapping[ordered_keys[i]]
            pass

        return ordered_values
        
        

        
        

if __name__ == '__main__':
    try:
        rospy.loginfo('Executing pa10_controller_interface')
        controller = IndustrialRobotSimulatorNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass

