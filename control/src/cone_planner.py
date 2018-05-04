#!/usr/bin/env python
import os
import numpy
import rospy
import time
import threading
import math
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from custm_msgs import cone_list



#####################################################
#         Define Threading Class                    #
#####################################################
class ThreadedFunction(threading.Thread):

    #####################################################
    #          Initialize Object                        #
    #####################################################
    def __init__(self, fcn_to_thread):
        threading.Thread.__init__(self)

        self.runnable = fcn_to_thread
        self.daemon = True

    def run(self):
        self.runnable()


#####################################################
#               Initialize Planner class            #
#####################################################
class cone_planner:
    def __init__(self, config):
    #####################################################
    #          Initialize Boolean States for System     #
    #####################################################
    # TODO: define 'states' needed

    #####################################################
    #    Initialize Variables for EXPLORATION MODE      #
    #####################################################
    # TODO: define 'variables' needed

    #####################################################
    #            External Boolean Parameters            #
    #####################################################
    # TODO: define 'external_var' needed

    #####################################################
    #                Internal Parameters                #
    #####################################################
    # TODO: define 'parameters' needed
    self.maxdist  = config.MAX_DIST                       # Initialize the maximal distance between cones
    self.wp_merge = config.MERGE_DIST                     # Initialize the distance for two waypoints to be merged

    #####################################################
    #         Initialize ROS Messages/Parameters        #
    #####################################################
    # TODO: check if anything else is needed
    self.time_prev = 0
    self.current_time = 0

    self.PUBLISH_RATE = config.PUBLISH_rate
    self.ODOM_feedback = Odometry()
    p = Pose()
    p.position.x = 0
    p.position.y = 0
    self.GOAL_WP = PoseArray()
    self.GOAL_WP.poses.append(p)

    #####################################################
    #         Initialize ROS Subscriber/Publisher       #
    #####################################################
    # TODO: check if any other messages are needed to be published
    rospy.init_node('planner_node', anonymous=True)
    self.pub_pwm_vel = rospy.Publisher('path', PoseArray, queue_size=10)
    self.rate = rospy.Rate(self.PUBLISH_RATE)



    #####################################################
    #               Initialize Subscriber               #
    #####################################################
    # TODO: check if any other messages are needed to be subscribed
    def initialize_subscriber(self):
        while not rospy.is_shutdown():
            while self.PURE_PURSUIT_ACTIVE:
                rospy.Subscriber('cone_list', cone_list, self.generate_waypoints)
                rospy.spin()

    #####################################################
    #               Initialize Subscriber               #
    #####################################################
    def generate_waypoints(self):
    # TODO: explain better the code
        """
        This function is called every time a new list of cones is generated.
        Insights:
        The waypoints are generate according to a simple policy:
        TO BE CONTINUED
        :return the list of the sorted centered waypoints
        """
        while not rospy.is_shutdown():
            print "PL:   Path Planning running:\r"
            ##################### Save cones in dedicated data structure. Vector #########################
            # TODO: read cones list. Check the documentation and discuss
            yellow_list = []
            blue_list = []
            orange_list = []
            waypoints = []

            dist_b_to_y = numpy.array([[self.distance(blue_cone, yellow_cone) for yellow_cone in yellow_list]
                                       for blue_cone in blue_list])
            argmin_blue      = numpy.argmin(dist_b_to_y, axis = 1)
            argmin_yellow    = numpy.argmin(dist_b_to_y, axis = 0)
            # Compute
            waypoints_blue   = [ 0.5(blue_cone + yellow_list[i]) for (blue_cone, i) in zip(blue_list, argmin_blue)]
            waypoints_yellow = [ 0.5(yellow_cone + blue_list[i]) for (yellow_cone, i) in zip(yellow_list, argmin_yellow)]
            # TODO: Merge waypoints and the sort. Sorting needed.


    #####################################################
    #                  Support functions                #
    #####################################################
    def distance(self, p1, p2):
        return numpy.sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

