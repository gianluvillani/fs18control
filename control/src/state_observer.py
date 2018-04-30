#!/usr/bin/env python

#####################################################
#####################################################
#####################################################
#                                                   #
#               Main Controller Interface           #
#                                                   #
#####################################################
#                                                   #
#     - publishing to Emergency Interrupt (40Hz)    #
#     - considers different Logic Flags to trigger  #
#       emergency or vehicle halt                   #
#     - checks whether checkpoint_publisher runs    #
#       | essential_topics are available            #
#       | mapping_complete | lidar_crash detection  #
#       is false etc
#     - 'e' start exploration mode                  #
#     - 'a' running autonomous racing               #
#     - 'r' reinitialize vehicle to last checkpoint #
#       using ros service to request initial pose   #
#                                                   #
# written by Philipp Rothenhaeusler - phirot@kth.se #
#                                                   #
#####################################################
#####################################################
#####################################################
#####################################################


import os
import tf
import time
import rospy
import numpy
import numpy as np
import threading
from std_msgs.msg import Float64
from control.msg import pwm_interface
import scipy.integrate as Integration
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose, Quaternion


#####################################################
#               Initialize Threading Class          #
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
#          Initialize Failsafe Class                #
#####################################################
class Vehicle:

    #####################################################
    #          Initialize Object                        #
    #####################################################
    def __init__(self):

        #####################################################
        #          Initialize Boolean States for System     #
        #####################################################
        self.MAIN_INTERFACE_STARTED = False
        self.SIMULATION_ACTIVE = False
        self.PARAMETERS_INITIALIZED = False
        self.PARAMETER_UPDATED = False
        self.MANUAL_CONTROL_ACTIVATED = False
        self.EXPLORATION_ACTIVATED = False
        self.AUTONOMOUS_RACING_ACTIVATED = False
        self.SAFETY_REGION_WATCHDOG_OK = False                  # Lidar Safety Region to detect Crash or Obstacles
        self.STATE_OBSERVER_ACTIVE = False
        self.PUBLISH_RATE_SIM = 40
        self.EMERGENCY_FLAG = False
        self.SUBSCRIBER_ACTIVE = False
        self.PUBLISHER_VEL_ACTIVE = False
        self.PUBLISHER_ODOM_ACTIVE = False

        #####################################################
        #          Initialize Locks for Threaded Access     #
        #####################################################
        self.PUBLISH_VEL_ACTIVE = False
        self.PUBLISH_ODOM_ACTIVE = False
        self.KEY_READ_ACTIVE = False
        self.STATE_UPDATED = False

        #####################################################
        #          Initialize Odomentry                     #
        #####################################################
        self.SYSTEM_GAIN = 1.7 / 15
        self.WHEEL_BASE = 0.324
        self.delta = 0
        self.R = 0
        self.v = 0
        self.v_prev = 0
        self.psi = 0
        self.psi_prev = 0
        self.dv = 0
        self.v_abs = 0
        self.x_odom = 0
        self.y_odom = 0
        self.psi_odom = 0
        self.dx = 0
        self.dy = 0
        self.dpsi = 0

        self.steering_PWM_max = 81
        self.steering_PWM_min = -81
        self.steering_DEG_max = 35
        self.steering_DEG_min = -35

        #####################################################
        #          Initialize System Dynamics               #
        #####################################################
        self.u = 0
        self.y = 0
        self.A = -2/3
        self.PHI = numpy.exp(self.A*(1./self.PUBLISH_RATE_SIM))
        self.B = 1.7/15
        self.C = 1
        self.D = 0
        self.B_INT = Integration.quad(lambda s: self.B*self.state_transition_system(s), 0, 1./self.PUBLISH_RATE_SIM)
        self.THETA = self.B_INT[0]
        self.dt = 0
        self.t_prev = 0

        self.control_input_velocity_PWM = 0
        self.control_input_steering_PWM = 0
        self.control_input_velocity_SI = 0
        self.control_input_steering_SI = 0
        self.odometry_obs = Odometry()
        self.odometry_obs_seq = 0
        self.velocity_output = Float64()
        self.velocity_output.data = 0

        self.STATE_UPDATED = False
        self.CONTROL_INPUT_UPDATED = False

        rospy.init_node('control_state_observer_node', anonymous=True)
        self.pub_vehicle_state = rospy.Publisher('control_state_observer', Float64, queue_size=1)
        self.pub_odometry = rospy.Publisher('control_state_observer_odometry', Odometry, queue_size=1)
        self.rate = rospy.Rate(self.PUBLISH_RATE_SIM)


    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #                   SET PARAMETER                   #
    #####################################################
    def set_f1vt18_parameter(self, parameter_name, value=False):
        resolved_global_name = "/f1vt18/" + parameter_name
        rospy.set_param(resolved_global_name, value)

    #####################################################
    #                   SET PARAMETER                   #
    #####################################################
    def get_f1vt18_parameter(self, parameter_name, value=False):
        resolved_global_name = "/f1vt18/" + parameter_name
        return rospy.get_param(resolved_global_name, value)

    #####################################################
    #              UPDATE PARAMETERS                    #
    #####################################################
    def update_parameters(self):
        while not rospy.is_shutdown():
            while not self.PARAMETERS_INITIALIZED:
                self.PARAMETERS_INITIALIZED = self.get_f1vt18_parameter("PARAMETERS_INITIALIZED")
                self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.MAIN_INTERFACE_STARTED)
                pass
            self.PARAMETER_UPDATED = True
            self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.MAIN_INTERFACE_STARTED)
        self.MAIN_INTERFACE_STARTED = False
        self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.MAIN_INTERFACE_STARTED)

    #####################################################
    #          Define State Transition System           #
    #####################################################
    def state_transition_system(self, t):
        return numpy.exp(self.A * t)

    ####################################################
    #             Subscribe to pwm_interface           #
    ####################################################
    def initialize_subscriber(self):
        while not rospy.is_shutdown():
            rospy.Subscriber("/pwm_interface", pwm_interface, self.fetch_input)
            #rospy.Subscriber("/odometry/filtered", pwm_interface, self.fetch_input)
            self.SUBSCRIBER_ACTIVE = True
            rospy.spin()
        self.SUBSCRIBER_ACTIVE = False

    ####################################################
    #             Publish to /encoder/filtered         #
    ####################################################
    def publish_velocity(self):
        while not rospy.is_shutdown():
            self.PUBLISH_VEL_ACTIVE = True
            self.pub_vehicle_state.publish(self.velocity_output)
            self.PUBLISH_VEL_ACTIVE = False
            self.rate.sleep()
            self.PUBLISHER_VEL_ACTIVE = True
        self.PUBLISHER_VEL_ACTIVE = False

    ####################################################
    #             Publish to /encoder/filtered         #
    ####################################################
    def publish_odometry(self):
        while not rospy.is_shutdown():
            self.PUBLISH_ODOM_ACTIVE = True
            self.pub_odometry.publish(self.odometry_obs)
            self.PUBLISH_ODOM_ACTIVE = False
            self.rate.sleep()
            self.PUBLISHER_ODOM_ACTIVE = True
        self.PUBLISHER_ODOM_ACTIVE = False

    ####################################################
    #   Callback function for updating control input   #
    ####################################################
    def fetch_input(self, ctrl_input):
        self.control_input_velocity_PWM = ctrl_input.velocity
        self.control_input_steering_PWM = ctrl_input.steering

        self.control_input_velocity_SI = self.SYSTEM_GAIN*self.control_input_velocity_PWM
        self.control_input_steering_SI = ((self.steering_PWM_max - self.control_input_steering_PWM) / (self.steering_PWM_max - self.steering_PWM_min))\
                                      *(self.steering_DEG_max-self.steering_DEG_min)-self.steering_DEG_min
        #print('Received {0}'.format(self.control_input_velocity_PWM))
        self.CONTROL_INPUT_UPDATED = True



    ####################################################
    #                   UPDATE STATE                   #
    ####################################################
    def update_state(self):
        while not rospy.is_shutdown():
            while not self.PARAMETERS_INITIALIZED:
                pass

            self.CONTROL_INPUT_UPDATED = False
            #print("WAIT FOR CONTROL_INPUT_UPDATED\r")
            while not self.CONTROL_INPUT_UPDATED:
                pass
            self.dt = rospy.get_time() - self.t_prev
            self.u = self.control_input_velocity_PWM
            #print("UPDATED CONTROL INPUT SUCCESSFUL\r")

            #print("COMPUTING NEW_STATE\r")
            #print('xc= {0}, xn= {1}, u= {2} \r'.format(self.v_prev, self.v, self.u))
            #print('PHI= {0}, THETA= {1} '.format(self.PHI, self.THETA))
            #print('A= {0}, B_INT= {1} '.format(self.A, self.B_INT))

            self.v = self.PHI * self.v_prev + self.THETA * self.u
            self.y = self.C * self.v + self.D * self.u
            self.v_prev = self.v

            while self.PUBLISH_VEL_ACTIVE:
                pass
            self.velocity_output.data = self.y
            self.STATE_UPDATED = True
            self.t_prev = rospy.get_time()
        self.SIMULATION_ACTIVE = False

    ####################################################
    #                   UPDATE STATE                   #
    ####################################################
    def update_odometry(self):
        while not rospy.is_shutdown():
            while not self.PARAMETERS_INITIALIZED:
                pass
            while not self.STATE_UPDATED:
                pass

            self.delta = self.control_input_steering_SI
            self.dv = self.v - self.v_prev

            self.v_prev = self.v

            self.R = self.WHEEL_BASE / self.delta
            self.psi = self.v / self.R
            self.dpsi = self.psi - self.psi_prev
            self.psi_prev = self.psi

            self.dx = np.cos(self.psi) * self.dv
            self.dy = np.sin(self.psi) * self.dv

            self.x_odom += self.dx
            self.y_odom += self.dy
            self.psi_odom += self.dpsi

            q = tf.transformations.quaternion_from_euler(0, 0, self.psi_odom)


            self.odometry_obs.header.seq = self.odometry_obs_seq
            self.odometry_obs.pose.pose.position.x = self.x_odom
            self.odometry_obs.pose.pose.position.y = self.y_odom
            self.odometry_obs.pose.pose.orientation = q

            self.STATE_UPDATED = True
        self.SIMULATION_ACTIVE = False

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def display_interface(self):
        #####################################################
        #               WHILE LOOP to DISPLAY INTERFACE     #
        #####################################################
        while not rospy.is_shutdown():

            #####################################################
            # Initialize PARAMETERS FROM ROS PARAMETER SERVER   #
            #####################################################
            print("##################################\r")
            print(" Waiting for PARAMETERS INI       \r")
            print("----------------------------------\r")
            print('Press q if frozen...              \r')
            while not self.PARAMETERS_INITIALIZED and not rospy.is_shutdown():
                pass
            print("  PARAMETERS INITIALIZED          \r")
            print("----------------------------------\r")
            self.STATE_OBSERVER_ACTIVE = True

            #####################################################
            #     UPDATE PARAMETERS FROM ROS PARAMETER SERVER   #
            #####################################################
            self.PARAMETER_UPDATED = False
            print("##################################\r")
            print("  Waiting for PARAMETERS UPDATE   \r")
            print("----------------------------------\r")
            print('Press q if frozen...              \r')
            #while not self.PARAMETER_UPDATED and not rospy.is_shutdown():
            #    pass
            print("    PARAMETERS UPDATED            \r")
            print("----------------------------------\r")

            #####################################################
            # UPDATE STATES                                     #
            #####################################################
            self.STATE_UPDATED = False
            print("##################################\r")
            print(" Waiting for STATE UPDATE         \r")
            print("----------------------------------\r")
            print('Press q if frozen...              \r')
            while not self.STATE_UPDATED and not rospy.is_shutdown():
                pass
            print("COMPUTING NEW_STATE\r")
            print('xc= {0:2.2f}, xn= {1:2.2f}, u= {2:2.2f} \r'.format(self.v_prev, self.v, self.u))
            print("----------------------------------\r")
            print("   STATE UPDATED                  \r")
            print("----------------------------------\r")


            #####################################################
            #        Print Control Interface to Terminal        #
            #####################################################
            print("##################################\r")
            print('Simulation Model Status :  {0}    \r'.format(self.STATE_OBSERVER_ACTIVE))
            print("----------------------------------\r")
            print("##################################\r")
            time.sleep(0.5)

#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        print("##################################\r")
        print("# PROGRAM HAS BEEN STARTED       #\r")
        print("##################################\r")
        print("... initialize vehicle object     \r")
        time.sleep(1)
        sim_f1vt18 = Vehicle()

        thread_sub = ThreadedFunction(sim_f1vt18.initialize_subscriber)
        thread_pub_vel = ThreadedFunction(sim_f1vt18.publish_velocity)
        thread_pub_odom = ThreadedFunction(sim_f1vt18.publish_odometry)
        thread_upd_param = ThreadedFunction(sim_f1vt18.update_parameters)
        thread_upd_state = ThreadedFunction(sim_f1vt18.update_state)
        thread_upd_odom = ThreadedFunction(sim_f1vt18.update_odometry)

        thread_sub.start()
        thread_pub_vel.start()
        thread_pub_odom.start()
        thread_upd_param.start()
        thread_upd_state.start()
        thread_upd_odom.start()

        sim_f1vt18.display_interface()

        rospy.set_param('/f1vt18/STATE_OBSERVER_ACTIVE', False)
    except rospy.ROSInterruptException:
        print("##################################\r")
        print("# PROGRAM HAS BEEN INTERRUPTED   #\r")
        print("##################################\r")
        rospy.set_param('/f1vt18/STATE_OBSERVER_ACTIVE', False)
        pass
