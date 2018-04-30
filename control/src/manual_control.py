#!/usr/bin/env python

#####################################################
#####################################################
#####################################################
#                                                   #
#                Manual Controller                  #
#                                                   #
#####################################################
#                                                   #
#       - simplified control with w,a,s,d,b,q       #
#       - clarified variable definition             #
#       - clean code structure                      #
#       - merged two nodes                          #
#       - increased publishing frequency            #
#           to pwm interface ('pwm_interface')      #
#       - PWM_Interface *.ino (Teensy) idles if no  #
#           control signal is received              #
#       - using clear screen instead of curses      #
#       - requires pip install readchar             #
#       - included threading                        #
#       - updated new msg file in same package      #
#       - included active braking test (0 then -20) #
#                                                   #
# written by Philipp Rothenhaeusler - phirot@kth.se #
#                                                   #
#####################################################
#####################################################
#####################################################
#####################################################

import os
import time
import rospy
import readchar
import threading
from control.msg import pwm_interface
from control.msg import pwm_controller


#####################################################
#               Initialize Threading                #
#####################################################
class ThreadedFunction(threading.Thread):

    def __init__(self, fcn_to_thread):
        threading.Thread.__init__(self)

        self.runnable = fcn_to_thread
        self.daemon = True

    def run(self):
        self.runnable()


class VehicleManualController:

    def __init__(self):
        #####################################################
        #               Initialize Variables                #
        #####################################################
        self.MANUAL_CONTROL_ACTIVATED = False
        self.PARAMETERS_INITIALIZED = False
        self.PARAMETER_UPDATED = False
        self.EMERGENCY_FLAG = False
        self.REFERENCE_UPDATED = False
        self.UPDATE_DISPLAY_REQUEST = False

        self.SIMULATION = True
        self.BRAKE_ESC_IDLE = False
        self.BRAKE_ESC_IDLE_COUNT = 0
        self.BRAKE_ESC_IDLE_MAX = 5
        self.BRAKE_ACTIVE = False
        self.WRITE_FLAG = False
        self.STATE_UPDATED = False
        self.VELOCITY_REFERENCE_UPDATED = False
        self.REINITIALIZE_ZERO_STEER = False                                 # STEER reset request => 'r'
        self.KEY = ''

        self.STEP_VEL = 5
        self.STEP_STEER = 20
        self.PWM_STEER_OFFSET = 9

        self.SAT_VEL_MIN = -100
        self.SAT_VEL_MAX = 100

        self.SAT_STEER_MIN = -81
        self.SAT_STEER_MAX = 81

        self.PWM_VEL = 0
        self.PWM_VEL_SAT = 0
        self.PWM_STEER = 0
        self.PWM_STEER_SAT = 0

        self.PWM_STEER_INITIALIZED = False               # Servo too weak to go back
        self.PWM_STEER_ZERO_CALIBRATION = 81             # wheels aligned more  straight if first steer right, then zero
        self.PWM_STEER_JITTER = 0
        self.PWM_STEER_JITTER_OFFSET = 5
        self.PWM_JITTER_timer0 = 0
        self.PWM_JITTER_timer1 = 0
        self.PWM_JITTER_dT = 0.1

        self.SYSTEM_GAIN = 1.7/15                       # CTRL Input of 15 corresponds to 1.7 m/s (estimate)


        #####################################################
        #               Initialize ROS NODE                 #
        #####################################################
        self.PUBLISH_RATE = 40
        self.PWM = pwm_interface()
        self.REF_VEL = pwm_controller()

        rospy.init_node('manual_control_node', anonymous=True)
        self.pub_pwm = rospy.Publisher('pwm_interface', pwm_interface, queue_size=10)
        self.pub_velocity_reference = rospy.Publisher('pwm_controller', pwm_controller, queue_size=10)
        self.rate = rospy.Rate(self.PUBLISH_RATE)

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
                pass
            self.get_f1vt18_parameter("PARAMETERS_INITIALIZED")
            self.get_f1vt18_parameter("PUBLISH_RATE_PWM")
            self.set_f1vt18_parameter("MANUAL_CONTROL_ACTIVATED")
            self.get_f1vt18_parameter("EMERGENCY_FLAG")
            self.PARAMETER_UPDATED = True

    #####################################################
    #          Publish STOP to PWM_INTERFACE            #
    #####################################################
    def publish_pwm(self):
        while not rospy.is_shutdown():
            while not self.MANUAL_CONTROL_ACTIVATED:
                pass
            while self.SIMULATION:
                pass
            self.PWM_VEL = min(max(self.PWM_VEL, self.SAT_VEL_MIN), self.SAT_VEL_MAX)
            self.PWM_STEER = min(max(self.PWM_STEER, self.SAT_STEER_MIN), self.SAT_STEER_MAX)
            self.PWM_VEL_SAT = self.PWM_VEL
            self.PWM_STEER_SAT = self.PWM_STEER
            self.PWM.velocity = self.PWM_VEL_SAT
            self.PWM.steering = self.PWM_STEER_SAT

            if self.REINITIALIZE_ZERO_STEER:
                if not self.PWM_STEER_INITIALIZED:
                    self.PWM.steering = self.PWM_STEER_ZERO_CALIBRATION
                    self.PWM_JITTER_timer0 = rospy.get_time()

                    while(rospy.get_time() - self.PWM_JITTER_timer0) < self.PWM_JITTER_dT:
                        self.pub_pwm.publish(self.PWM)
                        self.PWM_STEER_INITIALIZED = True
                        self.REINITIALIZE_ZERO_STEER = False
                    else:
                        pass
                else:
                    pass
            else:
                self.PWM_STEER_INITIALIZED = False              # initialize REF_STEER_PWM on next reset request

            self.pub_pwm.publish(self.PWM)
            self.STATE_UPDATED = True
            self.rate.sleep()

    #####################################################
    #    Publish Reference Velocity to PWM_Controller   #
    #####################################################
    def publish_reference_velocity(self):
        while not rospy.is_shutdown():
            while not self.MANUAL_CONTROL_ACTIVATED:
                pass
            while not self.SIMULATION:
                pass
            self.PWM_VEL = min(max(self.PWM_VEL, self.SAT_VEL_MIN), self.SAT_VEL_MAX)
            self.PWM_STEER = min(max(self.PWM_STEER, self.SAT_STEER_MIN), self.SAT_STEER_MAX)
            self.PWM_VEL_SAT = self.PWM_VEL
            self.PWM_STEER_SAT = self.PWM_STEER

            if self.REINITIALIZE_ZERO_STEER:
                if not self.PWM_STEER_INITIALIZED:
                    self.PWM.steering = self.PWM_STEER_ZERO_CALIBRATION
                    self.PWM_JITTER_timer0 = rospy.get_time()

                    while (rospy.get_time() - self.PWM_JITTER_timer0) < self.PWM_JITTER_dT:
                        self.pub_pwm.publish(self.PWM)
                        self.PWM_STEER_INITIALIZED = True
                        self.REINITIALIZE_ZERO_STEER = False
                    else:
                        pass
                else:
                    pass
            else:
                self.PWM_STEER_INITIALIZED = False  # initialize REF_STEER_PWM on next reset request

            self.REF_VEL.velocity = self.SYSTEM_GAIN*self.PWM_VEL_SAT
            self.REF_VEL.steering = 0
            self.pub_velocity_reference.publish(self.REF_VEL)
            self.VELOCITY_REFERENCE_UPDATED = True
            self.rate.sleep()

    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def display_interface(self):
        while not rospy.is_shutdown():
            self.PARAMETER_UPDATED = False
            while not self.PARAMETER_UPDATED:
                pass

            if self.SIMULATION:
                self.VELOCITY_REFERENCE_UPDATED = False
                while not self.VELOCITY_REFERENCE_UPDATED and not rospy.is_shutdown():
                    pass
            else:
                self.STATE_UPDATED = False
                while not self.STATE_UPDATED and not rospy.is_shutdown():
                    pass

            #####################################################
            #        Print Control Interface to Terminal        #
            #####################################################
            self.cls()
            print("######################################## -- Active --   \r")
            print("-----------------------------------------------------   \r")
            print('VELOCITY:                  [    w | s   ]   \r')
            print('STEER:                     [    a | d   ]   \r')
            print('BRAKE:             (toogle)[      b     ]   \r')
            print('RESET:             (switch)[      r     ]   \r')
            print('RESET-EMERGENCY:   (switch)[      e     ]   \r')
            print('SET-EMERGENCY:     (switch)[      u     ]   \r')
            print('TESTING/SIMULATION:(toogle)[      t     ]  -  {0} -    \r'.format(self.SIMULATION))
            print('QUIT:              (switch)[      q     ]   \r')
            print('########################################------------   \r')
            print('VEL: {0} | Steer: {1}\r'.format(self.PWM_VEL_SAT, self.PWM_STEER_SAT))
            print("-----------------------------------------------------   \r")
            print('User-Input: {0}\r'.format(self.KEY))
            while not self.UPDATE_DISPLAY_REQUEST:
                pass

            self.UPDATE_DISPLAY_REQUEST = False

    #####################################################
    #            Define User Input Function             #
    #####################################################
    def read_user_input(self):
        while not self.PARAMETERS_INITIALIZED:
            pass

        while self.KEY != 'q' and not rospy.is_shutdown():

            self.PARAMETER_UPDATED = False
            while not self.PARAMETER_UPDATED and not rospy.is_shutdown():
                pass

            self.MANUAL_CONTROL_ACTIVATED = True

            if self.SIMULATION:
                self.VELOCITY_REFERENCE_UPDATED = False
                while not self.VELOCITY_REFERENCE_UPDATED and not rospy.is_shutdown():
                    pass
            else:
                self.STATE_UPDATED = False
                while not self.STATE_UPDATED and not rospy.is_shutdown():
                    pass

            self.KEY = ''
            self.KEY = readchar.readchar()

            #####################################################
            # ######         Process User Input          ###### #
            #####################################################
            #####################################################
            #              Process Exploration Mode             #
            #####################################################
            #####################################################
            #     RESET BRAKE_ACTIVE Flag to exit braking       #
            #####################################################
            if (self.KEY == 'w' or self.KEY == 's' or self.KEY == 'a' or self.KEY == 'd' or self.KEY == 'r') \
                    and self.BRAKE_ACTIVE:
                self.PWM_VEL = 0
                self.PWM_STEER = 0
                self.BRAKE_ACTIVE = False
                self.STATE_UPDATED = False
                while not self.STATE_UPDATED:
                    pass
            else:
                pass
            #####################################################
            #        READ KEY for updating ACCELERATION         #
            #####################################################
            if self.KEY == 'w':
                self.PWM_VEL = self.PWM_VEL + self.STEP_VEL
            elif self.KEY == 's':
                self.PWM_VEL = self.PWM_VEL - self.STEP_VEL
            else:
                pass

            #####################################################
            #        READ KEY for updating STEERING             #
            #####################################################
            if self.KEY == 'a':
                self.PWM_STEER = self.PWM_STEER - self.STEP_STEER
            elif self.KEY == 'd':
                self.PWM_STEER = self.PWM_STEER + self.STEP_STEER
            else:
                pass

            #####################################################
            #        READ KEY for updating BRAKING              #
            #####################################################
            if self.KEY == 'b' or self.BRAKE_ACTIVE:
                if (self.KEY == 'b') and self.BRAKE_ACTIVE:               # Deactivate on second 'b' user input
                    self.BRAKE_ACTIVE = False
                    self.PWM_VEL = 0
                    self.PWM_STEER = 0
                else:
                    if self.BRAKE_ESC_IDLE:
                        self.BRAKE_ESC_IDLE = False
                        self.PWM_VEL = -20
                    elif self.BRAKE_ACTIVE:
                        pass                                            # # WAIT until control is set to zero before sending negative input
                    else:
                        if self.BRAKE_ESC_IDLE_COUNT < self.BRAKE_ESC_IDLE_MAX:
                            self.BRAKE_ESC_IDLE_COUNT += 1
                            self.PWM_VEL = 0
                            self.PWM_STEER = 0
                        else:
                            self.BRAKE_ESC_IDLE_COUNT = 0
                            self.BRAKE_ESC_IDLE = True
                        self.BRAKE_ACTIVE = True
            else:
                pass

            #####################################################
            #        READ KEY for updating RESET                #
            #####################################################
            if self.KEY == 'r':
                self.REINITIALIZE_ZERO_STEER = True
                self.PWM_VEL = 0
                self.PWM_STEER = 0
            else:
                pass

            #####################################################
            #           RESET EMERGENCY to False                #
            #####################################################
            if self.KEY == 'e':
                self.EMERGENCY_FLAG = False
            else:
                pass
            #####################################################
            #           Activate Testing (Simulation)           #
            #####################################################
            if self.KEY == 't':
                self.SIMULATION = not self.SIMULATION
            else:
                pass

            #####################################################
            #              Set EMERGENCY to True                #
            #####################################################
            if self.KEY == 'u':
                self.EMERGENCY_FLAG = True
            else:
                pass
            self.STATE_UPDATED = False
            self.MANUAL_CONTROL_ACTIVATED = True
            self.UPDATE_DISPLAY_REQUEST = True

        self.PWM_VEL_SAT = 0
        self.PWM_STEER_SAT = 0
        self.STATE_UPDATED = False
        self.SIMULATION = False
        while not self.STATE_UPDATED:
            pass
        self.EMERGENCY_FLAG = True
        self.MANUAL_CONTROL_ACTIVATED = False


#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN STARTED                  #\r")
        print("#######################################################\r")
        print("... initialize manual controller                       \r")
        time.sleep(1)

        f1vt18_manual_controller = VehicleManualController()
        thread_pub_pwm = ThreadedFunction(f1vt18_manual_controller.publish_pwm)
        thread_pub_vel = ThreadedFunction(f1vt18_manual_controller.publish_reference_velocity)
        thread_upd = ThreadedFunction(f1vt18_manual_controller.update_parameters)
        thread_disp = ThreadedFunction(f1vt18_manual_controller.display_interface)

        thread_pub_pwm.start()
        thread_pub_vel.start()
        thread_upd.start()
        thread_disp.start()

        f1vt18_manual_controller.read_user_input()
        rospy.set_param('/f1vt18/MANUAL_CONTROL_ACTIVATED', False)

    except rospy.ROSInterruptException:
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN INTERRUPTED              #\r")
        print("#######################################################\r")
        rospy.set_param('/f1vt18/MANUAL_CONTROL_ACTIVATED', False)
        pass
