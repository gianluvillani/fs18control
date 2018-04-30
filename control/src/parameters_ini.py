#!/usr/bin/env python

#####################################################
#####################################################
#####################################################
#                                                   #
#             Initialize Parameters once            #
#                                                   #
#####################################################
#                                                   #
#     - defines a specific list of parameters
#                                                   #
# written by Philipp Rothenhaeusler - phirot@kth.se #
#                                                   #
#####################################################
#####################################################
#####################################################
#####################################################
import os
import rospy
import time
import threading


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
#      Define Parameter Intializer Class            #
#####################################################
class Initializer:

    def __init__(self):
        self.PARAMETERS_INITIALIZED = False
        self.MAIN_INTERFACE_STARTED = False

    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #              INITIALIZE PARAMETER                 #
    #####################################################
    def initialize_f1vt18_parameter(self, parameter_name, value=False):
        resolved_global_name = "/f1vt18/" + parameter_name
        rospy.set_param(resolved_global_name, value)
        print('Initialized       {0} : {1}\r'.format(resolved_global_name, value))

    #####################################################
    #               UPDATE PARAMETER                    #
    #####################################################
    def update_f1vt18_parameter(self, parameter_name, value=False):
        resolved_global_name = "/f1vt18/" + parameter_name
        rospy.set_param(resolved_global_name, value)
        print('Updated           {0} : {1}\r'.format(resolved_global_name, value))

    #####################################################
    #                   GET PARAMETER                   #
    #####################################################
    def get_f1vt18_parameter(self, parameter_name, value=False):
        resolved_global_name = "/f1vt18/" + parameter_name
        print('Received          {0} : {1}\r'.format(resolved_global_name, value))
        return rospy.get_param(resolved_global_name, value)

    #####################################################
    #                   SET PARAMETER                   #
    #####################################################
    def set_f1vt18_parameter(self, parameter_name, value):
        resolved_global_name = "/f1vt18/" + parameter_name
        print('Set               {0} : {1}\r'.format(resolved_global_name, value))

    #####################################################
    #                Set Parameters                     #
    #####################################################
    def initialize_parameters(self):
        self.cls()
        #####################################################
        # ##             Start Initializaion             ## #
        #####################################################
        #####################################################
        #              Global Logic Parameters              #
        #####################################################
        self.initialize_f1vt18_parameter("MAIN_INTERFACE_STARTED")
        self.initialize_f1vt18_parameter("PARAMETERS_INITIALIZED")
        self.initialize_f1vt18_parameter("PARAMETER_UPDATED")
        self.initialize_f1vt18_parameter("ESSENTIAL_TOPICS_ALIVE")
        self.initialize_f1vt18_parameter("PURE_PURSUIT_ACTIVE")
        self.initialize_f1vt18_parameter("EXPLORATION_MAPPING_COMPLETE")
        self.initialize_f1vt18_parameter("PATH_PUBLISHED")
        self.initialize_f1vt18_parameter("CHECKPOINT_ARRAY_PUBLISHED")
        self.initialize_f1vt18_parameter("READY_FOR_EXPLORATION")
        self.initialize_f1vt18_parameter("READY_FOR_AUTONOMOUS")
        self.initialize_f1vt18_parameter("EXPLORATION_ACTIVATED")
        self.initialize_f1vt18_parameter("AUTONOMOUS_RACING_ACTIVATED")
        self.initialize_f1vt18_parameter("SAFETY_REGION_WATCHDOG_OK")
        self.initialize_f1vt18_parameter("EMERGENCY_FLAG")

        self.initialize_f1vt18_parameter("EXPLORATION_EXT_MAP_REQUEST")
        self.initialize_f1vt18_parameter("EXPLORATION_EXT_PATH_REQUEST")
        self.initialize_f1vt18_parameter("EXPLORATION_UPDATE_MAP")
        self.initialize_f1vt18_parameter("MAP_UPDATED_CYCLE")
        self.initialize_f1vt18_parameter("MAP_UPDATED_CYCLE_MAX")
        self.initialize_f1vt18_parameter("MAP_UPDATED")
        self.initialize_f1vt18_parameter("EXPLORATION_PATH_UPDATED")
        self.initialize_f1vt18_parameter("EXPLORATION_EXPLORE_AHEAD")
        self.initialize_f1vt18_parameter("EXPLORATION_LOCAL_GOAL_POSE_REACHED")
        self.initialize_f1vt18_parameter("EXPLORATION_PATH_GENERATION_ACTIVATED")


        #####################################################
        #     Vehicle Dimension and Sensor Parameters       #
        #####################################################
        self.set_f1vt18_parameter("WHEEL_BASE", 0.324)
        self.set_f1vt18_parameter("WHEEL_RADIUS", 0.08/2)
        self.set_f1vt18_parameter("ENC_TS", 0.08/2)
        self.set_f1vt18_parameter("ENC_T_PER_REV", 8)
        self.set_f1vt18_parameter("ENC_REINITIALIZE_PARAMETERS", False)

        #####################################################
        #     Controller Parameters and Saturation          #
        #####################################################
        self.set_f1vt18_parameter("SAT_VEL_MIN", -100)                   # Necessary for Active Braking
        self.set_f1vt18_parameter("SAT_VEL_MAX", 100)
        self.set_f1vt18_parameter("PWM_STEER_OFFSET", 9)
        self.set_f1vt18_parameter("SAT_STEER_MIN", -81)
        self.set_f1vt18_parameter("SAT_STEER_MAX", 81)
        self.set_f1vt18_parameter("STEP_VEL", 5)
        self.set_f1vt18_parameter("STEP_STEER", 15)

        #####################################################
        #              ROS Parameters                       #
        #####################################################
        # PUBLISH_RATE_PWM
        #set_f1vt18_parameter("SAT_VEL_MIN", -100)  # Necessary for Active Braking
        #set_f1vt18_parameter("SAT_VEL_MAX", 100)
        #set_f1vt18_parameter("PWM_STEER_OFFSET", 9)
        #set_f1vt18_parameter("SAT_STEER_MIN", -81)
        #set_f1vt18_parameter("SAT_STEER_MAX", 81)
        #set_f1vt18_parameter("STEP_VEL", 5)
        #set_f1vt18_parameter("STEP_STEER", 15)

        print("                                                       \r")
        print("#######################################################\r")
        print("PARAMETERS ARE INITIALIZED now ...                     \r")
        print("-------------------------------------------------------\r")
        self.PARAMETERS_INITIALIZED = True
        self.update_f1vt18_parameter("PARAMETERS_INITIALIZED", self.PARAMETERS_INITIALIZED)

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def display_interface(self):
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN STARTED                  #\r")
        print("#######################################################\r")
        print("Trying to initialize parameters now...                 \r")
        print("                                                       \r")
        time.sleep(2)
        while not self.PARAMETERS_INITIALIZED and not rospy.is_shutdown():
            self.MAIN_INTERFACE_STARTED = self.get_f1vt18_parameter("MAIN_INTERFACE_STARTED")
            if not self.MAIN_INTERFACE_STARTED:
                print("#######################################################\r")
                print("#       Main Interface has not been started yet       #\r")
                print("#######################################################\r")
                print('/MAIN_INTERFACE_STARTED : {0}\r'.format(self.MAIN_INTERFACE_STARTED))
            else:
                print("#######################################################\r")
                print("#          Main Interface has been started            #\r")
                print("#######################################################\r")
                print('/MAIN_INTERFACE_STARTED : {0}\r'.format(self.MAIN_INTERFACE_STARTED))
                print("#######################################################\r")
                print('Initializing parameters now ...\r')
                print("-------------------------------------------------------\r")
                self.initialize_parameters()
            self.cls()

        if self.PARAMETERS_INITIALIZED:
            print("#######################################################\r")
            print("#           Successfully initialized                  #\r")
            print("#######################################################\r")

        else:
            print("#######################################################\r")
            print("#           Not successfully initialized              #\r")
            print("#######################################################\r")
        print("#... can be closed now                                #\r")
        print("#-- (idle 3600s until with close CTRL + C)            #\r")
        print("#######################################################\r")
        time.sleep(5)


#####################################################
#                   Main Loop                       #
#####################################################
if __name__ == "__main__":
    try:
        Vehicle_Initializer = Initializer()
        Vehicle_Initializer.display_interface()
    except rospy.ROSInterruptException:
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN INTERRUPTED              #\r")
        print("#######################################################\r")
        pass
