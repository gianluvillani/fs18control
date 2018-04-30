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
#       using ros service to request initial pose   #
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
from std_msgs.msg import Bool
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
        self.PARAMETERS_INITIALIZED = False
        self.PARAMETER_UPDATED = False
        self.MAIN_INTERFACE_ACTIVE = False
        self.ESSENTIAL_TOPICS_ALIVE = False
        self.CONTROLLER_ACTIVE = False
        self.EXPLORATION_MAPPING_COMPLETE = False
        self.PATH_PUBLISHED = False
        self.CHECKPOINT_ARRAY_PUBLISHED = False
        self.WATCHDOG_TRIGGER = False
        self.WATCHDOG_TRIGGER_RESET = False

        self.READY_FOR_EXPLORATION = False
        self.READY_FOR_AUTONOMOUS = False

        self.MANUAL_CONTROL_ACTIVATED = False
        self.EXPLORATION_ACTIVATED = False
        self.AUTONOMOUS_RACING_ACTIVATED = False
        self.SAFETY_REGION_WATCHDOG_OK = False                  # Lidar Safety Region to detect Crash or Obstacles

        self.EMERGENCY_FLAG = False

        #####################################################
        #          Initialize Locks for Threaded Access     #
        #####################################################
        self.PUBLISH_ACTIVE = False
        self.KEY_READ_ACTIVE = False

        #####################################################
        #          Initialize Variables for User Input      #
        #####################################################
        self.KEY = ''

        #####################################################
        #    Initialize Variables for EXPLORATION MODE      #
        #####################################################
        self.EXPLORATION_EXT_MAP_REQUEST = False                # Path planning requests new map if controller reached goal and requests path
        self.EXPLORATION_EXT_PATH_REQUEST = False               # Controller requests new path if reached goal
        self.EXPLORATION_UPDATE_MAP = False                     # is set true in initialization of exploration_mode_ini
        self.MAP_UPDATED_CYCLE = 0
        self.MAP_UPDATED_CYCLE_MAX = 2                          # Requires 2 Map_updates for MAP_UPDATED becoming True (recommended to increase / robustness)
        self.MAP_UPDATED = False
        self.EXPLORATION_PATH_UPDATED = False
        self.EXPLORATION_EXPLORE_AHEAD = False
        self.EXPLORATION_LAP_COMPLETED = False
        self.EXPLORATION_PATH_GENERATION_ACTIVATED = False

        #####################################################
        #               Initialize Publisher                #
        #####################################################
        self.EMERGENCY_MSG = Bool()
        self.EMERGENCY_MSG.data = False

        self.STATE_UPDATED = False
        self.dt_interface = 0.4
        self.PUBLISH_RATE = 40
        rospy.init_node('main_interface_node', anonymous=True)
        self.pub_emergency = rospy.Publisher('pwm_interface_emergency', Bool, queue_size=1)
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
                self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.MAIN_INTERFACE_STARTED)
                pass

            self.WATCHDOG_TRIGGER = self.get_f1vt18_parameter("WATCHDOG_TRIGGER")
            self.set_f1vt18_parameter("WATCHDOG_TRIGGER_RESET", self.WATCHDOG_TRIGGER_RESET)
            if self.WATCHDOG_TRIGGER:
                if self.WATCHDOG_TRIGGER and not self.WATCHDOG_TRIGGER_RESET:
                    self.set_f1vt18_parameter("WATCHDOG_TRIGGER_RESET", self.WATCHDOG_TRIGGER_RESET)
                    self.set_f1vt18_parameter("EMERGENCY_FLAG", self.EMERGENCY_FLAG)
                    self.EMERGENCY_FLAG = True
                elif self.WATCHDOG_TRIGGER and self.WATCHDOG_TRIGGER_RESET:
                    while self.WATCHDOG_TRIGGER:
                        self.WATCHDOG_TRIGGER = self.get_f1vt18_parameter("WATCHDOG_TRIGGER")
                    self.set_f1vt18_parameter("WATCHDOG_TRIGGER_RESET", self.WATCHDOG_TRIGGER_RESET)
                    self.set_f1vt18_parameter("EMERGENCY_FLAG", self.EMERGENCY_FLAG)
                    pass
            else:
                pass

            self.READY_FOR_EXPLORATION = self.ESSENTIAL_TOPICS_ALIVE and self.CONTROLLER_ACTIVE
            self.READY_FOR_AUTONOMOUS = self.EXPLORATION_MAPPING_COMPLETE and self.ESSENTIAL_TOPICS_ALIVE and \
                                            self.CHECKPOINT_ARRAY_PUBLISHED and not self.EMERGENCY_FLAG

            self.READY_FOR_AUTONOMOUS = self.EXPLORATION_MAPPING_COMPLETE and self.ESSENTIAL_TOPICS_ALIVE and \
                                        self.CHECKPOINT_ARRAY_PUBLISHED and self.AUTONOMOUS_RACING_ACTIVATED and \
                                        self.SAFETY_REGION_WATCHDOG_OK and not self.EMERGENCY_FLAG

            self.set_f1vt18_parameter("EMERGENCY_FLAG", self.EMERGENCY_FLAG)
            self.set_f1vt18_parameter("EXPLORATION_ACTIVATED", self.EXPLORATION_ACTIVATED)
            self.set_f1vt18_parameter("AUTONOMOUS_RACING_ACTIVATED", self.AUTONOMOUS_RACING_ACTIVATED)
            self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.MAIN_INTERFACE_STARTED)



            #####################################################
            #         MANUAL_CONTROL_PRIORITY                   #
            #####################################################
            if self.MANUAL_CONTROL_ACTIVATED:
                self.EXPLORATION_ACTIVATED = False
                self.AUTONOMOUS_RACING_ACTIVATED = False
            else:
                pass
            self.PARAMETER_UPDATED = True
            self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.MAIN_INTERFACE_STARTED)
        self.MAIN_INTERFACE_STARTED = False
        self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.MAIN_INTERFACE_STARTED)


    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #             Initialize EXPLORATION Mode           #
    #####################################################
    def exploration_mode_init(self):
        rospy.Subscriber('/map', OccupancyGrid, self.exploration_mode_new_map)  # for Autonomous CTRL
        rospy.Subscriber('/path', PoseArray, self.exploration_mode_new_path)
        if self.EXPLORATION_LAP_COMPLETED:
            self.EXPLORATION_MAPPING_COMPLETE = True
            self.EXPLORATION_UPDATE_MAP = True
        rospy.spin()

    #####################################################
    #             Initialize EXPLORATION Mode           #
    #####################################################
    def exploration_mode_new_map(self, data):
        if self.EXPLORATION_UPDATE_MAP:
            self.MAP_UPDATED_CYCLE = self.MAP_UPDATED_CYCLE + 1
            if self.MAP_UPDATED_CYCLE >= self.MAP_UPDATED_CYCLE_MAX:
                self.MAP_UPDATED_CYCLE = 0
                self.MAP_UPDATED = True
                self.exploration_mode_find_path()
        else:
            pass

    #####################################################
    #             Initialize EXPLORATION Mode           #
    #####################################################
    def exploration_mode_find_path(self):
        self.EXPLORATION_PATH_GENERATION_ACTIVATED = True
        self.EXPLORATION_UPDATE_MAP = False
        self.MAP_UPDATED = False

    #####################################################
    #             Initialize EXPLORATION Mode           #
    #####################################################
    def exploration_mode_new_path(self, data):
        if self.EXPLORATION_PATH_GENERATION_ACTIVATED:
            self.EXPLORATION_PATH_UPDATED = True
            self.EXPLORATION_EXPLORE_AHEAD = True
        else:
            pass

    #####################################################
    #          Publish STOP to PWM_INTERFACE            #
    #####################################################
    def publish_emergency(self):
        while not rospy.is_shutdown():
            self.PUBLISH_ACTIVE = True
            self.EMERGENCY_MSG.data = self.EMERGENCY_FLAG
            self.pub_emergency.publish(self.EMERGENCY_MSG)
            self.PUBLISH_ACTIVE = False
            self.STATE_UPDATED = True
            self.rate.sleep()

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def display_interface(self):
        #####################################################
        #               WHILE LOOP for User Input           #
        #####################################################
        while not rospy.is_shutdown():

            print("####################################\r")
            print(" Waiting for PARAMETERS INI         \r")
            print("------------------------------------\r")
            print('Press q if frozen...                \r')
            while not self.PARAMETERS_INITIALIZED and not rospy.is_shutdown():
                time.sleep(2)
                pass

            #####################################################
            #     UPDATE PARAMETERS FROM ROS PARAMETER SERVER   #
            #####################################################
            self.PARAMETER_UPDATED = False
            print("####################################\r")
            print("  Waiting for PARAMETERS UPDATE     \r")
            print("------------------------------------\r")
            print('Press q if frozen...                \r')
            while not self.PARAMETER_UPDATED and not rospy.is_shutdown():
                pass

            print("####################################\r")
            print(" Waiting for STATE UPDATE           \r")
            print("------------------------------------\r")
            print('Press q if frozen...                \r')
            while not self.STATE_UPDATED and not rospy.is_shutdown():
                pass


            self.cls()
            #####################################################
            # ###     Print Control Interface to Terminal   ### #
            #####################################################
            #####################################################
            #                 Show current mode                 #
            #####################################################
            print("##################################\r")
            print("Mode: [x]= active | [-]= inactive \r")
            print("----------------------------------\r")
            if self.MANUAL_CONTROL_ACTIVATED:
                print("[x] MANUAL_CONTROL_ACTIVATED      \r")
            else:
                print("[-] MANUAL_CONTROL_ACTIVATED      \r")
            if self.EXPLORATION_ACTIVATED:
                print("[x] EXPLORATION_ACTIVATED         \r")
            else:
                print("[-] EXPLORATION_ACTIVATED         \r")
            if self.AUTONOMOUS_RACING_ACTIVATED:
                print("[x] AUTONOMOUS_RACING_ACTIVATED   \r")
            else:
                print("[-] AUTONOMOUS_RACING_ACTIVATED   \r")
            print("----------------------------------\r")



            #####################################################
            #                 Show current status flags         #
            #####################################################
            print("##################################\r")
            print("Status: [x] = TRUE | [-] = FALSE  \r")
            print("----------------------------------\r")
            if self.MANUAL_CONTROL_ACTIVATED:
                print("[x] ESSENTIAL_TOPICS_ALIVE        \r")
            else:
                print("[-] ESSENTIAL_TOPICS_ALIVE        \r")
            if self.EXPLORATION_MAPPING_COMPLETE:
                print("[x] MAPPING_COMPLETE              \r")
            else:
                print("[-] MAPPING_COMPLETE              \r")
            if self.ESSENTIAL_TOPICS_ALIVE:
                print("[x] ESSENTIAL_TOPICS_ALIVE        \r")
            else:
                print("[-] ESSENTIAL_TOPICS_ALIVE        \r")
            if self.EMERGENCY_FLAG:
                print("[x] EMERGENCY_FLAG                \r")
            else:
                print("[-] EMERGENCY_FLAG                \r")
            print("----------------------------------\r")


            #####################################################
            #                 Show User Control Keys            #
            #####################################################
            print("##################################\r")
            print("Enter Key for Control Interface:  \r")
            print("----------------------------------\r")
            print("# AUTONOMOUS (toogle): ['a']      #\r")
            print("# EMERGENCY (toogle):  ['e']      #\r")
            if self.READY_FOR_EXPLORATION:
                print("# START EXPLORATION:   ['x']      #\r")
            else:
                pass

            if self.READY_FOR_AUTONOMOUS:
                print("# START AUTONOMOUS:    ['a']      #\r")
            else:
                pass
            ''' 
            if self.AUTONOMOUS_RACING_ACTIVATED and not self.SAFETY_REGION_WATCHDOG_OK and self.ESSENTIAL_TOPICS_ALIVE:
                print("# REINITIALIZE:(LAST)     [     'i'    ]              #\r")
            elif self.AUTONOMOUS_RACING_ACTIVATED and not self.SAFETY_REGION_WATCHDOG_OK and not self.ESSENTIAL_TOPICS_ALIVE:
                print("# REINITIALIZE:(START)     [     '--'    ]             #\r")
            else:
                pass
                '''
            print("# QUIT :               ['q']      #\r")
            print("###################################\r")
            time.sleep(self.dt_interface)
            self.cls()




    #####################################################
    #            Define User Input Function             #
    #####################################################
    def read_user_input(self):
        self.MAIN_INTERFACE_STARTED = True

        while not self.PARAMETERS_INITIALIZED:
            pass

        while self.KEY != 'q' and not rospy.is_shutdown():

            self.PARAMETER_UPDATED = False
            while not self.PARAMETER_UPDATED:
                pass

            self.STATE_UPDATED = False
            while not self.STATE_UPDATED:
                pass

            self.KEY = readchar.readchar()

            #####################################################
            # ######         Process User Input          ###### #
            #####################################################

            #####################################################
            #              Process Autonomous Mode              #
            #####################################################
            if self.KEY == 'a':
                self.AUTONOMOUS_RACING_ACTIVATED = not self.AUTONOMOUS_RACING_ACTIVATED
            else:
                pass

            #####################################################
            #              TOOGLE EMERGENCY to True             #
            #####################################################
            if self.KEY == 'e':
                self.EMERGENCY_FLAG = not self.EMERGENCY_FLAG
                if self.WATCHDOG_TRIGGER:
                    self.WATCHDOG_TRIGGER_RESET = True
                    self.PARAMETER_UPDATED = False
                    while not self.PARAMETER_UPDATED and self.WATCHDOG_TRIGGER:
                        pass
                    self.WATCHDOG_TRIGGER_RESET = False
                else:
                    pass
            else:
                pass

            #####################################################
            #                   Quit Program              #
            #####################################################
            if self.KEY == 'q':
                self.EMERGENCY_FLAG = True
            else:
                pass
            ''' 
            #####################################################
            #              Process Exploration Mode             #
            #####################################################
            if self.KEY == 'x':
                self.EXPLORATION_ACTIVATED = True
                self.EXPLORATION_UPDATE_MAP = True
                self.exploration_mode_init()
            else:
                pass
            '''
            self.STATE_UPDATED = False

        self.EMERGENCY_FLAG = True
        self.MAIN_INTERFACE_ACTIVE = False


#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        print("----------------------------------\r")
        print("##################################\r")
        print("#  PROGRAM HAS BEEN STARTED      #\r")
        print("##################################\r")
        print("... initialize vehicle object     \r")
        time.sleep(1)
        f1vt18 = Vehicle()

        Thread_Publish_Emergency = ThreadedFunction(f1vt18.publish_emergency)
        Thread_Update_Parameters = ThreadedFunction(f1vt18.update_parameters)
        Thread_Display_Interface = ThreadedFunction(f1vt18.display_interface)

        Thread_Publish_Emergency.start()
        Thread_Update_Parameters.start()
        Thread_Display_Interface.start()

        f1vt18.read_user_input()
        rospy.set_param('/f1vt18/MAIN_INTERFACE_STARTED', False)
        rospy.set_param('/f1vt18/PARAMETERS_INITIALIZED', False)

    except rospy.ROSInterruptException:
        print("##################################\r")
        print("# PROGRAM HAS BEEN INTERRUPTED   #\r")
        print("##################################\r")
        rospy.set_param('/f1vt18/MAIN_INTERFACE_STARTED', False)
        rospy.set_param('/f1vt18/PARAMETERS_INITIALIZED', False)
        pass
