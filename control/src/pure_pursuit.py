#!/usr/bin/env python
import os
import numpy
import rospy
import time
import threading
import readchar
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion as Q2E
from control.msg import pwm_interface
from control.msg import pwm_controller

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
#               Initialize Vehicle class            #
#####################################################
class PurePursuitController:
    def __init__(self):
        #####################################################
        #          Initialize Boolean States for System     #
        #####################################################
        self.PARAMETERS_INITIALIZED = False
        self.PARAMETER_UPDATED = False
        self.MAIN_INTERFACE_ACTIVE = False
        self.ESSENTIAL_TOPICS_ALIVE = False
        self.CONTROLLER_ACTIVE = False
        self.EXPLORATION_MAPPING_COMPLETE = False
        self.PATH_PUBLISHED = False
        self.CHECKPOINT_ARRAY_PUBLISHED = False

        self.READY_FOR_EXPLORATION = False
        self.READY_FOR_AUTONOMOUS = False

        self.MANUAL_CONTROL_ACTIVATED = False
        self.EXPLORATION_ACTIVATED = False
        self.AUTONOMOUS_RACING_ACTIVATED = False
        self.SAFETY_REGION_WATCHDOG_OK = False  # Lidar Safety Region to detect Crash or Obstacles

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
        self.EXPLORATION_EXT_MAP_REQUEST = False  # Path planning requests new map if controller reached goal and requests path
        self.EXPLORATION_EXT_PATH_REQUEST = False  # Controller requests new path if reached goal
        self.EXPLORATION_UPDATE_MAP = False  # is set true in initialization of exploration_mode_ini
        self.MAP_UPDATED_CYCLE = 0
        self.MAP_UPDATED_CYCLE_MAX = 2  # Requires 2 Map_updates for MAP_UPDATED becoming True (recommended to increase / robustness)
        self.MAP_UPDATED = False
        self.EXPLORATION_PATH_UPDATED = False
        self.EXPLORATION_EXPLORE_AHEAD = False
        self.EXPLORATION_LAP_COMPLETED = False
        self.EXPLORATION_PATH_GENERATION_ACTIVATED = False
        #####################################################
        #            External Boolean Parameters            #
        #####################################################
        self.GOAL_REACHED = False
        self.EXPLORATION_ACTIVATED = False
        self.AUTONOMOUS_RACING_ACTIVATED = False
        self.PARAMETERS_INITIALIZED = False
        self.PARAMETER_UPDATED = False
        self.PURE_PURSUIT_ACTIVE = False

        #####################################################
        #                Internal Parameters                #
        #####################################################
        self.wheel_base = 0.324
        self.CTRL = False
        self.STATE_UPDATED = False
        self.VELOCITY_CONTROLLER_ACTIVATED = True
        self.dt_interface = 0.5
        self.KEY = ''


        #####################################################
        #               Initialize State Variables          #
        #####################################################
        self.x = 0
        self.y = 0
        self.v = 0
        self.yaw = 0
        self.x_prev = 0
        self.y_prev = 0
        self.v_prev = 0
        self.yaw_prev = 0
        self.yaw_rate = 0

        #####################################################
        #         Initialize CTRL Tuning Variables          #
        #####################################################
        self.path_prev = 0                                      # compare if path topic has been updated
        self.lookahead_dist = 0.8
        self.traj_num_points = 0
        self.traj_x = []
        self.traj_y = []
        self.points_per_meter = 10

        #####################################################
        #               Initialize Output Variables         #
        #####################################################
        self.steering_saturation_threshold = 35                                     # 45deg max steering input
        self.steering = 0
        self.velocity = 0
        self.PWM_VEL_MIN = 0
        self.PWM_VEL_MAX = 12
        self.SAT_STEER_MIN = -81
        self.SAT_STEER_MAX = 81
        self.REF_VEL_PWM = 0
        self.REF_STEER_PWM = 0
        self.SYSTEM_GAIN = 1.7 / 15

        #####################################################
        #         Initialize ROS Messages/Parameters        #
        #####################################################
        self.time_prev = 0
        self.Ts = 1                                             # delta T to determine derivatives
        self.current_time = 0

        self.PUBLISH_RATE = 40
        self.PWM = pwm_interface()
        self.REF = pwm_controller()
        self.ODOM_feedback = Odometry()
        p = Pose()
        p.position.x = 0
        p.position.y = 0
        self.GOAL_WP = PoseArray()
        self.GOAL_WP.poses.append(p)

        #####################################################
        #         Initialize ROS Subscriber/Publisher       #
        #####################################################
        rospy.init_node('ctrl_node', anonymous=True)
        self.pub_pwm_vel = rospy.Publisher('pwm_interface', pwm_interface, queue_size=10)
        self.pub_ref_vel = rospy.Publisher('pwm_controller', pwm_controller, queue_size=10)
        self.pub_goal_wp = rospy.Publisher('/pure_pursuit_goal_pose', PoseArray, queue_size=1)
        self.rate = rospy.Rate(self.PUBLISH_RATE)

    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #               Initialize Subscriber               #
    #####################################################
    def initialize_subscriber(self):
        while not rospy.is_shutdown():
            while self.PURE_PURSUIT_ACTIVE:
                # rospy.Subscriber('qualisys/fb', Odometry, self.compute_control_input)             # for MOCAP experimental CTRL
                rospy.Subscriber('/odometry/filtered_map', Odometry, self.update_state)    # for Autonomous CTRL
                rospy.Subscriber('path_map', PoseArray, self.generate_trajectory)
                rospy.spin()

    #####################################################
    #              Update Vehicle States                #
    #####################################################
    def update_state(self, data):
        if self.PURE_PURSUIT_ACTIVE:
            self.Ts =  rospy.get_time() - self.time_prev
            self.x = data.pose.pose.position.x
            self.y = data.pose.pose.position.y
            self.yaw_prev = self.yaw
            _, _, self.yaw = Q2E([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
            self.yaw_rate = (self.yaw - self.yaw_prev) / self.Ts
            self.v = ((self.x - self.x_prev) ** 2 + (self.y - self.y_prev) ** 2) ** 0.5 / self.Ts

            self.x_prev = self.x
            self.y_prev = self.y
            self.yaw_prev = self.yaw
            self.time_prev = rospy.get_time()
            self.STATE_UPDATED = True
        else:
            pass

    #####################################################
    #               Generate Trajectory                 #
    #####################################################
    def generate_trajectory(self, points):
        wp_x = []
        wp_y = []
        for i in range(0, len(points.poses)):
            wp_x.append(points.poses[i].position.x)
            wp_y.append(points.poses[i].position.y)
        self.traj_x = []
        self.traj_y = []
        for index in range(1, len(wp_x)):
            dist_x = wp_x[index] - wp_x[index - 1]
            dist_y = wp_y[index] - wp_y[index - 1]

            len_temp = (dist_x ** 2 + dist_y ** 2) ** 0.5

            num_points = int(len_temp * float(self.points_per_meter))

            for num in range(0, num_points):
                temp_x = wp_x[index - 1] + num * dist_x / num_points
                self.traj_x.append(temp_x)
                temp_y = wp_y[index - 1] + num * dist_y / num_points
                self.traj_y.append(temp_y)

        self.traj_num_points = len(self.traj_x)
        self.path_prev = len(points.poses)
        print("GT:   PATH_UPDATED\r")

    #####################################################
    #             Execute Control Algorithm             #
    #####################################################
    def compute_control_input(self):
        while not rospy.is_shutdown():
            while not self.PURE_PURSUIT_ACTIVE and not rospy.is_shutdown():
                pass
            self.STATE_UPDATED = False
            while not self.STATE_UPDATED and not rospy.is_shutdown():
                pass
            print "PP:   Pure Pursuit running:\r"
            min_dist = 10000
            min_index = 1
            ref_index = 1
            if self.traj_num_points != 0:
                print "PP:   --> Trajectory given [x]\r"
                for i in range(0, self.traj_num_points):
                    temp_dist = numpy.sqrt((self.x - self.traj_x[i]) ** 2 + (self.y - self.traj_y[i]) ** 2)
                    if temp_dist <= min_dist:
                        min_dist = temp_dist
                        min_index = i

                self.nearest_point_index = min_index
                print('PP:   --> Distance to Trajectory Point: {0} \r'.format(min_dist))

                if min_dist > self.lookahead_dist:
                    print('PP:   --> Waypoint is at a good distance\r')
                    ref_index = self.nearest_point_index
                    self.velocity = 0.3
                    self.CTRL = True
                else:
                    print('PP:   --> closest Waypoint is too close\r')
                    ref_index = self.nearest_point_index + int(self.lookahead_dist * self.points_per_meter)
                    if self.AUTONOMOUS_RACING_ACTIVATED:
                        ref_index = ref_index % self.traj_num_points
                    else:
                        pass
                    print('PP:   Ref-Index: {0}  |  Length: {1}\r'.format(ref_index, self.traj_num_points))

                    if (ref_index > self.traj_num_points):
                        print('PP:   --> Waypoint is out of range for given trajectory --> STOP\r')
                        self.velocity = 0  # Include Braking control sequence here
                        self.CTRL = False
                        # self.GOAL_REACHED = True

                    else:
                        print('PP:   --> Waypoint is has been adapted successfully\r')
                        self.velocity = 0.3
                        self.CTRL = True

                if self.CTRL:
                    ref_state = [self.traj_x[ref_index], self.traj_y[ref_index]]
                    self.GOAL_WP.poses[0].position.x = ref_state[0]
                    self.GOAL_WP.poses[0].position.y = ref_state[1]
                    print('PP:   --> FROM\r' + str([self.x, self.y]))
                    print('PP:   --> GOTO\r' + str(ref_state))
                    dx = ref_state[0] - self.x
                    dy = ref_state[1] - self.y

                    y_goal = -numpy.sin(self.yaw) * dx + numpy.cos(self.yaw) * dy

                    distance_to_goal = numpy.sqrt(dx ** 2 + dy ** 2)
                    self.steering = numpy.arctan(self.wheel_base * 2 * y_goal / distance_to_goal ** 2) * 180 / numpy.pi
                    if abs(self.steering) > self.steering_saturation_threshold:
                        self.velocity = 0.1
                    else:
                        pass
                else:
                    print "PP:   --> No Control applied [-] --> Out of range condition\r"
                    self.velocity = 0
                    self.steering = 0
            else:
                print "PP:   --> No Control applied [-]\r"
                self.steering = 0
                self.velocity = 0

            self.REF_VEL_PWM = self.velocity * (1 / self.SYSTEM_GAIN)
            self.REF_STEER_PWM = (self.steering / self.steering_saturation_threshold) * 100 * (1 / self.SYSTEM_GAIN)

            self.REF_VEL_PWM = min(max(self.REF_VEL_PWM, self.PWM_VEL_MIN), self.PWM_VEL_MAX)
            self.REF_STEER_PWM = -min(max(self.REF_STEER_PWM, self.SAT_STEER_MIN), self.SAT_STEER_MAX)
            self.actuate()
            print('PP:   Steering in deg [PWM]: {0} [{1}] and Velocity in m/s [PWM]: {2} [{3}]\r'.format(self.steering, self.REF_STEER_PWM, self.velocity, self.REF_VEL_PWM))

    #####################################################
    #                PUBLISH ACTUATION                  #
    #####################################################
    def actuate(self):
        while not self.PURE_PURSUIT_ACTIVE:
            pass
        if self.VELOCITY_CONTROLLER_ACTIVATED:
            self.REF.velocity = self.velocity
            self.REF.steering = self.steering
            self.pub_ref_vel.publish(self.REF)
            print ("--> Sent ACTUATION signal (VEL_CONTROLLER - PID): \r")
            pass
        else:
            self.PWM.velocity = self.REF_VEL_PWM
            self.PWM.steering = self.REF_STEER_PWM
            self.pub_pwm_vel.publish(self.PWM)
            print ("--> Sent ACTUATION signal (VEL_CONTROLLER - ESC): \r")
        self.publish_goal_waypoint()


    #####################################################
    #              Publish Goal Waypoint                #
    #####################################################
    def publish_goal_waypoint(self):
        self.current_time = rospy.Time.now()
        self.GOAL_WP.header.stamp = self.current_time
        self.GOAL_WP.header.frame_id = 'map'
        self.pub_goal_wp.publish(self.GOAL_WP)

    #####################################################
    #                   SET PARAMETER                   #
    #####################################################
    def set_f1vt18_parameter(self, parameter_name, value=False):
        resolved_global_name = "/f1vt18/" + parameter_name
        rospy.set_param(resolved_global_name, value)

    #####################################################
    #                   GET PARAMETER                   #
    #####################################################
    def get_f1vt18_parameter(self, parameter_name, value=False):
        resolved_global_name = "/f1vt18/" + parameter_name
        return rospy.get_param(resolved_global_name, value)

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def update_parameters(self):
        while not rospy.is_shutdown():
            while not self.PARAMETERS_INITIALIZED:
                self.PARAMETERS_INITIALIZED = self.get_f1vt18_parameter("PARAMETERS_INITIALIZED")

            #####################################################
            #                   GET PARAMETER                   #
            #####################################################
            self.AUTONOMOUS_RACING_ACTIVATED = self.get_f1vt18_parameter("AUTONOMOUS_RACING_ACTIVATED")

            #####################################################
            #                   SET PARAMETER                   #
            #####################################################
            self.set_f1vt18_parameter("PURE_PURSUIT_ACTIVE",self.PURE_PURSUIT_ACTIVE)

            self.PARAMETER_UPDATED = True
        self.PURE_PURSUIT_ACTIVE = False

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def display_interface(self):
        #####################################################
        #               WHILE LOOP for User Input           #
        #####################################################
        while not rospy.is_shutdown():

            print("#######################################################\r")
            print("       Waiting for PARAMETERS TO BE INITIALIZED        \r")
            print("-------------------------------------------------------\r")
            print('Press q if frozen...                                   \r')
            while not self.PARAMETERS_INITIALIZED and not rospy.is_shutdown():
                pass

            print("#######################################################\r")
            print(" Waiting for MAIN INTERFACE TO START AUTONOMOUS RACING \r")
            print("-------------------------------------------------------\r")
            print('Press q if frozen...                                   \r')
            while not self.AUTONOMOUS_RACING_ACTIVATED and not rospy.is_shutdown():
                pass

            self.PURE_PURSUIT_ACTIVE = True

            print("#######################################################\r")
            print("       Waiting for STATE TO BE UPDATED                 \r")
            print("-------------------------------------------------------\r")
            print('Press q if frozen...                                   \r')
            while not self.STATE_UPDATED and not rospy.is_shutdown():
                pass

            #####################################################
            #     UPDATE PARAMETERS FROM ROS PARAMETER SERVER   #
            #####################################################
            self.PARAMETER_UPDATED = False
            print("#######################################################\r")
            print("       Waiting for PARAMETERS TO BE UPDATED            \r")
            print("-------------------------------------------------------\r")
            print('Press q if frozen...                                   \r')
            while not self.PARAMETER_UPDATED and not rospy.is_shutdown():
                pass

            #####################################################
            #                 Show User Control Keys            #
            #####################################################
            print("######################################## -- Active --   \r")
            print("-----------------------------------------------------   \r")
            print('VELOCITY_CONTROLLER_ACTIVE:[      v     ]  -  {0} -    \r'.format(self.VELOCITY_CONTROLLER_ACTIVATED))
            print('QUIT:              (switch)[      q     ]   \r')
            print('########################################------------   \r')
            print('VEL: {0} | Steer: {1}\r'.format(self.velocity, self.steering))
            print("-----------------------------------------------------   \r")
            print('User-Input: {0}\r'.format(self.KEY))
            time.sleep(self.dt_interface)
            #self.cls()
        self.PURE_PURSUIT_ACTIVE = False

    #####################################################
    #            Define User Input Function             #
    #####################################################
    def read_user_input(self):
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
            #             TOOGLE VELOCITY CONTROLLER            #
            #####################################################
            if self.KEY == 'v':
                self.VELOCITY_CONTROLLER_ACTIVATED = True
            else:
                pass
        self.PURE_PURSUIT_ACTIVE = False


if __name__ == '__main__':
    try:
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN STARTED                  #\r")
        print("#######################################################\r")
        print("... initialize vehicle object                          \r")
        time.sleep(1)
        pure_pursuit = PurePursuitController()
        Thread_Subscriber = ThreadedFunction(pure_pursuit.initialize_subscriber)
        Thread_Parameter = ThreadedFunction(pure_pursuit.update_parameters)
        Thread_Controller = ThreadedFunction(pure_pursuit.compute_control_input)
        Thread_Display = ThreadedFunction(pure_pursuit.display_interface)

        Thread_Subscriber.start()
        Thread_Parameter.start()
        Thread_Controller.start()
        Thread_Display.start()

        pure_pursuit.read_user_input()
        rospy.set_param('/f1vt18/PURE_PURSUIT_ACTIVE', False)
    except rospy.ROSInterruptException:
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN INTERRUPTED              #\r")
        print("#######################################################\r")
        rospy.set_param('/f1vt18/PURE_PURSUIT_ACTIVE', False)
        pass
        pass
