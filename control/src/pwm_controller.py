#!/usr/bin/env python

import os
import time
import rospy
import osqp
import numpy
import numpy as np
import scipy as sp
import scipy.sparse as sparse
import scipy.integrate as Integration
import readchar
import threading
from control.msg import pwm_controller
from control.msg import pwm_interface
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import euler_from_quaternion as EFQ


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


class VelocityController:
    #####################################################
    #          Initialize Object                        #
    #####################################################
    def __init__(self):
        #####################################################
        #               Initialize Variables                #
        #####################################################
        self.PWM_CONTROLLER_STARTED = False

        #EXTERNAL PARAMETERS
        self.CONTROLLER_ACTIVE = False
        self.FEEDBACK_ERROR = False
        self.VEHICLE_SLIPPING = False
        self.VEHICLE_SLIP_DETECTION_ACTIVE = False
        self.VEHICLE_ACTIVE_BRAKING_ACTIVE = False
        self.VEHICLE_TRACTION_CONTROL_ACTIVE = False
        self.PARAMETERS_INITIALIZED = False

        #INTERNAL PARAMETERS
        self.SUBSCRIBER_ACTIVE = True
        self.PUBLISHER_ACTIVE = True
        self.CONTROL_CHOICE = 'PID'             # Choose between 'PID' or 'MPC'
        self.FEEDBACK_ENC_ACTIVATED = True
        self.FEEDBACK_ODOM_ACTIVATED = False
        self.FEEDBACK_OBS_ACTIVATED = False

        self.FEEDBACK_ENC_AVG_INI = False

        self.PARAMETER_UPDATED = False
        self.REFERENCE_STATE_UPDATED = False
        self.FEEDBACK_ODOM_UPDATED = False
        self.FEEDBACK_OBS_UPDATED = False
        self.FEEDBACK_ENC_UPDATED = False
        self.SLIP_UPDATED = False
        self.CONTROL_UPDATED = False

        self.FEEDBACK_ENC_PUBLISH_RATE = 380
        self.FEEDBACK_ENC_AVG_N_MAX = int(2. / (1. / 380))
        self.FEEDBACK_ENC_AVG_n = 0
        self.FEEDBACK_ENC_prev = 0
        self.FEEDBACK_ENC_sum = 0

        self.PUBLISH_RATE_PWM = 40
        self.PUBLISHER_ODOM_VEL_ACTIVE = False
        self.DEACTIVATE_PUBLISH_ON_IDLE = False
        self.CONSIDER_NONLINEARITY = True
        self.N_IDLE = 0
        self.N_IDLE_THRESHOLD = 300

        self.PWM_CONTROLLER_STARTED = False
        self.COMPUTE_CONTROL_SIGNAL = False
        self.PUBLISH_CONTROL_SIGNAL = False
        self.CONTROL_OUTPUT_UPDATED = False
        self.PUBLISH_ACTIVE = False
        self.WRITE_FLAG = False
        self.BRAKE_FLAG = False
        self.BRAKE_ACTIVE = False
        self.PID_INITIALIZED = False

        #CONTROL CONSTANTS AND TUNING PARAMETERS
        self.PWM_velocity_control_signal = 0        # in %  Output
        self.PWM_steering_control_signal = 0
        self.reference_velocity = 0                 # in m/s
        self.reference_steering = 0
        self.feedback_velocity_odom = 0
        self.feedback_velocity_enc = 0
        self.feedback_velocity_obs = 0
        self.feedback_velocity = 0
        self.vehicle_slip = 0
        self.SAT_VEL_MIN = -100
        self.SAT_VEL_MAX = 100

        self.PWM_STEER_OFFSET = 9
        self.SAT_STEER_MIN = -81
        self.SAT_STEER_MAX = 81

        self.STEP_VEL = 30
        self.STEP_STEER = 4

        self.KEY = ''

        self.PWM_VEL = 0
        self.PWM_VEL_SAT = 0
        self.PWM_STEER = 0
        self.PWM_STEER_SAT = 0

        self.PWM_NL_SAT_MIN = -6
        self.PMW_NL_SAT_MAX = 12
        self.dt_interface = 0.5

        #ODOMETRY FEEDBACK
        self.fb_odom_x = 0
        self.fb_odom_y = 0
        self.fb_odom_y_prev = 0
        self.fb_odom_x_prev = 0

        #TIMERS
        self.fb_odom_timer = 0
        self.dt_interface = 0.5


        #PID_CONTROL_PARAMETERS (initialization in controller function)
        self.P = 0
        self.I = 0
        self.D = 0
        self.ctrl_output = 0
        self.e_int = 0
        self.e_d = 0
        self.e_p = 0
        self.AWU = False
        self.ctrl_AWU_threshold = 100           # MAXIMUM CONTROL (SATURATION OF CONTROL EFFECT)
        self.t_prev = 0
        self.dt = 0
        self.e = 0
        self.e_prev = 0



        # MPC CONTROL PARAMETERS
        # System Dynamics
        self.u = 0
        self.y = 0
        self.A = -2/3
        self.PHI = numpy.exp(self.A*(1./self.PUBLISH_RATE_PWM))
        self.B = 1.7/15
        self.C = 1
        self.D = 0
        self.B_INT = Integration.quad(lambda s: self.B*self.state_transition_system(s), 0, 1./self.PUBLISH_RATE_PWM)
        self.THETA = self.B_INT[0]
        self.dt = 0
        self.t_prev = 0

        ''' 
        # Discrete time model of a quadcopter
        self.Ad = sparse.csc_matrix([self.PHI])
        self.Bd = sparse.csc_matrix([self.THETA])
        [self.nx, self.nu] = self.Bd.shape

        # Constraints
        u0 = 0
        self.u_min = np.array([-100])
        self.u_max = np.array([100])
        self.x_min = np.array([-3])
        self.x_max = np.array([18])           # 60 kph --> 16,667 m/s

        # Objective function
        self.Q = sparse.diags([1])
        self.QN = self.Q
        self.R = 0.1 * sparse.eye(1)

        # Initial and reference states
        self.x0 = np.zeros(1)
        self.xr = np.array([0])

        # Prediction horizon
        self.N = 10

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        self.P = sparse.block_diag([sparse.kron(sparse.eye(self.N), self.Q), self.QN, sparse.kron(sparse.eye(self.N), self.R)])

        # - linear objective
        self.q = np.hstack([np.kron(np.ones(self.N), -self.Q.dot(self.xr)), -self.QN.dot(self.xr), np.zeros(self.N * self.nu)])

        # - linear dynamics
        self.Ax = sparse.kron(sparse.eye(self.N + 1), -sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N + 1, k=-1), self.Ad)
        self.Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), self.Bd)
        self.Aeq = sparse.hstack([self.Ax, self.Bu])
        self.leq = np.hstack([-self.x0, np.zeros(self.N * self.nx)])
        self.ueq = self.leq

        # - input and state constraints
        self.Aineq = sparse.eye((self.N + 1) * self.nx + self.N * self.nu)
        self.lineq = np.hstack([np.kron(np.ones(self.N + 1), self.x_min), np.kron(np.ones(self.N), self.u_min)])
        self.uineq = np.hstack([np.kron(np.ones(self.N + 1), self.x_max), np.kron(np.ones(self.N), self.u_max)])

        # - OSQP constraints
        self.A = sparse.vstack([self.Aeq, self.Aineq])
        self.l = np.hstack([self.leq, self.lineq])
        self.u = np.hstack([self.ueq, self.uineq])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        # Setup workspace
        self.prob.setup(self.P, self.q, self.A, self.l, self.u, warm_start=True, verbose=False)

        self.res = self.prob.solve()
'''

        #####################################################
        #               Initialize Publisher                #
        #####################################################
        rospy.init_node('pwm_controller_node', anonymous=True)
        self.pub_PWM = rospy.Publisher('pwm_interface', pwm_interface, queue_size=10)
        self.pub_odom_vel = rospy.Publisher('odom/velocity', Float64, queue_size=10)
        self.PWM = pwm_interface()
        self.odom_vel = Float64()
        self.FB_ENC = Float64()
        self.FB_ODOM = Odometry()
        self.rate = rospy.Rate(self.PUBLISH_RATE_PWM)

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

            #self.get_f1vt18_parameter("PARAMETERS_INITIALIZED")
            #self.get_f1vt18_parameter("PUBLISH_RATE_PWM")
            #self.get_f1vt18_parameter("EMERGENCY_FLAG")
            #self.set_f1vt18_parameter("VEHICLE_SLIPPING"
            #self.set_f1vt18_parameter("VEHICLE_SLIP_DETECTION_ACTIVE")
            #self.set_f1vt18_parameter("VEHICLE_SLIP_DETECTION_ACTIVE")


            # UPDATE PARAMETERS HERE
            self.VEHICLE_SLIPPING = False
            self.VEHICLE_SLIP_DETECTION_ACTIVE = False
            self.PARAMETER_UPDATED = True
            self.set_f1vt18_parameter("PWM_CONTROLLER_STARTED", self.PWM_CONTROLLER_STARTED)
        self.PWM_CONTROLLER_STARTED = False
        self.set_f1vt18_parameter("PWM_CONTROLLER_STARTED", self.PWM_CONTROLLER_STARTED)

    #####################################################
    #          Define State Transition System           #
    #####################################################
    def state_transition_system(self, t):
        return np.exp(self.A * t)

    #####################################################
    #             Initialize Subscriber                 #
    #####################################################
    def initialize_subscriber(self):
        while not rospy.is_shutdown():
            while not self.PWM_CONTROLLER_STARTED:
                pass
            rospy.Subscriber('pwm_controller', pwm_controller, self.update_reference_state)
            #rospy.Subscriber('odom/filtered_map', Odometry , self.update_feedback_odom)
            #rospy.Subscriber('odom/state_observer', Odometry , self.update_feedback_odom_observer)
            #rospy.Subscriber('encoder/velocity', Float64, self.update_feedback_enc)
            #rospy.Subscriber('control_state_observer', Float64, self.update_feedback_obs)
            rospy.Subscriber('control_state_observer', Float64, self.update_feedback_enc)

            self.SUBSCRIBER_ACTIVE = True
            rospy.spin()
        self.SUBSCRIBER_ACTIVE = False

    #####################################################
    #             /pwm_controller Callback              #
    #####################################################
    def update_reference_state(self, reference):
        self.reference_velocity = reference.velocity        # in % [-100,100]
        self.reference_steering = reference.steering
        self.REFERENCE_STATE_UPDATED = True

    #####################################################
    #             /pwm_controller Callback              #
    #####################################################
    def update_feedback_odom(self, feedback_odom):
        if self.FEEDBACK_ODOM_ACTIVATED:
            self.N_IDLE = 0
            self.DEACTIVATE_PUBLISH_ON_IDLE = False
        else:
            pass

        self.fb_odom_x = feedback_odom.pose.pose.position.x
        self.fb_odom_y = feedback_odom.pose.pose.position.y

        dt = rospy.get_time() - self.fb_odom_timer
        self.feedback_velocity_odom = (((self.fb_odom_x - self.fb_odom_x_prev)**2 + (self.fb_odom_y - self.fb_odom_y_prev)**2)**0.5)/dt

        self.fb_odom_x_prev = self.fb_odom_x
        self.fb_odom_y_prev = self.fb_odom_y
        self.feedback_velocity_odom = feedback_odom.velocity

        self.FEEDBACK_ODOM_UPDATED = True
        self.fb_odom_timer = rospy.get_time()


    #####################################################
    #             /pwm_controller Callback              #
    #####################################################
    def update_feedback_obs(self, feedback_obs):
        if self.FEEDBACK_OBS_ACTIVATED:
            self.N_IDLE = 0
            self.DEACTIVATE_PUBLISH_ON_IDLE = False
        else:
            pass
        self.feedback_velocity_obs = feedback_obs.data

        self.FEEDBACK_OBS_UPDATED = True

    #####################################################
    #             /pwm_controller Callback              #
    #####################################################
    def update_feedback_enc(self, feedback_enc):
        if self.FEEDBACK_ENC_ACTIVATED:
            self.N_IDLE = 0
            self.DEACTIVATE_PUBLISH_ON_IDLE = False
        else:
            pass
        ''' 
        if not self.FEEDBACK_ENC_AVG_INI:
            self.FEEDBACK_ENC_sum += feedback_enc.data
            self.FEEDBACK_ENC_AVG_n += 1
            if self.FEEDBACK_ENC_AVG_n >= self.FEEDBACK_ENC_AVG_N_MAX:
                self.FEEDBACK_ENC_AVG_INI = True
            else:
                pass
        else:
            self.FEEDBACK_ENC_sum -= self.FEEDBACK_ENC_prev
            self.FEEDBACK_ENC_sum += feedback_enc.data
        '''
        #self.feedback_velocity_enc = self.FEEDBACK_ENC_sum / self.FEEDBACK_ENC_AVG_N_MAX
        self.feedback_velocity_enc = feedback_enc.data
        self.FEEDBACK_ENC_UPDATED = True
        #self.FEEDBACK_ENC_prev = feedback_enc.data

    #####################################################
    #          Publish STOP to PWM_INTERFACE            #
    #####################################################
    def publish_pwm(self):
        while not rospy.is_shutdown():
            while not self.PWM_CONTROLLER_STARTED:
                pass
            while not self.PUBLISH_CONTROL_SIGNAL:
                pass
            while self.DEACTIVATE_PUBLISH_ON_IDLE:
                pass

            self.N_IDLE += 1                                # if no reference is received, deactivate
            if self.N_IDLE >= self.N_IDLE_THRESHOLD:
                self.PWM_velocity_control_signal = 0
                self.DEACTIVATE_PUBLISH_ON_IDLE = True
            else:
                pass

            self.PUBLISH_ACTIVE = True
            if (self.PWM_velocity_control_signal > self.PWM_NL_SAT_MIN) \
                        and (self.PWM_velocity_control_signal < self.PMW_NL_SAT_MAX) and self.CONSIDER_NONLINEARITY:
                temp_PWM_vel = 0
            else:
                temp_PWM_vel = self.PWM_velocity_control_signal

            self.PWM.velocity = temp_PWM_vel
            self.PWM.steering = self.PWM_steering_control_signal
            self.pub_PWM.publish(self.PWM)
            self.PUBLISH_ACTIVE = False

            self.CONTROL_OUTPUT_UPDATED = True
            self.rate.sleep()
            self.PUBLISHER_ACTIVE = True
        self.PUBLISHER_ACTIVE = False

    #####################################################
    #          Publish STOP to PWM_INTERFACE            #
    #####################################################
    def publish_odom_vel(self):                 # MIGHT TO BE REMOVED --> SEPARATE NODE MORE USEFUL FOR PUBLISHING VELOCITY
        while not rospy.is_shutdown():
            while not self.PWM_CONTROLLER_STARTED:
                pass
            while not self.PUBLISH_CONTROL_SIGNAL:
                pass
            self.odom_vel.data = self.feedback_velocity_odom
            self.pub_odom_vel.publish(self.odom_vel)
            self.rate.sleep()
            self.PUBLISHER_ODOM_VEL_ACTIVE = True
        self.PUBLISHER_ODOM_VEL_ACTIVE = False

    def update_slip(self):
        while not rospy.is_shutdown():
            while not self.PARAMETERS_INITIALIZED or not self.VEHICLE_SLIP_DETECTION_ACTIVE:
                pass

            self.FEEDBACK_ODOM_UPDATED = False
            while not self.FEEDBACK_ODOM_UPDATED:
                pass
            self.FEEDBACK_ENC_UPDATED = False
            while not self.FEEDBACK_ENC_UPDATED:
                pass

            self.vehicle_slip = (self.feedback_velocity_enc - self.feedback_velocity_odom) / self.feedback_velocity_odom # in % [-100, 100], -100 Wheel standing still
            self.SLIP_UPDATED = True
            self.set_f1vt18_parameter("MAIN_INTERFACE_STARTED", self.PWM_CONTROLLER_STARTED)

    def traction_control(self):
        while not rospy.is_shutdown():
            while not self.PARAMETERS_INITIALIZED or not self.VEHICLE_SLIP_DETECTION_ACTIVE or not self.VEHICLE_TRACTION_CONTROL_ACTIVE:
                pass
            self.SLIP_UPDATED = False
            while not self.SLIP_UPDATED:
                pass


    #####################################################
    #              UPDATE PARAMETERS                    #
    #####################################################
    def compute_control_signal(self):
        while not rospy.is_shutdown():
            if self.CONTROL_CHOICE == 'PID':
                while not self.COMPUTE_CONTROL_SIGNAL:
                    pass

                self.CONTROL_OUTPUT_UPDATED = False
                while self.CONTROL_OUTPUT_UPDATED:
                    pass

                self.REFERENCE_STATE_UPDATED = False
                while not self.REFERENCE_STATE_UPDATED:
                    pass
                if not self.PID_INITIALIZED:
                    self.P = 7
                    self.I = 3
                    self.D = 7
                    self.e_int = 0
                    self.e_d = 0
                    self.e_p = 0
                    self.AWU = False
                    self.t_prev = 0
                    self.dt = 0
                    self.e = 0
                    self.e_prev = 0
                    self.PID_INITIALIZED = True
                    self.ctrl_AWU_threshold = 100
                    self.ctrl_output = 0
                else:
                    self.e = self.reference_velocity - self.feedback_velocity_enc
                    self.dt = rospy.get_time() - self.t_prev
                    self.e_p = self.P*self.e
                    #print('Feedback velocity Encoder: {0}\r'.format(self.feedback_velocity_enc))

                    if not self.AWU:
                        self.e_int += self.I*self.e
                    else:
                        pass

                    self.e_d = (self.e - self.e_prev) / self.dt
                    self.ctrl_output = self.e_p + self.e_int + self.e_d
                    if abs(self.ctrl_output) >= self.ctrl_AWU_threshold:
                        self.AWU = True
                    else:
                        self.AWU = False
                    while self.PUBLISH_ACTIVE:
                        pass
                    self.PWM_velocity_control_signal = self.ctrl_output
                    self.e_prev = self.e

                ## IF K_ERROR > K_ERROR_THRESHOLD:
                # PWM_CONTROLLER_ERROR
                self.PUBLISH_CONTROL_SIGNAL = True
                self.CONTROL_UPDATED = True
                self.CONTROLLER_ACTIVE = True
                self.set_f1vt18_parameter("CONTROLLER_ACTIVE", self.CONTROLLER_ACTIVE)
            elif self.CONTROL_CHOICE == 'MPC':
                ''' 
                self.res = self.prob.solve()

                # Check solver status
                if self.res.info.status != 'solved':
                    raise ValueError('OSQP did not solve the problem!')

                # Apply first control input to the plant
                ctrl = self.res.x[-self.N * self.nu:-(self.N - 1) * self.nu]
                self.x0 = self.Ad.dot(self.x0) + self.Bd.dot(ctrl)


                self.PWM_velocity_control_signal = ctrl

                self.PUBLISH_CONTROL_SIGNAL = True
                self.CONTROL_UPDATED = True
                self.CONTROLLER_ACTIVE = True
                self.set_f1vt18_parameter("CONTROLLER_ACTIVE", self.CONTROLLER_ACTIVE)

                # Update initial state
                self.l[:self.nx] = -self.x0
                self.u[:self.nx] = -self.x0
                self.prob.update(l=self.l, u=self.u)
                '''
            else:
                print('CONTROLLER CHOICE ERROR!')
        self.CONTROLLER_ACTIVE = False
        self.set_f1vt18_parameter("CONTROLLER_ACTIVE", self.CONTROLLER_ACTIVE)


    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def display_interface(self):

        #####################################################
        #               WHILE LOOP for User Input           #
        #####################################################
        while not rospy.is_shutdown():
            self.cls()
            print("####################################\r")
            print(" Waiting for PARAMETERS INI         \r")
            print("------------------------------------\r")
            print('Press q if frozen...                \r')
            while not self.PARAMETERS_INITIALIZED and not rospy.is_shutdown():
                pass
            self.PWM_CONTROLLER_STARTED = True
            print("####################################\r")
            print("  Waiting for REFERENCE UPDATE      \r")
            print("------------------------------------\r")
            print('Press q if frozen...                \r')
            while not self.REFERENCE_STATE_UPDATED and not rospy.is_shutdown():
                pass

            self.COMPUTE_CONTROL_SIGNAL = True
            #####################################################
            #        Print Control Interface to Terminal        #
            #####################################################
            print("####################################\r")
            print("Current Status:                     \r")
            print("------------------------------------\r")
            print('PUBLISHER_ACTIVE:      {0} -        \r'.format(self.PUBLISHER_ACTIVE))
            print('SUBSCRIBER_ACTIVE:     {0} -        \r'.format(self.SUBSCRIBER_ACTIVE))
            print('CONTROLLER_ACTIVE:     {0} -        \r'.format(self.CONTROLLER_ACTIVE))
            print("# QUIT :    [     'q'    ]         #\r")
            print("####################################\r")
            time.sleep(self.dt_interface)

    #####################################################
    #   Initialize Control Interface for Terminal       #
    #####################################################
    def read_user_input(self):
        while self.KEY != 'q' and not rospy.is_shutdown():
            while not self.PARAMETERS_INITIALIZED and not rospy.is_shutdown():
                pass
            while not self.PARAMETER_UPDATED and not rospy.is_shutdown():
                pass
            self.KEY = ''

            self.KEY = readchar.readchar()

        self.PWM_STEER = 0
        self.PWM_VEL = 0
        self.PWM_CONTROLLER_STARTED = False


if __name__ == "__main__":
    try:
        print("####################################\r")
        print("#    PROGRAM HAS BEEN STARTED      #\r")
        print("####################################\r")
        print("... initialize controller object    \r")
        time.sleep(1)
        f1vt18_controller = VelocityController()

        Thread_Initialize_Subscriber = ThreadedFunction(f1vt18_controller.initialize_subscriber)
        Thread_Publish_PWM = ThreadedFunction(f1vt18_controller.publish_pwm)
        Thread_Publish_ODOM_VEL = ThreadedFunction(f1vt18_controller.publish_odom_vel)
        Thread_Update_Parameters = ThreadedFunction(f1vt18_controller.update_parameters)
        Thread_Compute_Control = ThreadedFunction(f1vt18_controller.compute_control_signal)
        Thread_Display_Interface = ThreadedFunction(f1vt18_controller.display_interface)

        Thread_Initialize_Subscriber.start()
        Thread_Publish_PWM.start()
        Thread_Publish_ODOM_VEL.start()
        Thread_Update_Parameters.start()
        Thread_Compute_Control.start()
        Thread_Display_Interface.start()

        f1vt18_controller.read_user_input()

    except rospy.ROSInterruptException:
        print("####################################\r")
        print("#  PROGRAM HAS BEEN INTERRUPTED    #\r")
        print("####################################\r")
        rospy.set_param('\MAIN_INTERFACE_STARTED', False)