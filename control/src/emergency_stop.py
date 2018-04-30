#!/usr/bin/env python

import os
import readchar
import rospy
import time
from std_msgs.msg import Bool
#!/usr/bin/env python


#####################################################
#           Class definition for Failsafe           #
#####################################################
class Failsafe():
    def __init__(self):
        self.KEY = ''
        self.status_error = Bool()
        self.status_error = False
        rospy.init_node('pwm_interface_emergency_node', anonymous=True)
        self.pub_shutdown = rospy.Publisher('pwm_interface_emergency', Bool, queue_size=1)
        self.rate = rospy.Rate(10)
        self.listen_for_keyboard_input()

    #####################################################
    #                   Clear Screen                    #
    #####################################################
    def cls(self):
        os.system("clear")

    #####################################################
    #           Listening for Keyboard input            #
    #####################################################
    def listen_for_keyboard_input(self):
      time.sleep(0.5)
      self.status_error = False
      self.pub_shutdown.publish(self.status_error)
      while not self.KEY == 'x' and not rospy.is_shutdown():
         self.cls()
         print "--------------------------"
         print ("Status: " + str(self.status_error))
         print "#########################################################"
         print("# Enter 's' to trigger                                  #")
         print("# Enter 'r' to reset                                    #")
         print("# Enter 'x' to exit  and reset emergency to falses      #")
         print("#########################################################")
         self.KEY = readchar.readchar()
         print("Key entered: " + str(self.KEY))
         if self.KEY == 's':
             self.status_error = True
             print "----------------------------------------------------"
             print "  --- Emergency Exit ---  "
             # sys.exit()
         elif self.KEY == 'r':
             self.status_error = False
             print "----------------------------------------------------"
             print " -- RESET triggered ---  "
         elif self.KEY == 'x':
             self.status_error = False
             print "----------------------------------------------------"
             print " -- Exit program and emergency triggered ---  "
         else:
             pass
         self.pub_shutdown.publish(self.status_error)

      #####################################################
      #                 STOP on Panic exit                #
      #####################################################
      self.status_error = True
      self.pub_shutdown.publish(self.status_error)


if __name__ == "__main__":
    try:
        watchdog = Failsafe()
    except rospy.ROSInterruptException:
        pass
