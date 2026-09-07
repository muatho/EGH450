#!/usr/bin/env python3
import time
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

s1 = None
s2 = None
channels = [12,13] # GPIO12 (PWM0)    GPIO13 (PWM1)
t_low = 0.0005 # 500us ------ 0 degrees
t_high = 0.003 # 3000us -- 180 degrees
freq = 50 # 50Hz PWM signal
duty_low = (t_low / (1/freq))*100   # t_on = D*T ---> t_on/T = D
duty_high = (t_high / (1/freq))*100 # Multiplied by 100 to get Duty % 
                                    # duty_low ~= 0 degrees
                                    # duty_high ~=180 degrees

# Servo A ROS Callback
def callback_a(msg_in):
    # msg_in.data = Bool: True / False
    if msg_in.data:
        rospy.loginfo("[PDD] Actuating Servo A to Open Position")
        s1.ChangeDutyCycle(duty_high/2) # Set servo pos to 90 degrees
        time.sleep(0.5) # Wait for servo rotation
        s1.ChangeDutyCycle(0) # Prevent the servo from attempting to continue rotating in order to stop jitters
    else:
        rospy.loginfo("[PDD] Actuating Servo A to Closed Position")
        s1.ChangeDutyCycle(duty_low) # Repeat steps from before
        time.sleep(0.5)
        s1.ChangeDutyCycle(0)


# Servo B ROS Callback
def callback_b(msg_in):
    # msg_in.data = Bool: True / False
    if msg_in.data:
        rospy.loginfo("[PDD] Actuating Servo B to Open Position")
        s2.ChangeDutyCycle(duty_high/2) # Set servo pos to 90 degrees
        time.sleep(0.5) # Wait for servo rotation
        s2.ChangeDutyCycle(0) # Prevent the servo from attempting to continue rotating in order to stop jitters
    else:
        rospy.loginfo("[PDD] Actuating Servo B to Closed Position")
        s2.ChangeDutyCycle(duty_low) # Repeat steps from before
        time.sleep(0.5)
        s2.ChangeDutyCycle(0)


def shutdown():
    # Clean up any set ROS subscribers
    rospy.loginfo("[PDD] Shutdown cleanup in progress...")
    if sub_a is not None:
        sub_a.unregister()
    if sub_b is not None:
        sub_b.unregister()
    
    s1.stop() # Stop PWM
    s2.stop() # Stop PWM

    GPIO.cleanup() # Close GPIO
    rospy.loginfo("[PDD] Shutdown complete")

if __name__ == '__main__':
    # Setup the ROS backend for this node
    rospy.init_node('servo_controller', anonymous=True)
    
    # Setup the subscribers for both servos
    sub_a = rospy.Subscriber('/actuator_control/actuator_a', Bool, callback_a)
    sub_b = rospy.Subscriber('/actuator_control/actuator_b', Bool, callback_b)

    # Set board numbering mode
    GPIO.setmode(GPIO.BCM)

    # Set pins as an output and define servos as PWM
    GPIO.setup(channels, GPIO.OUT)
    s1 = GPIO.PWM(channels[0], freq) # Set PWM to apply 50Hz pulses on active pins
    s2 = GPIO.PWM(channels[1], freq)

    # Start PWM, but with 0% duty cycle (no pulse)
    s1.start(0)
    s2.start(0)

    # Perform cleanup on shutdown
    rospy.on_shutdown(shutdown)

    rospy.loginfo("[PDD] GPIO setup complete, ROS nodes running")
    # Loop forever
    rospy.spin()
