#!/usr/bin/env python3

import rospy
import sys
import threading
from duckietown_msgs.msg import WheelsCmdStamped
import os

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller_node', anonymous=True)
        self.current_command = None
        self.command_lock = threading.Lock()
        self.robot_name = os.environ.get('VEHICLE_NAME', 'duckiealexa')
        
        # Create publisher for wheel commands
        self.pub = rospy.Publisher(f'/{self.robot_name}/wheels_driver_node/wheels_cmd', 
                                  WheelsCmdStamped, queue_size=1)
        
        rospy.loginfo("Motor Controller Node initialized")
    
    def send_motor_command_fast(self, vel_left, vel_right):
        try:
            # Create and publish wheel command message directly
            msg = WheelsCmdStamped()
            msg.header.stamp = rospy.Time.now()
            msg.vel_left = vel_left
            msg.vel_right = vel_right
            
            self.pub.publish(msg)
            return True
            
        except Exception as e:
            rospy.logerr(f"Motor command failed: {e}")
            return False
    
    def stop_motors(self):
        return self.send_motor_command_fast(0.0, 0.0)
    
    def move_forward(self, speed=0.3):
        # Fixed: Forward should be positive velocities
        return self.send_motor_command_fast(speed, speed)
    
    def move_backward(self, speed=0.3):
        # Fixed: Backward should be negative velocities
        return self.send_motor_command_fast(-speed, -speed)
    
    def turn_left(self, speed=0.3):
        return self.send_motor_command_fast(-speed, speed)
    
    def turn_right(self, speed=0.3):
        return self.send_motor_command_fast(speed, -speed)
    
    def execute_command(self, command):
        with self.command_lock:
            if command == 'W':
                self.move_forward()
                rospy.loginfo("Moving forward")
            elif command == 'S':
                self.move_backward()
                rospy.loginfo("Moving backward")
            elif command == 'A':
                self.turn_left()
                rospy.loginfo("Turning left")
            elif command == 'D':
                self.turn_right()
                rospy.loginfo("Turning right")
            elif command == 'X':
                self.stop_motors()
                rospy.loginfo("Stopping")
    
    def run_interactive_mode(self):
        rospy.loginfo("Motor Controller Interactive Mode")
        rospy.loginfo("Available commands: W=FORWARD, S=BACKWARD, A=LEFT, D=RIGHT, X=STOP, Q=QUIT")
        rospy.loginfo("Commands execute immediately for faster response")
        
        while not rospy.is_shutdown():
            try:
                char = input("Enter command (W/S/A/D/X/Q): ").strip().upper()
                
                if char == 'Q':
                    self.stop_motors()
                    break
                elif char in ['W', 'S', 'A', 'D', 'X']:
                    # Execute command immediately in separate thread for faster response
                    threading.Thread(target=self.execute_command, args=(char,), daemon=True).start()
                else:
                    print("Invalid command. Please use: W=FORWARD, S=BACKWARD, A=LEFT, D=RIGHT, X=STOP, Q=QUIT")
                    
            except KeyboardInterrupt:
                self.stop_motors()
                break
        
        rospy.loginfo("Motor Controller shutting down")

def main():
    try:
        controller = MotorController()
        controller.run_interactive_mode()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()