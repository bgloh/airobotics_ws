#!/usr/bin/env python3

import rospy
import os
import sys

class LEDController:
    def __init__(self):
        rospy.init_node('led_controller_node', anonymous=True)
        rospy.loginfo("LED Controller Node initialized")
        self.robot_name = os.environ.get('VEHICLE_NAME', 'duckiealexa')
    
    def set_led_pattern(self, pattern_name):
        try:
            # Use rosservice call command directly
            # cmd = f'rosservice call /duckiealexa/led_emitter_node/set_pattern "pattern_name: {{data: {pattern_name}}}"'
            cmd = f'rosservice call /{self.robot_name}/led_emitter_node/set_pattern "pattern_name: {{data: {pattern_name}}}"'
            rospy.loginfo(f"Calling service: {cmd}")
            
            result = os.system(cmd)
            
            if result == 0:
                rospy.loginfo(f"Successfully set LED pattern to: {pattern_name}")
                return True
            else:
                rospy.logwarn(f"Failed to set LED pattern to: {pattern_name}")
                return False
            
        except Exception as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def run_interactive_mode(self):
        rospy.loginfo("LED Controller Interactive Mode")
        rospy.loginfo("Available commands: G=GREEN, B=BLUE, R=RED, W=WHITE, O=OFF, Q=QUIT")
        
        while not rospy.is_shutdown():
            try:
                char = input("Enter command (G/B/R/W/O/Q): ").strip().upper()
                
                if char == 'Q':
                    break
                elif char == 'G':
                    self.set_led_pattern('GREEN')
                elif char == 'B':
                    self.set_led_pattern('BLUE')
                elif char == 'R':
                    self.set_led_pattern('RED')
                elif char == 'W':
                    self.set_led_pattern('WHITE')
                elif char == 'O':
                    self.set_led_pattern('LIGHT_OFF')
                else:
                    print("Invalid command. Please use: G=GREEN, B=BLUE, R=RED, W=WHITE, O=OFF, Q=QUIT")
                    
            except KeyboardInterrupt:
                break
        
        rospy.loginfo("LED Controller shutting down")

def main():
    try:
        controller = LEDController()
        controller.run_interactive_mode()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()