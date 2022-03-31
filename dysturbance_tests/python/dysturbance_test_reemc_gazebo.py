#!/usr/bin/env python


# Import required Python code.
import roslib
import rospy
import sys
import numpy as np

from eurobench_simulated_pendulum.srv import EuroBenchPendulum, EuroBenchPendulumResponse, EuroBenchPendulumRequest

# Give ourselves the ability to run a dynamic reconfigure server.


# Node example class.
class DysturbanceTest():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.

        rospy.wait_for_service('/eurobench_simulated_pendulum_node/apply_body_force')

        apply_body_force = rospy.ServiceProxy('/eurobench_simulated_pendulum_node/apply_body_force', EuroBenchPendulum)
            
        pendulum_mass = rospy.get_param("/dysturbance/pendulum/added_mass")
        pendulum_length = rospy.get_param("/dysturbance/pendulum/length")
        pendulum_id = rospy.get_param("/dysturbance/pendulum/id")
        
        platform_ground_inclination = rospy.get_param("/dysturbance/platform/ground_inclination")
        platform_id = rospy.get_param("/dysturbance/platform/id")
        
        
        subject_id = rospy.get_param("/dysturbance/subject/id")
        subject_name = rospy.get_param("/dysturbance/subject/name")
        subject_mass = rospy.get_param("/dysturbance/subject/mass")
        subject_height = rospy.get_param("/dysturbance/subject/height")
        subject_com_height = rospy.get_param("/dysturbance/subject/com_height")
        subject_base_depth = rospy.get_param("/dysturbance/subject/base_depth")
        subject_base_width = rospy.get_param("/dysturbance/subject/base_width")
        subject_orientation = rospy.get_param("/dysturbance/subject/orientation")
        
        protocol_id = rospy.get_param("/dysturbance/protocol/id")
        protocol_name = rospy.get_param("/dysturbance/protocol/name")
        protocol_notes = rospy.get_param("/dysturbance/protocol/notes")
        protocol_repetitions = rospy.get_param("/dysturbance/protocol/repetitions")
        protocol_parameters_id = rospy.get_param("/dysturbance/protocol/parameters/id")
        protocol_parameters_initial_upper_position = rospy.get_param("/dysturbance/protocol/parameters/initial_upper_position")

        dt = 0.1
        resp2 = apply_body_force.call(EuroBenchPendulumRequest(pendulum_mass, protocol_parameters_initial_upper_position*np.pi/180, 
                                                               subject_orientation*np.pi/180.0,pendulum_length, dt, 'base_link'))
    #     # Create a publisher for our custom message.
    #     pub = rospy.Publisher(topic, node_example_data)
    #     # Set the message to publish as our custom message.
        
    #     msg = node_example_data()
    #     # Initialize message variables.
    #     msg.a = 1
    #     msg.b = 2
    #     msg.message = init_message
    #     # Main while loop.
    #     while not rospy.is_shutdown():
    #         # Fill in custom message variables with values from dynamic reconfigure server.
    #         msg.message = self.message
    #         msg.a = self.a
    #         msg.b = self.b
    #         # Publish our custom message.
    #         pub.publish(msg)
    #         # Sleep for a while before publishing new messages. Division is so rate != period.
    #         if rate:
    #             rospy.sleep(1/rate)
    #         else:
    #             rospy.sleep(1.0)

    # # Create a callback function for the dynamic reconfigure server.
    # def reconfigure(self, config, level):
    #     # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
    #     self.message = config["message"]
    #     self.a = config["a"]
    #     self.b = config["b"]
    #     # Return the new variables.
    #     return config

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('dysturbance_test_reemc_gazebo')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = DysturbanceTest()
    except rospy.ROSInterruptException: pass