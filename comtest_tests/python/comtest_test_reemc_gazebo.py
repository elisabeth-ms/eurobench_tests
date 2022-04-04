#!/usr/bin/env python
    
import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
import numpy as np
import math
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import Duration, Time, Float64
import matplotlib.pylab as plt
from std_srvs.srv import Empty, EmptyResponse



class ComTest():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.pub_ref_x_force = rospy.Publisher('/force_x_applied', Float64, queue_size=10)
        self.pub_ref_y_force = rospy.Publisher('/force_y_applied', Float64, queue_size=10)
        self.start_server = rospy.Service('/start', Empty, self.start_forces)
        self.start = False
        self.firstTime = True
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        # self.send_force("base_link", 100.0, 100.0, 0.1)
        freq = 20.0
        rate = rospy.Rate(freq) # 10hz
        t = rospy.get_time()
        force_x = 0.0
        force_y = 0.0
        self.start_time = rospy.get_time()
        while not rospy.is_shutdown():
            print("hello")
            if(self.start):
                if(self.firstTime):
                    self.start_time = rospy.get_time()
                    self.firstTime = False
                t = rospy.get_time()
                force_x = self.compute_force(t, 55.0, 0.8, False)
                force_y = self.compute_force(t, 60.0, 0.5, False)
                self.send_force("base_link", force_x, force_y, 1/freq)
            print("publishing fx: ",force_x, "fy: ",force_y)
            self.pub_ref_x_force.publish(force_x)
            self.pub_ref_y_force.publish(force_y)
            rate.sleep()

    def start_forces(self, req):
        self.start = True
        return EmptyResponse


    def compute_force(self, time_instant, max_force, period_sin, add_random):
        print("Compute force")
        n = math.trunc((time_instant-self.start_time)/period_sin)
        aux_t = time_instant-self.start_time - n*period_sin
        print("time: ",time_instant-self.start_time,"period: ", period_sin, "n: ", n, "aux_ t: ", aux_t)

        force = max_force*np.sin(aux_t*2*np.pi/(period_sin))
        if add_random:
            force = force + np.random.normal()
        return force
    
    def send_force(self, body_name, x_force, y_force, dt):
        request = ApplyBodyWrenchRequest()
        request.body_name = body_name
        request.reference_frame = "world"
        request.wrench.force.x = x_force
        request.wrench.force.y = y_force
        request.duration = rospy.Duration.from_sec(dt)
        res2 = self.apply_body_wrench(request)
        return res2.success




# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('dysturbance_test_reemc_gazebo')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = ComTest()
    except rospy.ROSInterruptException: pass


