#!/usr/bin/env python


# Import required Python code.
import roslib
import rospy
import rospkg
import rosparam
import sys
import numpy as np
import os
from eurobench_simulated_pendulum.srv import EuroBenchPendulum, EuroBenchPendulumResponse, EuroBenchPendulumRequest
import shutil
import matplotlib.pyplot as plt

class ODESolver(object):
    """Second-order ODE Solver.
    Parameters
    ------------
    omega_0 : float
            initial angular velocity
    theta_0 : float
            initial angular displacement
    eta : float
        time step size
    n_iter : int
           number of steps
        
    Attributes
    -----------
    time_ : 1d-array
        Stores time values for each time step.
    omega_ : 1d-array
        Stores angular velocity values for each time step.
    theta_ : 1d-arra
       Stores angular displacement values for each time step.
        
    Methods
    -----------
    euler(alpha): Implements the Euler algorithm for the acceleration function alpha.
    
    midpoint(alpha): Implements the Midpoint algorithm for the acceleration function alpha.
    
    verlet(alpha): Implements the Verlet algorithm for the acceleration function alpha.
    """
    def __init__(self, omega_0 = 0, theta_0 = 10, eta=0.01, n_iter=10):
        self.omega_0 = omega_0
        self.theta_0 = theta_0
        self.eta = eta
        self.n_iter = n_iter
        
    def euler(self,alpha):
        """Implements Euler Method.
        
        Parameters
        ----------
        alpha : acceleration function
        
        Returns
        -------
        self : object
        """
        self.time_ = np.zeros(self.n_iter)
        self.omega_ = np.zeros(self.n_iter)
        self.theta_ = np.zeros(self.n_iter)
        self.omega_[0] = self.omega_0
        self.theta_[0] = self.theta_0*np.pi/180.0
        
        for i in range(self.n_iter-1):
            self.time_[i+1] = self.time_[i] + self.eta
            self.omega_[i+1] = self.omega_[i] + self.eta*alpha(self.theta_[i])
            self.theta_[i+1] = self.theta_[i] + self.eta*self.omega_[i]
        return self
    
    def midpoint(self,alpha):
        """Implement Midpoint Method.
        
        Parameters
        ----------
        alpha : acceleration function
        Returns
        -------
        self : object
        """
        self.time_ = np.zeros(self.n_iter)
        self.omega_ = np.zeros(self.n_iter)
        self.theta_ = np.zeros(self.n_iter)
        self.omega_[0] = self.omega_0
        self.theta_[0] = self.theta_0*np.pi/180.0
        
        for i in range(self.n_iter-1):
            self.time_[i+1] = self.time_[i] + self.eta
            self.omega_[i+1] = self.omega_[i] + self.eta*alpha(self.theta_[i])
            self.theta_[i+1] = self.theta_[i] + 0.5*self.eta*(self.omega_[i]+self.omega_[i+1])
        return self
    
    def verlet(self,alpha):
        """Implement Verlet Method.
        
        Parameters
        ----------
        alpha : acceleration function
        Returns
        -------
        self : object
        """
        self.time_ = np.zeros(self.n_iter)
        self.theta_ = np.zeros(self.n_iter)
        self.theta_[0] = self.theta_0*np.pi/180.0
        self.time_[1]= self.eta
        self.theta_[1] = self.theta_[0]+self.omega_0*self.eta +0.5* (self.eta**2)*alpha(self.theta_[0])
        
        for i in range(self.n_iter-2):
            self.time_[i+2] = self.time_[i+1] + self.eta
            self.theta_[i+2] = 2.0*self.theta_[i+1] -self.theta_[i] + (self.eta**2)*alpha(self.theta_[i+1])
        return self
def alpha(x):
    return -np.sin(x)




# Node example class.
class DysturbanceTest():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.


            
        self.pendulum_mass = rospy.get_param("/dysturbance/pendulum/added_mass")
        self.pendulum_length = rospy.get_param("/dysturbance/pendulum/length")
        self.pendulum_id = rospy.get_param("/dysturbance/pendulum/id")
        
        self.platform_ground_inclination = rospy.get_param("/dysturbance/platform/ground_inclination")
        self.platform_id = rospy.get_param("/dysturbance/platform/id")
        
        
        self.subject_id = rospy.get_param("/dysturbance/subject/id")
        self.subject_name = rospy.get_param("/dysturbance/subject/name")
        self.subject_mass = rospy.get_param("/dysturbance/subject/mass")
        self.subject_height = rospy.get_param("/dysturbance/subject/height")
        self.subject_com_height = rospy.get_param("/dysturbance/subject/com_height")
        self.subject_base_depth = rospy.get_param("/dysturbance/subject/base_depth")
        self.subject_base_width = rospy.get_param("/dysturbance/subject/base_width")
        self.subject_orientation = rospy.get_param("/dysturbance/subject/orientation")
        
        self.protocol_id = rospy.get_param("/dysturbance/protocol/id")
        self.protocol_name = rospy.get_param("/dysturbance/protocol/name")
        self.protocol_notes = rospy.get_param("/dysturbance/protocol/notes")
        self.protocol_repetitions = rospy.get_param("/dysturbance/protocol/repetitions")
        self.protocol_parameters_id = rospy.get_param("/dysturbance/protocol/parameters/id")
        self.protocol_parameters_initial_upper_position = rospy.get_param("/dysturbance/protocol/parameters/initial_upper_position")

        self.debug_acquisition = rospy.get_param("/dysturbance/debug_acquisition")
        self.reset_pendulum = rospy.get_param("/dysturbance/reset_pendulum")

        self.pendulumODESolver=ODESolver(omega_0 = 0, theta_0 = self.protocol_parameters_initial_upper_position, eta=0.001, n_iter=3000)


        rospy.set_param("/dysturbance/sampling_frequency", 10000.0)
        self.impact_duration = 0.01

        g = 9.81
        linear_density = 4.13

        # Create the structure of directories 

        self.create_directories_test()

        


        rospy.wait_for_service('/eurobench_simulated_pendulum_node/apply_body_force')

        for run in range(0, self.protocol_repetitions):
            platform_data_file_name_ = base_path + base_file_name + "_run_" + std::to_string(i) + "_platformData.csv";


            time=self.pendulumODESolver.verlet(alpha).time_
            theta=self.pendulumODESolver.verlet(alpha).theta_

            t=0
            for i, th in enumerate(theta):
                print(th)
                if th > 0:
                    t = time[i-1]
                    th = theta[i-1]
                    break

            print("Theta is ", th," at time: ", t)
            plt.plot(time,theta*180/np.pi,lw=3,color='blue')
            plt.xlabel('time(s)',size=13)
            plt.ylabel('angle (deg)',size=13)
            plt.title('Verlet Method',size=13)
            plt.show()

            apply_body_force = rospy.ServiceProxy('/eurobench_simulated_pendulum_node/apply_body_force', EuroBenchPendulum)

            resp2 = apply_body_force.call(EuroBenchPendulumRequest(self.pendulum_mass, self.protocol_parameters_initial_upper_position*np.pi/180, 
                                                                self.subject_orientation*np.pi/180.0,self.pendulum_length, self.impact_duration, 'torso_2_link'))
            



        # T0 = 2*np.pi*(np.sqrt(pendulum_length/g))
        # T = T0*(1+1/16.0*np.power(protocol_parameters_initial_upper_position,2)+11.0/3072.0*np.power(protocol_parameters_initial_upper_position,4))

    def create_directories_test(self):
        rospack = rospkg.RosPack()

        package_path = rospack.get_path('dysturbance_tests')

        self.base_path = package_path + "/tests/subject_"+str(self.subject_id)+"/"

        self.create_directory(self.base_path)

        config_subject_file_name = self.base_path + "subject_" + str(self.subject_id) +"_info.yaml"

        shutil.copyfile(package_path+"/config/config_subject.yaml", config_subject_file_name)


        self.base_path = self.base_path+"protocol_"+str(self.protocol_id)+"/"

        self.create_directory(self.base_path)

        self.base_file_name = "subject_"+str(self.subject_id)+"_cond_"
        self.base_file_name += str(self.protocol_id)
        self.base_file_name += str(self.protocol_parameters_id)
        self.base_file_name += str(self.pendulum_id)
        self.base_file_name += str(self.platform_id)

        self.base_path += base_file_name + "/"

        self.create_directory(self.base_path)

        self.base_path += "raw_data_input/"
        self.create_directory(self.base_path)

        config_file_name = self.base_path + self.base_file_name + "_testbed.yaml"

        rosparam.dump_params(config_file_name, '/dysturbance/', verbose=True)
    

    def create_directory(self, path):
        if not os.path.exists(path):
            print("Creating directory "+ path)
            os.mkdir(path)
        else:
            print("Directory: "+ path+" already exits!")


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('dysturbance_test_reemc_gazebo')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = DysturbanceTest()
    except rospy.ROSInterruptException: pass