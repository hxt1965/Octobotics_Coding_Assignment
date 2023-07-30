# credit: https://github.com/Nikkhil16/Inverted_Pendulum/blob/master/inverted_pendulum.py

import rospy 
# import rospkg
import sys 
import numpy as np
from control.matlab import * 
from math import sin, cos, pi
from inverted_pendulum_controller.msg import ControlForce 
from inverted_pendulum_sim.msg import CurrentState 
from inverted_pendulum_sim.srv import SetParams, SetParamsRequest, SetParamsResponse

class PendulumController:

    def __init__(self) -> None:
        self.control_force_pub = None
        self.current_state_sub = None
        self.init_pub_subs()

        self.force = 0 
        self.timer = 0

        self.current_theta = 0 
        self.current_x = 0 
        self.previous_error = pi 

        # default initialization values
        self.amplitude = 50 
        self.frequency = 2 

        self.pendulum_weight = 0 
        self.cart_weight = 0 
        self.pendulum_length = 0 
        self.cart_position = 0 
        self.theta = 300 

        self.control_type = 'pid'

        self.get_params()

        self.main_loop()
        
    def init_pub_subs(self) -> None:
        '''
        '''
        self.control_force_pub = rospy.Pulisher('/inverted_pendulum/control_force', 
                                                ControlForce,
                                                queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', 
                                                  CurrentState,
                                                  self.current_state_callback)
        # self.set_params_srv = rospy.Service()
    
    def get_params(self):
        self.pendulum_weight = rospy.get_param('~pendulum_weight')
        self.pendulum_length = rospy.get_param('~pendulum_length')
        self.cart_weight = rospy.get_param('~cart_weight')
        self.cart_position = rospy.get_param('~cart_position')
        self.theta = rospy.get_param('~theta')

    def get_force_from_pid(self, error, previous_error, time_delta, integral ):
        Kp = -150
        Kd = -20
        Ki = -20 

        derivative = (error - previous_error) / time_delta 
        integral += (error*time_delta)
        force = (Kp*error) + (Kd*derivative) + (Ki*integral)
        return force

    def current_state_callback(self, msg):
        '''
        '''

        # get pendulum current state here 
        self.current_theta = msg.curr_theta 
        error = self.get_error(self.current_theta)

        # calculate error 

        pass 
        
        

    def set_params(self):
        '''
        '''
        rospy.wait_for_service('/inverted_pendulum/set_params')
        try:
            set_params = rospy.ServiceProxy('/inverted_pendulum/set_params', SetParams)
            request = SetParamsRequest()
            
            request.pendulum_mass = self.pendulum_weight
            request.pendulum_length = self.pendulum_length
            request.cart_mass = self.cart_weight
            request.cart_x_0 = self.cart_position
            request.theta_0 = self.theta
            response = set_params(request)
            return response
        except rospy.ServiceException as err:
            rospy.logerr(f'Service call failed: ${err}')

    def get_error(self, theta):
        previous_error = theta%(2*pi)

        if previous_error > pi:
            previous_error = previous_error - (2*pi)

        return previous_error
    
    ## for PID, error is theta ? 
    def main_loop(self):
        # setup 

        while not rospy.is_shutdown():
            if self.control_type == 'pid':
                self.get_force_from_pid(0, 0, 0,0)
            elif self.control_type == 'sin':
                self.force = self.amplitude * sin(2*pi*self.frequency*self.timer)
            elif self.control_type == 'na':
                self.force = 0
            else:
                print('invalid control type argument')
                break
            
            force_msg = ControlForce()
            force_msg.force = self.force
            self.control_force_pub.publish(force_msg)
            self.timer += 1

        # if self.control_type == 'pid':
        #     while not rospy.is_shutdown():
        #         time_now = time.time()
        #         error = get_error()



        #         previous_error = error
        #         pass 

        # elif self.control_type == 'sin':
        #     while not rospy.is_shutdown():
        #         # implement clock.tick 
        #         self.force = self.amplitude * sin(2*pi*self.frequency*self.timer)
                
        #         force_msg = ControlForce()
        #         force_msg.force = self.force
        #         self.control_force_pub.publish(force_msg)
                
        #         self.timer += 1
        # else:
        #     while not rospy.is_shutdown():
        #         self.force = 0 





if __name__ == '__main__':
    rospy.init_node('PendulumController', anonymous=True)
    PendulumController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo()
