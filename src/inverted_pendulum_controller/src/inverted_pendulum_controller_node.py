import rospy 
import rospkg
import sys 
from math import sin, cos, pi
from inverted_pendulum_controller.msg import ControlForce 
from inverted_pendulum_sim.srv import SetParams


class PendulumController:

    def __init__(self) -> None:
        self.control_force_pub = None 
        self.current_state_sub = None 
        self.init_pub_subs()

        self.force = 0 
        self.draw_counter = 0

        self.amplitude = 50 
        self.frequency = 2 
        
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
    
    def current_state_callback(self):
        '''
        '''
        rospy.wait_for_service('/inverted_pendulum/set_params') 
        try:
            set_params = rospy.ServiceProxy('/inverted_pendulum/set_params', SetParams)
            request = SetParamsRequest()
            
            request.pendulum_mass = 0
            request.pendulum_length = 0
            request.cart_mass = 0 
            response = set_params(request)
            return response 
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed')

    def set_params(self):
        '''
        '''
        pass 
    
    ## for PID, error is theta ? 
    def main_loop(self):
        while not rospy.is_shutdown():
            # implement clock.tick 
            self.force = sin()




if __name__ == '__main__':
    rospy.init_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo()
