import rospy 
# import rospkg
import sys 
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

        self.amplitude = 50 
        self.frequency = 2 
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
            self.force = self.amplitude * sin(2*pi*self.frequency*self.timer)
            
            force_msg = ControlForce()
            force_msg.force = self.force 
            self.control_force_pub.publish(force_msg)
            
            self.timer += 1




if __name__ == '__main__':
    rospy.init_node('PendulumController', anonymous=True)
    PendulumController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo()
