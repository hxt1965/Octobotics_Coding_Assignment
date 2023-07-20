import rospy 
# import rospkg
import sys 
from inverted_pendulum_controller.msg import ControlForce 
from inverted_pendulum_sim.msg import CurrentState 
from inverted_pendulum_sim.srv import SetParams, SetParamsRequest, SetParamsResponse

class PendulumController:

    def __init__(self) -> None:
        self.control_force_pub = None 
        self.current_state_sub = None 
        self.init_pub_subs()
        
    def init_pub_subs(self) -> None:
        '''
        '''
        self.control_force_pub = rospy.Pulisher('/inverted_pendulum/control_force', 
                                                ControlForce,
                                                queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', 
                                                  CurrentState,
                                                  self.current_state_callback)
        
    
    def current_state_callback(self):
        '''
        '''
        return 
    
    def set_params(self):
        '''
        '''
        pass 




if __name__ == '__main__':
    rospy.init_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo()
