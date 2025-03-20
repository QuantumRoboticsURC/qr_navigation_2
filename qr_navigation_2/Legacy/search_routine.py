import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Int8
from geometry_msgs.msg import Twist
from custom_interfaces.msg import TargetCoordinates
from submodules.alvinxy import *

class Search_routine(Node):
    def __init__(self):
        super().__init__("Route")
       
        #Publicadores
        self.pub_cmd = self.create_publisher("cmd_vel_sr",Twist, 10)
        self.pub_arrived = self.create_publisher("arrived_sr",Bool, 1)


        #Suscripciones
        self.sub_hammer = self.create_subscription("detected_hammer",Bool,self.detected_hammer_callback)
        self.sub_bottle = self.create_subscription("detected_bottle",Bool,self.detected_bottle_callback)
        self.sub_aruco = self.create_subscription("detected_aruco", Bool,self.detected_aruco_callback)
        self.sub_state = self.create_subscription ("state", Int32, self.update_state,10)
        self.target_coordinates = self.create_subscription(TargetCoordinates,"target_coordinates",self.update_target,1)

        # Variables de estado
        self.Twist = Twist()
        self.arrived_sr = False
        self.cmd_vel_sr = Twist()
        self.state = Int32()
        self.detected = False
        self.target_coords = [None,None]
        self.orglong = 0.0
        self.orglat = 0.0
       
        #Callbacks tópicos
       
    def update_state(self,msg):
        self.state=msg.data
        
    def update_target(self,msg):
        self.target_coords[0]=msg.latitude
        self.target_coords[1]=msg.longitude
    
    def  detected_hammer_callback(self, msg):
        self.detected = msg.data


    def detected_bottle_callback(self,msg):
        self.detected = msg.data
        
    def detected_aruco_callback(self,msg):
        self.detected = msg.data
    
    def xy2ll_simplyfied_for_snail_generation(self, cord):
        return xy2ll(cord[0], cord[1], self.target_latitude, self.target_longitude)

    #2, 3, 4
    def routine(self,num_turns = 2):      
        if self.state == 2 or self.state ==3 or self.state ==4:
            origin = (0,0)
            cords_list = [origin]
            current_cord = origin
            current_turn = 1
            disatnce_between_each_sanil_point = 5
            while not self.detected:
                while current_cord != (origin[0] + num_turns*disatnce_between_each_sanil_point, origin[1] -num_turns*disatnce_between_each_sanil_point):
                    if current_cord == (origin[0] + current_turn*disatnce_between_each_sanil_point, origin[1] -current_turn*disatnce_between_each_sanil_point):
                        current_turn += 1
                    if  (current_cord[1] - disatnce_between_each_sanil_point >= origin[1] -current_turn*disatnce_between_each_sanil_point and
                    (current_cord[0], current_cord[1] - disatnce_between_each_sanil_point) not in cords_list ):
                        current_cord = (current_cord[0], current_cord[1] - disatnce_between_each_sanil_point)
                        cords_list.append(current_cord)
                    elif (current_cord[0] - disatnce_between_each_sanil_point >= origin[0] -current_turn*disatnce_between_each_sanil_point and
                    (current_cord[0] - disatnce_between_each_sanil_point, current_cord[1]) not in cords_list ):
                        current_cord = (current_cord[0] - disatnce_between_each_sanil_point, current_cord[1])
                        cords_list.append(current_cord)
                    elif (current_cord[1] + disatnce_between_each_sanil_point <= origin[1] + current_turn*disatnce_between_each_sanil_point and
                    (current_cord[0], current_cord[1] + disatnce_between_each_sanil_point) not in cords_list ):
                        current_cord = (current_cord[0], current_cord[1] + disatnce_between_each_sanil_point)
                        cords_list.append(current_cord)
                    elif (current_cord[0] + disatnce_between_each_sanil_point <= origin[0] + disatnce_between_each_sanil_point*current_turn and
                    (current_cord[0] + disatnce_between_each_sanil_point, current_cord[1]) not in cords_list ):
                        current_cord = (current_cord[0] + disatnce_between_each_sanil_point, current_cord[1])
                        cords_list.append(current_cord)
                
                cords_list.pop(0)
                self.routine = list(map(self.xy2ll_simplyfied_for_snail_generation, cords_list))

    def main(args=None):
        rclpy.init(args = args)
        sr = Search_routine()
        rclpy.spin(sr)
        sr.destroy_node()
        rclpy.shutdown()


    if __name__=="__main__":
        main()