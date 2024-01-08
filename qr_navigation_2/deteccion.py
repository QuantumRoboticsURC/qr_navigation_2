import rclpy
from rclpy import Node
from std_msgs.msg import Bool,Int8,Float64,Header
from geometry_msgs.msg import Twist,Point
from sensor_msgs.msg import Image
import numpy as np
import time
import cv2
import pyzed.sl as sl
import math


class Detect(Node):
    def __init__(self):
        super().__init__("Aruco_node")
        self.cmd_vel = self.create_publisher(Twist,"cmd_vel",10)
        self.x = 0
        self.y = 0
        self.distance = None
        
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }
        self.aruco_type = "DICT_4X4_50"
        self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
        self.init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

        # Open the camera
        status = self.zed.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
            print("Camera Open : "+repr(status)+". Exit program.")
            exit()

        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.point_cloud = sl.Mat()
        self.curr_signs_image_msg = Image()
        
        self.image_ocv = self.image.get_data()
        self.depth_ocv = self.image.get_data()
        
        self.mirror_ref = sl.Transform()
        self.mirror_ref.set_translation(sl.Translation(2.75,4.0,0)) 
         
        self.timer = self.create_timer(0.01,self.detect)
        
    def aruco_display(self,corners, ids, rejected, image):
        if len(corners) > 0:
            ids = ids.flatten()
            
            for (markerCorner, markerID) in zip(corners, ids):
                
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
    
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                print(f"x,y: {cX},{cY}")
                self.x=cX
                self.y=cY
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                
                print("[Inference] ArUco marker ID: {}".format(markerID))                
        return image
    
    def cv2_to_imgmsg(self, image, encoding = "bgr8"):
        #print("cv2_to_imgmsg image shape is:" + str(image.shape))
        if encoding == "bgr8":
            self.curr_signs_image_msg.header = Header()
            self.curr_signs_image_msg.height = image.shape[0]
            self.curr_signs_image_msg.width = image.shape[1]
            self.curr_signs_image_msg.encoding = encoding
            self.curr_signs_image_msg.is_bigendian = 0
            self.curr_signs_image_msg.step = image.shape[1]*image.shape[2]

            data = np.reshape(image, (self.curr_signs_image_msg.height, self.curr_signs_image_msg.step) )
            data = np.reshape(image, (self.curr_signs_image_msg.height*self.curr_signs_image_msg.step) )
            data = list(data)
            self.curr_signs_image_msg.data = data
            return self.curr_signs_image_msg
        else:            
            raise Exception("Error while convering cv image to ros message") 
          
    def detect(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.image_ocv = self.image.get_data()
            # Retrieve depth map. Depth is aligned on the left image
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            self.depth_ocv = self.depth.get_data()
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
    
            grayimg = cv2.cvtColor(self.image_ocv, cv2.COLOR_BGR2GRAY)
            #grayimgD = cv2.cvtColor(depth_ocv, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(grayimg, self.arucoDict, parameters=self.arucoParams)
            detected_markers = self.aruco_display(corners, ids, rejected, self.image_ocv)

    
            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            
            err, point_cloud_value = self.point_cloud.get_value(self.x, self.y)
            #distance = 0
            if math.isfinite(point_cloud_value[2]):
                self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                    point_cloud_value[1] * point_cloud_value[1] +
                                    point_cloud_value[2] * point_cloud_value[2])
                print(f"Distance to Camera at {{{self.x};{self.y}}}: {self.distance}")
		transform_aruco_midpoint_to_metric_system(point_cloud_value)
            else : 
        
                print(f"The distance can not be computed at {{{self.x},{self.y}}}")
            cv2.putText(detected_markers, f"Distancia: {self.distance}", (self.x, self.y -70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

def transform_aruco_midpoint_to_metric_system(self, point_cloud_value): 
   
        x_m = (0.25738586)*(point_cloud_value[0]) + 0.05862189

        x_px_times_y_px = point_cloud_value[0]*point_cloud_value[1]
        y_m = 0.29283879*point_cloud_value[0] + 0.00050015*point_cloud_value[1] + 0.00094536*x_px_times_y_px + 0.23096646

        x_px_times_z_px = point_cloud_value[0]*point_cloud_value[2]
        z_m = 0.16725805*point_cloud_value[0] - 0.00069012*point_cloud_value[2] + 0.00098029*x_px_times_z_px - 0.04520938 
	self.distance = math.sqrt(x_m+y_m+z_m)
        print(f"Distance to Camera at: {{{self.x_m};{self.y_m}}}: {self.distance}")

def main(args=None):
	rclpy.init(args=args)
	detect = Detect()
	rclpy.spin(detect)
	detect.destroy_node()
	rclpy.shutdown()
    
if __name__=="__main__":
    main()
	

