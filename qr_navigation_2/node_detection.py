import rclpy
import argparse
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Thread
from std_msgs.msg import Bool,Int8,Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from custom_interfaces.msg import CA
import pyzed.sl as sl
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from ultralytics import YOLO
#import torch

def euler_from_quaternion(x, y, z, w):
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)
	return roll_x, pitch_y, yaw_z # in radians


class Detections(Node):
   
	def __init__(self):
		super().__init__("detection_node")
		timer_group = MutuallyExclusiveCallbackGroup()
		listener_group = ReentrantCallbackGroup()
		self.publisher_ = self.create_publisher(Image, '/camera/image', 10)
		self.center_approach = self.create_publisher(CA,"/center_approach",10)
		self.obstacle = self.create_publisher(Bool,"/object_detected",1)
		self.distance_obstacle = self.create_publisher(Int8, "/distance", 1)
		self.twist = Twist()
		self.found_aruco = self.create_publisher(Bool, "/detected_aruco", 1)
		self.found_orange = self.create_publisher(Bool, "/detected_orange", 1)
		self.found = self.create_publisher(Bool, "/detected_bottle", 1)
		self.zed_angle = self.create_publisher(Float64, "/zed_angle", 10)
		self.CA = CA()
		self.bridge = CvBridge()
		self.state_pub = self.create_publisher(Int8, "/state", 1)
		self.create_subscription(Int8, "/state", self.update_state, 1, callback_group=listener_group)

		parser = argparse.ArgumentParser()
		parser.add_argument('--svo', type=str, default=None, help='optional svo file')
		args = parser.parse_args()
		
		self.vel_x = 0.33
		self.vel_y = 0
		self.vel_theta = 0.1
		self.model = YOLO("yolov8n.pt")
		#modelo_yolo = torch.load("yolov8n.pt")
		#self.model = YOLO(modelo_yolo)
		self.x = 0
		self.y = 0
		self.distance = None
		self.contador = 0
		self.aruco_dis = False
		self.orange_dis = False
		self.bottle_dis = False
		self.is_center = False
		self.state = -1
		self.PIXEL_DISTANCE = 50

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
		self.saturation_threshold = 50
		
		self.zed = sl.Camera()

		self.quality = 10 
		self.create_subscription(Int8, "/image_quality", self.quality_callback, 1)

		input_type = sl.InputType()
		if args.svo is not None:
			input_type.set_from_svo_file(args.svo)

		# Crear un objeto InitParameters y establecer parámetros de configuración
		self.init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)

		# Create a InitParameters object and set configuration parameters
		self.init_params = sl.InitParameters()
		self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
		self.init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

		# Open the camera
		status = self.zed.open(self.init_params)
		if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
			self.get_logger().info("Camera Open : "+repr(status)+". Exit program.")
			exit()
		
		# Create and set RuntimeParameters after opening the camera
		self.runtime_parameters = sl.RuntimeParameters()
		self.image = sl.Mat()
		self.depth = sl.Mat()
		self.point_cloud = sl.Mat()
		
		self.image_ocv = self.image.get_data()
		self.image_ocv = self.image_ocv[:,:-1]
		self.depth_ocv = self.image.get_data()
		self.quality = 18
		
		self.mirror_ref = sl.Transform()
		self.mirror_ref.set_translation(sl.Translation(2.75,4.0,0)) 
		self.x_zed = 0.0
		self.y_zed = 0.0
		self.sensors_data = sl.SensorsData()
		#self.curr_signs_image_msg = self.cv2_to_imgmsg(self.image_ocv)
		self.timer = self.create_timer(0.00001,self.detect, callback_group=timer_group)
	
	def get_zed_imu_angle(self):
		self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.IMAGE)
		quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
		roll,pitch,yaw=euler_from_quaternion(quaternion.get()[0],quaternion.get()[1],quaternion.get()[2],quaternion.get()[3])
		self.zed_angle.publish(Float64(data=yaw))

	def orange_display(self, contours, image):
		if contours:
			self.orange_dis = True
			self.contador += 1
			
			for (idx, contour) in enumerate(contours):
				x, y, w, h = cv2.boundingRect(contour)

				corners = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype = np.int32)
				corners = corners.reshape((-1, 1, 2))

				cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

				cx = int(x + w / 2.0)
				cy = int(y + h / 2.0)
				print(f"x,y: {cx}, {cy}")
				self.get_logger().info(f"x,y: {cx}, {cy}")
				
				self.x = cx
				self.y = cy
				self.get_logger().info("Objeto naranja detectado")

		else:
			self.orange_dis = False
			self.contador = 0
			self.get_logger().info("Objeto naranja no detectado")
		return image
	
	def aruco_display(self,corners, ids, rejected, image):
		
		if len(corners) > 0:
			ids = ids.flatten()
			self.aruco_dis = True
			self.contador +=1  
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
				self.get_logger().info(f"x,y: {cX},{cY}")
				self.x=cX
				self.y=cY
				cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
					0.5, (0, 255, 0), 2)
				
				#print("[Inference] ArUco marker ID: {}".format(markerID))      
		else:
			self.aruco_dis=False
			self.contador=0       
		return image
	
	def bottle_display(self, frame, bboxes, classes):
		if bboxes.any():
			self.bottle_dis = True
			self.contador += 1

			for bbox, cls in zip(bboxes, classes):
				if cls == 39:
					(x, y, x2, y2) = bbox

					# Dibujar un rectángulo alrededor de la botella
					cv2.rectangle(frame, (x, y), (x2, y2), (0, 255, 0), 2)

					cv2.putText(frame, 'Botella', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

					cx = int(x + (x2 - x) / 2.0)
					cy = int(y + (y2 - y) / 2.0)
					self.get_logger().info(f"x,y: {cx}, {cy}")
					self.x = cx
					self.y = cy
					self.get_logger().info("Botella detectada")

		else:
			self.bottle_dis = False
			self.contador = 0
			self.get_logger().info("Botella no detectada")
		return frame
		
  # Listener de calidad de Imagen
	def quality_callback(self, msg):
		self.quality = msg.data

	def cv2_to_imgmsg(self, image):
		msg = self.bridge.cv2_to_imgmsg(image, encoding = "bgra8")
		return msg

	def cv2_to_imgmsg_resized(self, image, scale_percent):
		widht = int(image.shape[1] * scale_percent / 100)
		height = int(image.shape[0] * scale_percent / 100)
		dim = (widht, height)
		resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
		msg = self.bridge.cv2_to_imgmsg(resized_image, encoding = "bgra8")
		return msg

	def cv2_to_imgmsg_bottle(self, image):
		msg = self.bridge.cv2_to_imgmsg(image, encoding = "bgr8")
		return msg

	def cv2_to_imgmsg_resized_bottle(self, image, scale_percent):
		widht = int(image.shape[1] * scale_percent / 100)
		height = int(image.shape[0] * scale_percent / 100)
		dim = (widht, height)
		resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
		msg = self.bridge.cv2_to_imgmsg(resized_image, encoding = "bgr8")
		return msg
	
	def update_state(self, msg):
		self.state = msg.data

	def contornos(self,image):

		# Convertir el fotograma al espacio de color HSV
		frame_hsv = cv2.cvtColor(self.image_ocv, cv2.COLOR_BGR2HSV)
		saturation = frame_hsv[:,:,1]
		saturation_normalized = cv2.normalize(saturation,None,0,100,cv2.NORM_MINMAX)

		# Definir los umbrales de color naranja en el espacio de color HSV
		lower_orange = np.array([5, 130, 160])
		upper_orange = np.array([22, 255, 255])

		# Crear una máscara para detectar objetos de color naranja
		mask1 = cv2.inRange(frame_hsv, lower_orange, upper_orange)
		mask2 = cv2.inRange(saturation_normalized,self.saturation_threshold,100)
		combined_mask = cv2.bitwise_and(mask1,mask2)

		# Aplicar Adaptive Thresholding
		thresh = cv2.adaptiveThreshold(combined_mask, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

		# Aplicar operaciones morfológicas para eliminar el ruido
		thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

		# Encontrar contornos en la máscara
		contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
				
		return contours
	
	def detect(self):
		if self.state == 3:
			if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
				self.get_zed_imu_angle()
				# Retrieve left image
				# self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
				self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
				self.x_zed = round(self.image.get_width() / 2)
				self.y_zed = round(self.image.get_height() / 2)

				self.image_ocv = self.image.get_data()
				# Retrieve depth map. Depth is aligned on the left image
				self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
				self.depth_ocv = self.depth.get_data()
				# Retrieve colored point cloud. Point cloud is aligned on the left image.
				self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

				self.corners = self.contornos(self.image_ocv)

				detected_orange = self.orange_display(self.corners, self.image_ocv)
				if self.corners:

					ht, wd = self.image_ocv.shape[:2]

					# Encontrar el contorno más grande
					max_contour = max(self.corners, key=cv2.contourArea)

					# Obtener el rectángulo delimitador
					x, y, w, h = cv2.boundingRect(max_contour)

					# Calcular el centro del objeto naranja
					centro_objeto = (x + w // 2, y + h // 2)

					# Calcular el centro de la imagen
					centro_imagen = (wd // 2, ht // 2)

					# Determinar la posición relativa
					if centro_objeto[0] < centro_imagen[0] - 50:
						self.posicion = "A la izquierda"
						self.get_logger().info("A la izquierda")
					elif centro_objeto[0] > centro_imagen[0] + 50:
						self.posicion = "A la derecha"
						self.get_logger().info("A la derecha")
					else:
						self.posicion = "Centrado"
						self.get_logger().info("Centrado")

					# Dibujar un rectángulo alrededor del objeto naranja
					cv2.rectangle(self.image_ocv, (x, y), (x + w, y + h), (0, 255, 0), 2)

					cv2.putText(self.image_ocv, 'Naranja '  , (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

					err, point_cloud_value = self.point_cloud.get_value(self.x, self.y)
					#distance = 0
					if math.isfinite(point_cloud_value[2]):
						self.get_logger().info("Checking math")
						detected = Bool()
						if self.contador >= 2:
							self.get_logger().info("Martillo encontrado con contador "+str(self.contador))
							detected.data = True
							self.found_orange.publish(detected)
							self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
												point_cloud_value[1] * point_cloud_value[1] +
												point_cloud_value[2] * point_cloud_value[2])
							self.get_logger().info(f"Distance to Object at {{{self.x};{self.y}}}: {self.distance}")
							self.get_logger().info(f"Contador: {self.contador}")
							
							cv2.circle(detected_orange, (self.x_zed, self.y_zed),4,(0,0,255),-1)
							
							self.get_logger().info(f"x_z: {self.x_zed} y_z: {self.y_zed}")
							
							self.CA.distance = self.distance
							self.CA.x = self.x - self.x_zed
							
							if self.x > (self.x_zed+self.PIXEL_DISTANCE):
								self.get_logger().info(f"Objeto naranja a la derecha por: {self.x_zed+self.PIXEL_DISTANCE - self.x} pixeles \nCorrección: {self.PIXEL_DISTANCE*(1200/self.distance)}")
								self.CA.detected = False
							elif self.x < (self.x_zed-self.PIXEL_DISTANCE):
								self.get_logger().info(f"Objetonaranja a la izquierda por: {self.x - self.x_zed-self.PIXEL_DISTANCE} pixeles \nCorrección: {self.PIXEL_DISTANCE*(1200/self.distance)}")
								self.CA.detected = False
							#elif self.x >= (self.x_zed-20) and self.x <= (self.x_zed+20):
							else:
								self.get_logger().info(f"Objeto al centro")
								cv2.putText(detected_orange, f"Centro", (self.x, self.y -80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
								self.CA.detected = True
							self.center_approach.publish(self.CA)
						else:
							self.distance=None
							self.get_logger().info("Not detected "+str(self.orange_dis))
							self.get_logger().info(f"x_z: {self.x_zed} y_z: {self.y_zed}")

					cv2.putText(detected_orange, f"Distancia: {self.distance}", (self.x, self.y - 64), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
					cv2.putText(detected_orange, f"Posicion: {self.posicion}", (self.x, self.y - 37), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
					#self.cv2_to_imgmsg(detected_orange)
					self.publisher_.publish(self.cv2_to_imgmsg_resized(detected_orange,self.quality))
					#self.get_logger().info("Publicando video")
				
				else:      
					#self.cv2_to_imgmsg(detected_orange)
					self.publisher_.publish(self.cv2_to_imgmsg_resized(detected_orange,self.quality))
					self.get_logger().info("Publicando video sin deteccion")

		elif self.state == 4:
			if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
				self.get_zed_imu_angle()
				# Retrieve left image
				self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
				self.x_zed = round(self.image.get_width() / 2)
				self.y_zed = round(self.image.get_height() / 2)
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
					detected = Bool()
					if self.contador >= 2:
						detected.data = True
						self.get_logger().info("Aruco encontrado")
						self.found_aruco.publish(detected)
						self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
											point_cloud_value[1] * point_cloud_value[1] +
											point_cloud_value[2] * point_cloud_value[2])
						self.get_logger().info(f"Distance to Aruco at {{{self.x};{self.y}}}: {self.distance}")
						self.get_logger().info(f"Contador: {self.contador}")
						
						self.x_zed = round(self.image.get_width() / 2)+self.PIXEL_DISTANCE
						self.y_zed = round(self.image.get_height() / 2)
						cv2.circle(detected_markers, (self.x_zed, self.y_zed),4,(0,0,255),-1)
						
						self.get_logger().info(f"x_z: {self.x_zed} y_z: {self.y_zed}")
						
						self.CA.distance = self.distance
						self.CA.x = self.x - self.x_zed
	  
						if self.x > (self.x_zed+self.PIXEL_DISTANCE):
							self.get_logger().info(f"Aruco a la derecha por: {self.x_zed+self.PIXEL_DISTANCE - self.x} pixeles \nCorrección: {self.PIXEL_DISTANCE*(1200/self.distance)}")
							self.CA.detected = False
						elif self.x < (self.x_zed-self.PIXEL_DISTANCE):
							self.get_logger().info(f"Aruco a la izquierda por: {self.x - self.x_zed-self.PIXEL_DISTANCE} pixeles \nCorrección: {self.PIXEL_DISTANCE*(1200/self.distance)}")
							self.CA.detected = False
						else: 
							self.get_logger().info(f"Aruco al centro")
							cv2.putText(detected_markers, f"Centro", (self.x, self.y -80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
							self.CA.detected = True
							
						self.center_approach.publish(self.CA)
					else:
						self.distance=None
						self.get_logger().info("Not detected " + str(self.aruco_dis))
						self.get_logger().info(f"x_z: {self.x_zed} y_z: {self.y_zed}")

			cv2.putText(detected_markers, f"Distancia: {self.distance}", (self.x, self.y - 64), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
			cv2.putText(detected_markers, f"Posicion: {self.posicion}", (self.x, self.y - 37), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
			self.publisher_.publish(self.cv2_to_imgmsg_resized(detected_markers, self.quality))
			#self.get_logger().info("Publicando video")
		
		elif self.state==2:
			if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
				self.get_zed_imu_angle()
				# Retrieve left image
				self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
				self.x_zed = round(self.image.get_width() / 2)
				self.y_zed = round(self.image.get_height() / 2)
				self.image_ocv = self.image.get_data()
				# Retrieve depth map. Depth is aligned on the left image
				self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
				self.depth_ocv = self.depth.get_data()
				self.image_col = cv2.cvtColor(self.image_ocv, cv2.COLOR_BGRA2RGB)
				# Retrieve colored point cloud. Point cloud is aligned on the left image.
				self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

				results = self.model(self.image_col)

				result = results[0]
				bboxes = np.array(result.boxes.xyxy.cpu(), dtype="int")
				classes = np.array(result.boxes.cls.cpu(), dtype="int")

				detected_bottle = self.bottle_display(self.image_col, bboxes, classes)

				if bboxes.any():
					ht, wd = self.image_col.shape[:2]

					for bbox, cls in zip(bboxes, classes):
						# Solo procesar detecciones de clase 39 (botella) <IMPORTANTE>
						if cls == 39:
							
							# Encontrar el contorno más grande
							max_bbox = bbox

							# Calcular el centro de la botella
							centro_objeto = ((max_bbox[0] + max_bbox[2]) // 2, (max_bbox[1] + max_bbox[3]) // 2)

							# Calcular el centro de la imagen
							centro_imagen = (wd // 2, ht // 2)

							# Determinar la posición relativa
							if centro_objeto[0] < centro_imagen[0] - 50:
								self.posicion = "A la izquierda"
								self.get_logger().info("A la izquierda")
							elif centro_objeto[0] > centro_imagen[0] + 50:
								self.posicion = "A la derecha"
								self.get_logger().info("A la derecha")
							else:
								self.posicion = "Centrado"
								self.get_logger().info("Centrado")

	
							# Publicar información sobre la botella detectada
							err, point_cloud_value = self.point_cloud.get_value(self.x, self.y)
							if math.isfinite(point_cloud_value[2]):
								detected = Bool()
								if self.contador >= 2:
									detected.data = True
									self.get_logger().info("Botella encontrada")
									self.found.publish(detected)
									self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
														point_cloud_value[1] * point_cloud_value[1] +
														point_cloud_value[2] * point_cloud_value[2])
									self.get_logger().info(f"Distance to Bottle at {{{self.x};{self.y}}}: {self.distance}")
									self.get_logger().info(f"Contador: {self.contador}")
									
									self.x_zed = round(self.image.get_width() / 2)
									self.y_zed = round(self.image.get_height() / 2)
									cv2.circle(detected_bottle, (self.x_zed, self.y_zed),4,(0,0,255),-1)
									
									self.get_logger().info(f"x_z: {self.x_zed} y_z: {self.y_zed}")
									
									self.CA.distance = self.distance
									self.CA.x = self.x - self.x_zed

									if self.x > (self.x_zed+self.PIXEL_DISTANCE):
										self.get_logger().info(f"Botella a la derecha por: {self.x_zed+self.PIXEL_DISTANCE - self.x} pixeles \nCorrección: {self.PIXEL_DISTANCE*(1200/self.distance)}")
										self.CA.detected = False
									elif self.x < (self.x_zed-self.PIXEL_DISTANCE):
										self.get_logger().info(f"Botella a la izquierda por: {self.x - self.x_zed-self.PIXEL_DISTANCE} pixeles \nCorrección: {self.PIXEL_DISTANCE*(1200/self.distance)}")
										self.CA.detected = False
									else: #self.x >= (self.x_zed-20) and self.x <= (self.x_zed+20):
										self.get_logger().info(f"Botella al centro")
										cv2.putText(detected_bottle, f"Centro", (self.x, self.y -80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
										self.CA.detected = True
									self.center_approach.publish(self.CA)
								else:
									self.distance=None
									self.get_logger().info("Not detected " + str(self.bottle_dis))
									self.get_logger().info(f"x_z: {self.x_zed} y_z: {self.y_zed}")


							cv2.putText(detected_bottle, f"Distancia: {self.distance}", (max_bbox[0], max_bbox[1] - 64),
										cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
							cv2.putText(detected_bottle, f"Posicion: {self.posicion}", (max_bbox[0], max_bbox[1] - 37),
										cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
							self.publisher_.publish(self.cv2_to_imgmsg_resized_bottle(detected_bottle, self.quality))
							#self.get_logger().info("Publicando video")

						else:
							self.distance = None
							self.get_logger().info("Not detected "+ str(self.bottle_dis))
							self.publisher_.publish(self.cv2_to_imgmsg_resized_bottle(detected_bottle, self.quality))
 
				else:
					self.publisher_.publish(self.cv2_to_imgmsg_resized_bottle(detected_bottle, self.quality))
					self.get_logger().info("Publicando video sin deteccion")
		
		elif self.state==0:
			if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
				self.get_zed_imu_angle()
				# Retrieve left image
				self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
				self.image_ocv = self.image.get_data()
				# Retrieve depth map. Depth is aligned on the left image
				self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
				self.depth_ocv = self.depth.get_data()
				# Retrieve colored point cloud. Point cloud is aligned on the left image.
				self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
				self.x_zed = round(self.image.get_width() / 2)
				self.y_zed = round(self.image.get_height() / 2)
				err, point_cloud_value = self.point_cloud.get_value(self.x_zed, self.y_zed)
				#distance = 0
				if math.isfinite(point_cloud_value[2]):
					self.distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
										point_cloud_value[1] * point_cloud_value[1] +
										point_cloud_value[2] * point_cloud_value[2])
					self.get_logger().info("Distancia "+str(self.distance))

					if(self.distance<650):
						object = Bool()
						object.data = True
					else:
						object = Bool()
						object.data = False

					self.obstacle.publish(object)
					self.distance_obstacle.publish(Int8(data=self.distance))
						
				self.publisher_.publish(self.cv2_to_imgmsg_resized(self.image_ocv, self.quality))	
						


def main(args=None):
	rclpy.init(args=args)
	det = Detections()
	executor = MultiThreadedExecutor()
	executor.add_node(det)
	executor.spin()
	det.zed.close()
	det.destroy_node()
	rclpy.shutdown()
	
if __name__=="__main__":
	main()
