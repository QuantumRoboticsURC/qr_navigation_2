import numpy as np
import time
import cv2
import pyzed.sl as sl
import math
x=100
y=100
ARUCO_DICT = {
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

def aruco_display(corners, ids, rejected, image):
	if len(corners) > 0:
		global x,y
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
			
			x=cX
			y=cY
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			
			print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return image




aruco_type = "DICT_4X4_50"

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

arucoParams = cv2.aruco.DetectorParameters_create()

zed = sl.Camera()

# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

# Open the camera
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
	print("Camera Open : "+repr(status)+". Exit program.")
	exit()

# Create and set RuntimeParameters after opening the camera
runtime_parameters = sl.RuntimeParameters()

i = 0
image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()
image_ocv = image.get_data()
depth_ocv = image.get_data()
mirror_ref = sl.Transform()
mirror_ref.set_translation(sl.Translation(2.75,4.0,0))

while True:
	if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
		# Retrieve left image
		zed.retrieve_image(image, sl.VIEW.LEFT)
		image_ocv = image.get_data()
		# Retrieve depth map. Depth is aligned on the left image
		zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
		depth_ocv = depth.get_data()
		# Retrieve colored point cloud. Point cloud is aligned on the left image.
		zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
  
		grayimg = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2GRAY)
		grayimgD = cv2.cvtColor(depth_ocv, cv2.COLOR_BGR2GRAY)
		corners, ids, rejected = cv2.aruco.detectMarkers(grayimgD, arucoDict, parameters=arucoParams)

		detected_markers = aruco_display(corners, ids, rejected, image_ocv)

		cv2.imshow("Image", detected_markers)
		# Get and print distance value in mm at the center of the image
		# We measure the distance camera - object using Euclidean distance
		
		err, point_cloud_value = point_cloud.get_value(x, y)

		if math.isfinite(point_cloud_value[2]):
			distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
								point_cloud_value[1] * point_cloud_value[1] +
								point_cloud_value[2] * point_cloud_value[2])
			print(f"Distance to Camera at {{{x};{y}}}: {distance}")
		else : 
			print(f"The distance can not be computed at {{{x},{y}}}")

	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break

cv2.destroyAllWindows()
