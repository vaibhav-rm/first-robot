import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
img = aruco.drawMarker(aruco_dict, 0, 500)

cv2.imwrite("aruco_0.png", img)
