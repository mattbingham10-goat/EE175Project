# from picamera2 import Picamera2, MappedArray
from time import sleep
import numpy as np
import cv2 as cv
import serial 
import time
import threading

def detect_circle(name, img):
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	circle_found = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 
									1, 20, param1= 30,
									param2 = 30, minRadius = 20, maxRadius= 100)
	frame_copy = img.copy()									
	if circle_found is not None:
		circle_found = np.round(circle_found[0, :]).astype("int")
		
		for (x, y, r) in circle_found:
			# a, b, r = pt[0], pt[1], pt[2]
			cv.circle(frame_copy, (x,y), r, (0,255,0), 2)
	# cv.imshow(name, frame_copy)
	return circle_found
	
def thresholding(img):
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	_, thresholded = cv.threshold(gray, 220, 255, cv.THRESH_BINARY)
	
	# cv.imshow("threshold", thresholded)
	return thresholded
	
def filter_red(img):
	HSV_frame = cv.cvtColor(img, cv.COLOR_BGR2HSV)
	mask1 = cv.inRange(HSV_frame, lower_red, upper_red)
	mask2 = cv.inRange(HSV_frame, lower_red2, upper_red2)
	mask = mask1 | mask2
	thresh_img = cv.bitwise_and(img, img, mask = mask)
	return thresh_img
	
def get_size(circle_array):
	if circle_array is None: 
		size = 0
	else:
		size = len(circle_array)
	return size
	
def locate_bright(img):
	gray = cv.GaussianBlur(img, (5,5), sigmaX = 4)
	# gray = cv.cvtColor(gray, cv.COLOR_BGR2GRAY)
	(minVal, maxVal, minLoc, maxLoc) = cv.minMaxLoc(gray)
	cv.circle(img,maxLoc,5,(255,0,0),2)
	cv.imshow("brightness", img)
	if current_time - last_sent_time >= interval:
		print(f"brightness: {maxVal}")
		print(f"brightness: {maxLoc}")
	if maxVal <= 10:
		return "EMPTY"
	return maxLoc[0]

def fetch_frames():
	global frame
	while True:
		ret, new_frame = video.read()
		if ret:
			frame = new_frame
			
ser = serial.Serial('/dev/ttyS0', 115200, write_timeout = 1)
num = "EMPTY"

# time.sleep(2)

last_sent_time= time.time()
interval = 0.5
exiting = False
loop_cnt = 16


frame = None
window_name = "Spotting Laser"
url = "http://192.168.45.101:81/stream"
# url2 = "http://192.168.45.226:81/stream"
# video = cv.VideoCapture("IMG_6004.mov")
# frame = cv.VideoCapture("Test1.jpg")
video = cv.VideoCapture(url)
video.set(cv.CAP_PROP_BUFFERSIZE, 3)

# cv.namedWindow(window_name, cv.WINDOW_AUTOSIZE)
# lower_red = np.array([0, 100, 70])
# upper_red = np.array([15,255,255])

# lower_red2 = np.array([165, 255, 255])
# upper_red2 = np.array([180,255,255])

thread = threading.Thread(target = fetch_frames)
thread.daemon = True
thread.start()

try:
	while(True):
		if frame is not None:
			current_time =time.time()
			# while(video.isOpened()):
				# video.set(cv.CAP_PROP_POS_FRAMES, video.get(cv.CAP_PROP_POS_FRAMES) +160)
				# video.set(cv.CAP_PROP_POS_FRAMES, video.get(cv.CAP_PROP_POS_FRAMES) +10)
				# frame = cam.capture_array()
			if exiting:
				break		
				
			# ycrcb_frame = cv.cvtColor(frame, cv.COLOR_BGR2YCrCb)
			# y_channel, cr_channel, cb_channel = cv.split(ycrcb_frame)
			# y_channel_stretched = cv.normalize(y_channel, None, 0, 255, cv.NORM_MINMAX)
			# y_channel_enhanced = cv.equalizeHist(y_channel_stretched)
			# contrast_stretched_ycrcb = cv.merge([y_channel_enhanced, cr_channel, cb_channel])	
			# contrast_stretched_frame = cv.cvtColor(contrast_stretched_ycrcb, cv.COLOR_BGR2YCrCb)
			
			x, y, w, h = 0, 50, 320//3, 240
			# x, y, w, h = 0, 0, 1200//3, 1200
			# x, y, w ,h = 0, 0, 640//3, 480
			
			cv.imshow("crop", frame[y:y+h,:])
			# cv.imshow("original", frame)
			
			thresh = thresholding(frame[y:y+h,:])

			x_loc= str(locate_bright(thresh))
			if x_loc != "EMPTY":
				x_loc = int(x_loc)		
											 
			left = frame[y:y+h, x: x+w]
			middle = frame[y:y+h, x+w:x+w+w] 
			right = frame[y:y+h, x+w+w:x+w+w+w]
			
			# cv.circle(frame,x_loc,5,(255,0,0),2)
			
			if x_loc == "EMPTY":
				num = "EMPTY"
			elif x_loc <= (x+w):
				num = "-10" + "\n"
			elif x_loc <= (x+w+w):
				num = "1" + "\n"				
			elif x_loc <= (x+w+w+w):
				num = "10" + "\n"
			
			if current_time - last_sent_time >= interval:
				last_sent_time = current_time
				if num == "EMPTY":
					num = "0" + "\n"
					ser.write(num.encode())
					ser.flush()
					print(" No Circle")
				else:
					ser.write(num.encode())
					ser.flush()
					print(f"sent: {num.strip()}")
							
				
			if cv.waitKey(100) & 0xFF == ord('q'):
				print("exit")
				break
			
			elif cv.waitKey(100) & 0xFF == ord('p'):
				print("manual")
				num= "2" + "\n"
				ser.write(num.encode())
				ser.flush()
				while True:
					if loop_cnt >= 16:
						num = "0" + "\n"
						ser.write(num.encode())
						ser.flush()
						print(f"sent: {num.strip()}")
						loop_cnt = 0
						
					key = cv.waitKey(40) & 0xFF
					if key== ord('w'):
						num = "1" + "\n"
						ser.write(num.encode())
						ser.flush()
						loop_cnt = 0
						print(f"sent: {num.strip()}")
					elif key == ord('d'):
						num = "10" + "\n"
						ser.write(num.encode())
						ser.flush()
						loop_cnt = 0
						print(f"sent: {num.strip()}")
					elif key == ord('a'):
						num = "-10" + "\n"
						ser.write(num.encode())
						ser.flush()
						loop_cnt = 0
						print(f"sent: {num.strip()}")
					# elif key == ord('s'):
						# num = "180" + "\n"
						# ser.write(num.encode())
						# ser.flush()
						# print(f"sent: {num.strip()}")
					elif key == ord('o'):
						num= "2" + "\n"
						ser.write(num.encode())
						ser.flush()
						loop_cnt = 6
						print(f"sent: {num.strip()}")
						break
					elif key == ord('q'):
						num= "2" + "\n"
						ser.write(num.encode())
						ser.flush()
						exiting = True
						print(f"sent: {num.strip()}")
						break
					else:
						loop_cnt += 1
					
			
except Exception as e:
	print("error capturing from the camera: ", e)
finally:
	# cam.stop()
	video.release()
	cv.destroyAllWindows()
	sleep(1)
	
			# use when using cable wired camera + using loop_cnt
# loop_cnt = 0
# cam = Picamera2()
# cam.preview_configuration.main.size = (1200, 1200)
# cam.preview_configuration.main.format = "RGB888"
# cam.preview_configuration.align()
# cam.configure("preview")
# cam.start()


			# from inside the while True for seperating the camera and sending Serial communication
# loop_cnt += 1
# time.sleep(0.2)

# left = thresh_image[y:y+h, x: x+w]
# middle = thresh_image[y:y+h, x+w:x+w+w] 
# right = thresh_image[y:y+h, x+w+w:x+w+w+w]
# circle_left = detect_circle("left", left)
# circle_middle = detect_circle("middle", middle)
# circle_right = detect_circle("right", right)
# size_left = get_size(circle_left)
# size_middle = get_size(circle_middle)
# size_right = get_size(circle_right)
# print(f"\n left: {size_left}, and middle: {size_middle}, and right: {size_right}")


# if size_left == size_middle == size_right:
	# num = "EMPTY"
# elif size_left > size_middle and size_left > size_right:
	# num = "45" + "\n"
# elif size_middle > size_left and size_middle > size_right:
	# num = "0" + "\n"
# else:
	# num = "-45" + "\n"		
	
# if num == "EMPTY":
	# num = "10" + "\n"
	# ser.write(num.encode())
	# ser.flush()
	# time.sleep(1)
	# print(" No Circle")
# else:
	# ser.write(num.encode())
	# ser.flush()
	# time.sleep(1)
	# print(f"sent: {num.strip()}")
