
"""Demo of Color Detection Algorithm/Following"""
import cv2
import math
import numpy as np
import sys
from color_balance import *
from rehaze import *
from motor_control import *

def find_color(frame):
  
  # Balance Colors
  color = balance(frame, 2)
  
  # Apply Gaussian Blur
  image_blur = cv2.GaussianBlur(color, (5,5), 0)

  # Convert image to HSV, then apply mask for specified color range
  hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)

  min_orange = np.array([0,100,20])
  max_orange = np.array([15,255,255])

  mask = cv2.inRange(hsv, min_orange, max_orange)

  kernel = np.ones((5,5),np.uint8)

  # Close small holes in detection mask
  mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
  mask_close = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, kernel)
  mask_clean = cv2.morphologyEx(mask_close, cv2.MORPH_CLOSE, kernel)
  return mask_clean

# Finds contours in generated image.
def find_contours(frame):
  
  # List of contours
  contours, _ = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

  filtered = []
  midpoints = []

  # Filters out contours that are unlikely to be a gate post.
  for c in contours:
    if cv2.contourArea(c) < 500:continue
    x,y,w,h = cv2.boundingRect(c)
    if w > (0.8*h):continue
    filtered.append(c)

  objects = np.zeros([frame.shape[0],frame.shape[1],3], 'uint8')

  # Takes the top three biggest contours found.
  filtered.sort(key=cv2.contourArea, reverse=True)
  filtered = filtered[:1]

  for c in filtered:
    col = (255, 255, 255)
    cv2.drawContours(objects,[c], -1, col, -1)
    area = cv2.contourArea(c)
    p = cv2.arcLength(c,True)
    approx = cv2.approxPolyDP(c, 0.03 * p, True)

    x,y,w,h = cv2.boundingRect(c)
    midpoint = (x+(w/2), y+(h/2))
    midpoints.append(midpoint)

    cv2.rectangle(objects,(x,y),(x+w,y+h),(0,255,0),2)
  return [objects, midpoints]

# Live: This function does live color detection.
def live(rehaze):
  cap = cv2.VideoCapture(0)

  width = int(cap.get(3))
  height = int(cap.get(4))
  center = ((width/2), (height/2))

  motor = True
  if "-nomotor" in sys.argv:
    motor = False
  armed = False

  if motor:
    # establish mavlink connection and switch into depth hold mode.
    connection = establish_connection()
    connection.wait_heartbeat()
    change_mode(connection, "ALT_HOLD")

  while(True):
    ret, frame = cap.read()

    if rehaze == True:
      frame = rehaze_image(frame)

    mask_clean = find_color(frame)
    contours = find_contours(mask_clean)
    objects = contours[0]
    midpoints = contours[1]

    if not midpoints:
      # Nothing detected, idle.
      if motor:
        # motor control enabled.
        stop(connection)
        disarm_motors(connection)
        armed = False
      pass
    else:
      # Detected orange. Follow item.
      midpoint = midpoints[0]

      float_x = round((midpoint[0] - center[0]) / center[0], 2)
      float_y = -(round((midpoint[1] - center[1]) / center[1], 2))

      if motor:
        # motor control enabled.
        if not armed:
          arm_motors(connection)
          armed = True
        move_target(connection, float_x, float_y)
        
      
    cv2.imshow('Raw', frame)
    cv2.imshow('Detection', objects)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  cap.release()
  cv2.destroyAllWindows()

# Reads in command line arguments to decide what to do.
if (len(sys.argv)) < 2:
  print("Not enough arguments.")
else:
  if sys.argv[1] == "-live":
    # Run live detection
    if sys.argv[2] == "true":
      live(True)
    else:
      live(False)