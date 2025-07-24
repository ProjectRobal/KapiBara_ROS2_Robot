import cv2
import numpy as np

# Read image.
img = cv2.imread('wall_1.png', cv2.IMREAD_COLOR)

# Convert to grayscale.
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

hist = cv2.calcHist(gray,[0],None,[256],[0,256])

print(hist)

low = 0

first = False

high = 255


max_i = np.argmax(hist)

i = max_i

while i < 255:
    
    if hist[i] == 0:
        high = i
        break
    
    i+=1
    
i = max_i

while i >= 0:
    
    if hist[i] == 0:
        low = i
        break

    i-=1

# high = 190

print(low,high)

ret,th1 = cv2.threshold(gray,200, 255,cv2.THRESH_BINARY)

cv2.imwrite("wall_th.png",th1)

# Blur using 3 * 3 kernel.
# gray_blurred = cv2.blur(gray, (3, 3))

contours, _ = cv2.findContours(th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

print(contours)

# Apply Hough transform on the blurred image.
detected_circles = cv2.HoughCircles(th1, 
                   cv2.HOUGH_GRADIENT, 1, 20, param1 = 200,
               param2 = 100, minRadius = 1, maxRadius = 400)

print(detected_circles)

for i, contour in enumerate(contours):
    if i == 0:
        continue

    # Approximate contour shape
    approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
    
    # Draw contour
    cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)
    
    x,y,w,h = cv2.boundingRect(contour)
    
    if w > 30 and h > 30:
        cv2.rectangle(img,(x,y),(x+w,y+h),(200,0,0),2)

cv2.imwrite("cricles.png", img)

# Draw circles that are detected.
if detected_circles is not None:

    # Convert the circle parameters a, b and r to integers.
    detected_circles = np.uint16(np.around(detected_circles))

    for pt in detected_circles[0, :]:
        a, b, r = pt[0], pt[1], pt[2]

        # Draw the circumference of the circle.
        cv2.circle(img, (a, b), r, (0, 255, 0), 2)

        # Draw a small circle (of radius 1) to show the center.
        cv2.circle(img, (a, b), 1, (0, 0, 255), 3)
        cv2.imshow("Detected Circle", img)
        cv2.waitKey(0)
        input()