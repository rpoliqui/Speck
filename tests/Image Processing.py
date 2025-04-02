import math
import cv2
import numpy as np

# define system constants
CRATE_WIDTH = 75  # mm
ORIENTATION_SQUARE_SIZE = 5  # mm

# Read the image
image = cv2.imread('Test Images/IMG_7284.jpg')
# resize the image
image = cv2.resize(image, (600, 800))  # Resize to 800x600
# Apply a Gaussian blur to reduce noise
blurred_image = cv2.GaussianBlur(image, (3, 3), 0)
# cv2.imshow('Blurred Image', blurred_image)

# Convert image to grayscale for edge detection
img_gray = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
cv2.imshow('Gray Scale', img_gray)

# Calculate limits for edge detection using the grayscale image
median = np.median(img_gray)
lower = int(max(0, (1.0 - 0.5) * median))
upper = int(min(255, (1.0 + 0.5) * median))
print("Edge thresholds:", lower, upper)

# Detect edges followed by 1 iteration of dilation and erosion to remove any background noise.
edge_image = cv2.Canny(img_gray, lower, upper)
edge_image = cv2.dilate(edge_image, None, iterations=1)
edge_image = cv2.erode(edge_image, None, iterations=1)

# Find contours
contours, hierarchy = cv2.findContours(edge_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Define arrays of important objects
large_contours = []
squares = []
center_points = []

# Detect squares and draw them on the image
for c in contours:
    # filter out small contours
    if cv2.contourArea(c) < 25:
        continue  # skip small contours
    large_contours.append(c)

    # find bounding box around each contour
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)
    box = np.array(box, dtype='int')

    # Get the width and height from the rectangle
    (w, h) = rect[1]
    if h == 0 or w == 0:
        continue  # avoid division by zero

    # Compute the aspect ratio (ensure it's >= 1 for logic to work)
    aspect_ratio = float(w) / h if w >= h else float(h) / w

    # contour is a square if aspect ratio is within 10% of 1
    is_square = aspect_ratio <= 1.1

    # if it is a square
    if is_square:
        # add to list of squares
        squares.append(rect)
        # Calculate centroid
        cX, cY = np.array(np.mean(box, axis=0), dtype='int')
        center_points.append([cX, cY])
        # draw the bounding box, center point, and corner circles
        cv2.drawContours(image, [box], -1, (255, 0, 0), 2)
        cv2.circle(image, (cX, cY), 2, (0, 0, 255), 3)
        for (x, y) in list(box):
            # print('(x,y):',(x,y))
            cv2.circle(image, (x, y), 2, (255, 0, 0), 2)

# draw point in center of image
image_center = [int(image.shape[1] / 2), int(image.shape[0] / 2)]
cv2.circle(image, image_center, 2, (0, 255, 0), 4)

# define adjustment variables. Variables will be updated in later stage
shift_x = 0
shift_y = 0
twist = 0

# convert list of center points and squares to an array
center_points = np.array(center_points)

# determine orientation of box based on number of squares detected
if len(squares) == 0:  # didn't find any squares
    print('Crate not found')

elif len(squares) == 1:  # found one square, assume bounding box around whole crate
    print('One Square Found')

elif len(squares) == 2:  # found 2 squares, assume two corners found
    print('Two Square Found')
    # determine in squares are in line or diagonal
    diff_vector = [center_points[0][0] - center_points[1][0], center_points[0][1] - center_points[1][1]]
    vector_angle = math.atan(diff_vector[1] / diff_vector[0]) * 180 / math.pi % 90
    rect_angle = ((squares[0][-1] + squares[1][-1]) / 2) % 90
    angle_diff = (abs(rect_angle) - abs(vector_angle))
    print(rect_angle, vector_angle, angle_diff)
    if abs(angle_diff) <= 10:  # assume stacked
        print(diff_vector)
        points = np.array([[center_points[0][0], center_points[0][1]],
                          [center_points[1][0], center_points[1][1]],
                          [center_points[1][0] - diff_vector[1], center_points[1][1] + diff_vector[0]],
                          [center_points[0][0] - diff_vector[1], center_points[0][1] + diff_vector[0]]])
        for point in points:
            cv2.circle(image, (point[0], point[1]), 2, (0, 255, 0), 2)
        rect = cv2.minAreaRect(points)
        box = np.array(cv2.boxPoints(rect), dtype=int)
    else:  # assume diagonal
        box = None
        rect = None

elif len(squares) == 3 or len(squares) == 4:  # found 3 - 4 squares, assume 3-4 corners found
    print('Three or Four Square Found')
    # use center points of squares to construct a box
    rect = cv2.minAreaRect(center_points)
    box = np.array(cv2.boxPoints(rect), dtype=int)
else:
    print('More than 4 squares found, could not find crate')
    box = None
    rect = None

# use side length of rectangle as reference for length
side_length = np.max(rect[1])
# find center of bounding box
crate_center = np.array(rect[0], dtype=int)
# draw box and center point
cv2.circle(image, crate_center, 2, (0, 0, 255), 4)
cv2.drawContours(image, [box], -1, (0, 0, 255), 2)
# draw array from point center to image center
cv2.arrowedLine(image, crate_center, image_center, (0, 0, 0), 1)
# calculate adjustments
shift_x = (image_center[0] - crate_center[0]) * CRATE_WIDTH / side_length
shift_y = (image_center[1] - crate_center[1]) * CRATE_WIDTH / side_length
twist = rect[-1]

# draw adjustments onto image
cv2.putText(
    image,  # image on which to draw text
    'Shift X = %.4f mm' % shift_x,
    (10, 20),  # bottom left corner of text
    cv2.FONT_HERSHEY_SIMPLEX,  # font to use
    0.5,  # font scale
    (255, 0, 0),  # color
    1,  # line thickness
)
cv2.putText(
    image,  # image on which to draw text
    'Shift Y = %.4f mm' % shift_y,
    (10, 40),  # bottom left corner of text
    cv2.FONT_HERSHEY_SIMPLEX,  # font to use
    0.5,  # font scale
    (255, 0, 0),  # color
    1,  # line thickness
)
cv2.putText(
    image,  # image on which to draw text
    'Twist = %.4f deg' % twist,
    (10, 60),  # bottom left corner of text
    cv2.FONT_HERSHEY_SIMPLEX,  # font to use
    0.5,  # font scale
    (255, 0, 0),  # color
    1,  # line thickness
)

print('twist:', twist, "deg")
print('shift X:', shift_x, "mm")
print('Shift Y:', shift_y, "mm")

# draw all large contours on the image
# cv2.drawContours(image, large_contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

# Display results
# cv2.imshow('Segmented Image', segmented_img)
cv2.imshow('Edges', edge_image)
cv2.imshow('Contours', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
