import math
import cv2
import numpy as np


def process_image(image, blur, sensitivity, loops=0):
    if loops > 100:
        print("!!Failed to Find Crate!!")
        cv2.imshow('Processed image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return
    print(blur, sensitivity)
    # resize the image
    image = cv2.resize(image, (600, 800))  # Resize to 800x600
    # Apply a Gaussian blur to reduce noise
    blurred_image = cv2.GaussianBlur(image, (blur, blur), 0)
    # cv2.imshow('Blurred Image', blurred_image)

    # create copy of image to draw on
    image_copy = image.copy()

    # Convert image to grayscale for edge detection
    img_gray = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Gray Scale', img_gray)

    # Calculate limits for edge detection using the grayscale image
    median = np.mean(blurred_image)
    lower = int(max(0, (1.0 - sensitivity) * median))
    upper = int(min(255, (1.0 + sensitivity) * median))

    # Detect edges followed by 1 iteration of dilation and erosion to remove any background noise.
    edge_image = cv2.Canny(img_gray, lower, upper)
    edge_image = cv2.dilate(edge_image, None, iterations=1)
    edge_image = cv2.erode(edge_image, None, iterations=1)

    # Find contours
    contours, hierarchy = cv2.findContours(edge_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

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

        # apply approximation algorithm to simplify contours
        epsilon = 0.1 * cv2.arcLength(c, True)

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
            # draw contours on original image
            cv2.drawContours(image, c, -1, (0, 255, 255), 1)
            # draw the bounding box, center point, and corner circles
            cv2.drawContours(image_copy, [box], -1, (255, 0, 0), 2)
            cv2.drawContours(image_copy, [box], -1, (255, 0, 0), 2)
            cv2.circle(image_copy, (cX, cY), 2, (0, 0, 255), 3)
            for (x, y) in list(box):
                # print('(x,y):',(x,y))
                cv2.circle(image_copy, (x, y), 2, (255, 0, 0), 2)

    # draw point in center of image
    image_center = [int(image.shape[1] / 2), int(image.shape[0] / 2)]
    cv2.circle(image_copy, image_center, 2, (0, 255, 0), 4)

    # define adjustment variables. Variables will be updated in later stage
    shift_x = 0
    shift_y = 0
    twist = 0

    # convert list of center points and squares to an array
    center_points = np.array(center_points)

    # determine orientation of box based on number of squares detected
    if len(squares) < 3:  # didn't find any squares
        print('Found %i sqaures(s)' %len(squares))
        if 9 >= blur > 1:  # reprocess image with less blur
            process_image(image, blur - 2, sensitivity, loops+1)
        elif blur == 1 and sensitivity < 1:  # reprocess image with more sensitivity
            process_image(image, 9, sensitivity + 0.05, loops+1)
        elif sensitivity >= 1:
            process_image(image, 9, sensitivity - .05, loops+1)
        return None

    elif len(squares) == 3:
        print('Three Squares Found')
        # use center points of squares to construct a box
        (x, y), rad = cv2.minEnclosingCircle(center_points)
        # if square one is significantly larger than the others
        if(squares[0][1][0] > 1.25*squares[1][1][0]) and (squares[0][1][0] > 1.25*squares[2][1][0]):
            # assume that it contains the entire crate.
            crate_center = [int(squares[0][0][0]), int(squares[0][0][1])]
            rect = squares[0]
        # if square two is significantly larger than the others
        elif(squares[1][1][0] > 1.25*squares[0][1][0]) and (squares[1][1][0] > 1.25*squares[2][1][0]):
            # assume that is contains the entire crate.
            crate_center = [int(squares[1][0][0]), int(squares[1][0][1])]
            rect = squares[1]
        # if square two is significantly larger than the others
        elif (squares[2][1][0] > 1.25*squares[0][1][0]) and (squares[2][1][0] > 1.25*squares[1][1][0]):
            # assume that is contains the entire crate.
            crate_center = [int(squares[2][0][0]), int(squares[2][0][1])]
            rect = squares[2]
        # otherwise assume three corners were found
        else:
            crate_center = [int(x), int(y)]
            cv2.circle(image_copy, crate_center, int(rad), (0, 0, 255), 2)
            rect = cv2.minAreaRect(center_points)
        box = cv2.boxPoints(rect)
        box = np.array(box, dtype='int')
        cv2.drawContours(image_copy, [box], -1, (0, 0, 255), 2)
        # calculate adjustments
        side_length = np.max(rect[1])
        shift_x = (image_center[0] - crate_center[0]) * CRATE_WIDTH / side_length
        shift_y = (image_center[1] - crate_center[1]) * CRATE_WIDTH / side_length
        twist = rect[-1] % 90
        # only need to rotate angles less than 45 degrees
        if twist > 45:
            twist = twist - 90
    elif len(squares) == 4:  # found 3 - 4 squares, assume 3-4 corners found
        print('Four Squares Found')
        # use center points of squares to construct a box
        rect = cv2.minAreaRect(center_points)
        box = np.array(cv2.boxPoints(rect), dtype=int)
        # use side length of rectangle as reference for length
        side_length = np.max(rect[1])
        # find center of bounding box
        crate_center = [int(rect[0][0]), int(rect[0][1])]
        cv2.drawContours(image_copy, [box], -1, (0, 0, 255), 2)
        # calculate adjustments
        shift_x = (image_center[0] - crate_center[0]) * CRATE_WIDTH / side_length
        shift_y = (image_center[1] - crate_center[1]) * CRATE_WIDTH / side_length
        # find angle to the nearest 90 degrees
        twist = rect[-1] % 90
        # only need to rotate angles less than 45 degrees
        if twist > 45:
            twist = twist - 90
    else:
        print('More than 4 squares found, could not find crate')
        crate_center = image_center

    # draw center point
    cv2.circle(image_copy, crate_center, 2, (0, 0, 255), 4)
    # draw array from point center to image center
    print(crate_center)
    cv2.arrowedLine(image_copy, crate_center, image_center, (0, 0, 0), 1)

    # draw adjustments onto image
    cv2.putText(
        image_copy,  # image on which to draw text
        'Shift X = %.4f mm' % shift_x,
        (10, 20),  # bottom left corner of text
        cv2.FONT_HERSHEY_SIMPLEX,  # font to use
        0.5,  # font scale
        (255, 0, 0),  # color
        1,  # line thickness
    )
    cv2.putText(
        image_copy,  # image on which to draw text
        'Shift Y = %.4f mm' % shift_y,
        (10, 40),  # bottom left corner of text
        cv2.FONT_HERSHEY_SIMPLEX,  # font to use
        0.5,  # font scale
        (255, 0, 0),  # color
        1,  # line thickness
    )
    cv2.putText(
        image_copy,  # image on which to draw text
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
    # cv2.imshow('Edges', edge_image)
    cv2.imshow('Processed Image', image_copy)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # rotate image based on calculation and reprocess
    M = cv2.getRotationMatrix2D(crate_center, twist, 1)
    (h, w) = image.shape[:2]
    rotated = cv2.warpAffine(image, M, (w, h))
    cv2.imshow('Rotated Image', rotated)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return shift_x, shift_y, twist


if __name__ == '__main__':
    # define system constants
    CRATE_WIDTH = 75  # mm

    # Read the image
    img = cv2.imread('Test Images/IMG_7294.jpg')
    process_image(img, 9, 0.2)
