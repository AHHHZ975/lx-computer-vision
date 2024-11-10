from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for reactive control
                            using the masked left lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    steer_matrix_left = np.ones(shape, dtype=np.uint8)

    # Making the one-third top of the matrix zero 
    # I did this to remove any outlier/noise that was observed on the duckietown in the lab.
    top_rows = int(shape[0] * 0.3)
    steer_matrix_left[:top_rows, :] = 0
    # ---
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for reactive control
                             using the masked right lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    steer_matrix_right = np.ones(shape, dtype=np.uint8)
    # Making the one-third top of the matrix zero 
    # I did this to remove any outlier/noise that was observed on the duckietown in the lab.
    top_rows = int(shape[0] * 0.3)
    steer_matrix_right[:top_rows, :] = 0    

    # ---
    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        mask_left_edge:   Masked image for the dashed-yellow line (numpy.ndarray)
        mask_right_edge:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape

    # # Load in the image and generate hsv and grayscale versions
    # imgbgr = cv2.imread('../../assets/images/visual_control/pic1_rect.png')

    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Most of our operations will be performed on the grayscale version
    image_grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Use the homography (technically Hinv) to mask out the horizon
    # When generating masks that will be used with OpenCV try and
    # instantiate them with dtype=np.uint8
    mask_ground = np.ones(image_grayscale.shape, dtype=np.uint8) # TODO: CHANGE ME

    # The image-to-ground homography associated with this image
    H = np.array([-4.137917960301845e-05, -0.00011445854191468058, -0.1595567007347241, 
                0.0008382870319844166, -4.141689222457687e-05, -0.2518201638170328, 
                -0.00023561657746150284, -0.005370140574116084, 0.9999999999999999])
    
    H = np.reshape(H,(3, 3))
    Hinv = np.linalg.inv(H)

    ############################ 
    # for i in range(image_grayscale.shape[0]):
    #     for j in range(image_grayscale.shape[1]):
    #         pixel_point = [i, j, 1]
    #         projected_point = np.matmul(H, pixel_point)
    #         if projected_point[1] < -0.08:
    #             mask_ground[i][j] = 0

    # image_grayscale = image_grayscale*mask_ground
    ####################### 

    # Identify a setting for the standard deviation that removes noise while not eliminating too much valid content.
    sigma = 4.5
    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(image_grayscale,(0,0), sigma)


    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)
    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)

    # Use the histogram above to choose the minimum threshold on the gradient magnitude. 
    # Edges whos gradient magnitude is below this threshold will be filtered out.
    threshold = 50
    mask_mag = (Gmag > threshold)

    # Using the above tool, we can identify the bounds as follows
    # Identify the lower and upper HSV bounds for the white and yellow lane markings
    # These values represent the maximum range, so they don't filter out anything
    # white_lower_hsv = np.array([0, 0, 50])         # CHANGE ME
    # white_upper_hsv = np.array([230, 100, 255])   # CHANGE ME
    # yellow_lower_hsv = np.array([10, 30, 30])        # CHANGE ME
    # yellow_upper_hsv = np.array([60, 255, 255])  # CHANGE ME

    white_lower_hsv = np.array([0, 0, 200])         # CHANGE ME
    white_upper_hsv = np.array([230, 70, 255])   # CHANGE ME
    yellow_lower_hsv = np.array([20, 30, 100])        # CHANGE ME
    yellow_upper_hsv = np.array([40, 255, 255])  # CHANGE ME

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    # Let's create masks for the left- and right-halves of the image
    width = image_grayscale.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0

    # In the left-half image, we are interested in the right-half of the dashed yellow line, which corresponds to negative x- and y-derivatives
    # In the right-half image, we are interested in the left-half of the solid white line, which correspons to a positive x-derivative and a negative y-derivative
    # Generate a mask that identifies pixels based on the sign of their x-derivative
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)

    # Let's generate the complete set of masks, including those based on color
    mask_left_edge = mask_ground * mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_ground * mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white

    return mask_left_edge, mask_right_edge
