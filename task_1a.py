'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 1A of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ BM_1078 ]
# Author List:		[ Anupama Gautam, Madhuri Malviya, Pranil Dargaiya, Shantanu Choudhary ]
# Filename:			task_1a.py
# Functions:		detect_shapes
# 					[getShapes_Centroid ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
def getShapes_Centroid(c,color):

        info_shape = []
        info_shape.append(color)
        
        # code for shape detection
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c,0.02*peri,True) # corner points
        objCor = len(approx)
        x, y, w, h = cv2.boundingRect(approx)

        M = cv2.moments(c)
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        
        if objCor == 3:
            info_shape.append("Triangle")
        elif objCor == 4:
            aspRatio = w/float(h)
            if 0.95<aspRatio<1.05:
                info_shape.append("Square")
            else:
                info_shape.append("Rectangle")
        elif objCor == 5:
            info_shape.append("Pentagon")
        else:
            info_shape.append("Circle")

        centroid = (cx,cy)
        info_shape.append(centroid)
        return info_shape
##############################################################

def detect_shapes(img):

    """
    Purpose:
    ---
    This function takes the image as an argument and returns a nested list
    containing details of colored (non-white) shapes in that image

    Input Arguments:
    ---
    `img` :	[ numpy array ]
            numpy array of image returned by cv2 library

    Returns:
    ---
    `detected_shapes` : [ list ]
            nested list containing details of colored (non-white) 
            shapes present in image
    
    Example call:
    ---
    shapes = detect_shapes(img)
    """    
    detected_shapes = []

    ##############	ADD YOUR CODE HERE	##############

    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_red=np.array([0, 148, 102])
    upper_red=np.array([15, 255, 255])
    lower_blue=np.array([61,163,113])
    upper_blue=np.array([179,255,255])
    lower_green=np.array([60,160,100])
    upper_green=np.array([107,255,255])
    lower_orange=np.array([10,100,100])
    upper_orange=np.array([59, 255, 255])

    mask_red=cv2.inRange(imgHSV,lower_red,upper_red)
    mask_blue=cv2.inRange(imgHSV,lower_blue,upper_blue)
    mask_green=cv2.inRange(imgHSV,lower_green,upper_green)
    mask_orange=cv2.inRange(imgHSV,lower_orange,upper_orange)

    cnt_red,hierarchy_red=cv2.findContours(mask_red,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt_blue,hierarchy_blue=cv2.findContours(mask_blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt_green,hierarchy_green=cv2.findContours(mask_green,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cnt_orange,hierarchy_orange=cv2.findContours(mask_orange,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
 
    for c in cnt_red:
        detected_shapes.append(getShapes_Centroid(c,'Red'))
    for c in cnt_blue:
        detected_shapes.append(getShapes_Centroid(c,'Blue'))
    for c in cnt_green:
        detected_shapes.append(getShapes_Centroid(c,'Green'))
    for c in cnt_orange:
        detected_shapes.append(getShapes_Centroid(c,'Orange'))    
    
    ##################################################
    return detected_shapes

def get_labeled_image(img, detected_shapes):
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
    """
    Purpose:
    ---
    This function takes the image and the detected shapes list as an argument
    and returns a labelled image

    Input Arguments:
    ---
    `img` :	[ numpy array ]
            numpy array of image returned by cv2 library

    `detected_shapes` : [ list ]
            nested list containing details of colored (non-white) 
            shapes present in image

    Returns:
    ---
    `img` :	[ numpy array ]
            labelled image
    
    Example call:
    ---
    img = get_labeled_image(img, detected_shapes)
    """
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

    for detected in detected_shapes:
        colour = detected[0]
        shape = detected[1]
        coordinates = detected[2]
        cv2.putText(img, str((colour, shape)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
    return img

if __name__ == '__main__':
    
    # path directory of images in 'test_images' folder
    img_dir_path = 'test_images/'

    # path to 'test_image_1.png' image file
    file_num = 1
    img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
    
    # read image using opencv
    img = cv2.imread(img_file_path)
    
    print('\n============================================')
    print('\nFor test_image_' + str(file_num) + '.png')
    
    # detect shape properties from image
    detected_shapes = detect_shapes(img)
    print(detected_shapes)
    
    # display image with labeled shapes
    img = get_labeled_image(img, detected_shapes)
    cv2.imshow("labeled_image", img)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()
    
    choice = input('\nDo you want to run your script on all test images ? => "y" or "n": ')
    
    if choice == 'y':

        for file_num in range(1,16):
            
            # path to test image file
            img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
            
            # read image using opencv
            img = cv2.imread(img_file_path)
    
            print('\n============================================')
            print('\nFor test_image_' + str(file_num) + '.png')
            
            # detect shape properties from image
            detected_shapes = detect_shapes(img)
            print(detected_shapes)
            
            # display image with labeled shapes
            img = get_labeled_image(img, detected_shapes)
            cv2.imshow("labeled_image", img)
            cv2.waitKey(2000)
            cv2.destroyAllWindows()


