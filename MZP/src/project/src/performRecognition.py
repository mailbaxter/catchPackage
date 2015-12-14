# Import the modules

import cv2
from sklearn.externals import joblib
from skimage.feature import hog
from sklearn.cross_validation import train_test_split
from sklearn.metrics import classification_report
from sklearn import datasets,svm
from nolearn.dbn import DBN
import numpy as np

def digitRecog(image):
    # Load the classifier
    clf = joblib.load("digits_cls.pkl")

    # Read the input image 
    im = cv2.imread(image)
    zipcode = []
    leftPoint = []

    # Crop the image
    #im = im[300:500, 400:1000]
    cv2.imwrite('crop.png', im)

    print(im.shape)
    # Rotate the image
    rows,cols,temp = im.shape

    M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
    im = cv2.warpAffine(im,M,(cols,rows))

    # Convert to grayscale and apply Gaussian filtering
    im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)

    # Threshold the image
    im_gray = cv2.adaptiveThreshold(im_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    ret, im_th = cv2.threshold(im_gray, 127, 255, cv2.THRESH_BINARY_INV)
    cv2.imwrite('gray.png',im_th)
    # Find contours in the image
    ctrs, hier = cv2.findContours(im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Get rectangles contains each contour
    rects = [cv2.boundingRect(ctr) for ctr in ctrs]

    # Filter out the contours that are noise
    rect_valid = []
    for rect in rects:
        if rect[2] > 5 and rect[2] < 50 and rect[3] > 5 and rect[3] < 50:
            rect_valid.append(rect)

    print(rect_valid)

    # For each rectangular region, calculate HOG features and predict
    # the digit using Linear SVM.
    for rect in rect_valid:
        leftPoint.append(rect[0])
        # Draw the rectangles
        print(rect)
        cv2.rectangle(im, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3) 
        # Make the rectangular region around the digit
        leng = int(rect[3] * 1)
        pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
        pt2 = int(rect[0] + rect[2] // 2 - leng // 2)
        print(pt1)
        print(pt2)
        print(leng)
        roi = im_th[pt1:pt1+leng, pt2:pt2+leng]
        # Resize the image
        cv2.imwrite('roi.png',roi)
        roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
        roi = cv2.dilate(roi, (3, 3))
        # Calculate the HOG features
        roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)
        nbr = clf.predict(np.array([roi_hog_fd]))
        zipcode.append(nbr[0])
        cv2.putText(im, str(int(nbr[0])), (rect[0], rect[1]),cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 3)
        
    sorting = sorted(range(len(leftPoint)),key=lambda x:leftPoint[x])
    zipcode = np.array(zipcode)
    zipcode = zipcode[np.array(sorting)]
    cv2.imwrite('withRecog.png',im)
    print(zipcode)
    return zipcode
        
    #cv2.imshow("Resulting Image with Rectangular ROIs", im)
    #cv2.waitKey()


digitRecog('camera_image.JPG')
