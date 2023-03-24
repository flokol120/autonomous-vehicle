import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Empty

from sensor_msgs.msg import Image
# from qualisys.msg import Subject

import cv2
from cv_bridge import CvBridge
import numpy as np
import time

import os

# node pub/sub queue length
QUEUE_LENGTH = 10

# [Image Encodings]
# mono8: Weil 8-bit greyscale, kein RGB
BW_IMAGE = 'mono8'
RGB_IMAGE = 'rgb8'

# [b/w values]

# grey value threshold for white pixels
LOWER_THRESH = 50
UPPER_THRESH = 255

# line offsets
DETECTION_STEPS = 40

# [Baselines]

# defaults and limits for pixel detection
RIGHT_BASELINE = [0.4, 0.7]
RIGHT_FIELD_BOUND = [0.1, 1.0]
LEFT_BASELINE = [0.1, 0.25]
LEFT_FIELD_BOUND = [0.0, 0.4]
# defaults and limits for pixel detection for overtaking
RIGHT_BASELINE_OVERTAKE = [0.65, 0.75]
RIGHT_FIELD_BOUND_OVERTAKE = [0.6, 1.0]
LEFT_BASELINE_OVERTAKE = [0.25, 0.42]
LEFT_FIELD_BOUND_OVERTAKE = [0, 0.45]

# [Baseline Offsets]

# offsets for next pixel
RIGHT_SEARCH_OFFSET = 40
LEFT_SEARCH_OFFSET = 40
SEARCH_OFFSET_FACTOR = 0.001 # extend the search baseline by width * X

# [Translation Matrix]
CAMERA_FOV = 1.5707963267948966
CAMERA_PITCH = -0.1

TEST = np.array([[70, 123], [630, 123], [229, 35], [433, 35]], dtype=np.float32)
TARGET = np.array([[200, 500], [700, 500], [200, 0], [700, 0]], dtype=np.float32)
MATRIX = cv2.getPerspectiveTransform(TEST, TARGET)

# [detection amounts]
lookAheadDistance = .5
farLookAheadDistance = 1


class ImageController(Node):
    image_only = False
    debug = False
    bridge = CvBridge()
    
    croppedBwImage = None

    # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
    # https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/#Create_the_Image_Publisher_Node_Python
    # https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/OpenCV-in-Python.html

    # horizont beschneiden
        # regions of interest (mittellinie, etc)
        # ggf. polynomial fahrweg berechnen 
        # -> auswertung per 'scanlines'? 
        # -> Bewegung der Linien befolgen
        # - Erosion und Dilation
        # - Canny Edge = Kantenerkennung
        # - Hough Line Transformation
        # - RANSAC

    def __init__(self, debug=False, image_only=False):
        super().__init__('image_controller')
        self.debug = debug
        self.image_only = image_only
        self.isOvertaking = False

        self.CURRENT_RIGHT_BASELINE = RIGHT_BASELINE
        self.CURRENT_RIGHT_FIELD_BOUND = RIGHT_FIELD_BOUND
        self.CURRENT_LEFT_BASELINE = LEFT_BASELINE
        self.CURRENT_LEFT_FIELD_BOUND = LEFT_FIELD_BOUND

        self.subRawImage = self.create_subscription(Image, '/camera/image_raw', self.handleImage, QUEUE_LENGTH)
        self.subReset = self.create_subscription(Empty, '/remote_controller/reset', self.resetCallback, QUEUE_LENGTH)
        self.subRecreateStack = self.create_subscription(Empty, '/remote_controller/recreate_stack', self.resetCallback, QUEUE_LENGTH)
        self.subSwitchLane = self.create_subscription(Bool, '/image_controller/switch_baselines', self.switchBaselinesCallback, QUEUE_LENGTH)

        self.pubBwImage = self.create_publisher(Image, '/image_controller/bw', QUEUE_LENGTH)
        self.pubCroppedImage = self.create_publisher(Image, '/image_controller/cropped', QUEUE_LENGTH)
        self.pubTransformedImage = self.create_publisher(Image, '/image_controller/transform', QUEUE_LENGTH)
        
        self.pubLineDetection = self.create_publisher(Image, '/image_controller/lineDetection', QUEUE_LENGTH)
        
        self.pubSteer = self.create_publisher(Float64, '/steering', QUEUE_LENGTH)
        self.pubSpeed = self.create_publisher(Float64, '/speed', QUEUE_LENGTH)
        
        self.pubRoadCrossing = self.create_publisher(Bool, '/image_controller/is_road_crossing', QUEUE_LENGTH)
        self.pubMRight = self.create_publisher(Float64, '/image_controller/m_right', QUEUE_LENGTH)
        self.pubMRightFar = self.create_publisher(Float64, '/image_controller/m_right_far', QUEUE_LENGTH)
        self.pubMLeftFar = self.create_publisher(Float64, '/image_controller/m_left_far', QUEUE_LENGTH)
        self.pubMLeft = self.create_publisher(Float64, '/image_controller/m_left', QUEUE_LENGTH)
        self.pubLaneDistRight = self.create_publisher(Float64, '/image_controller/lane_dist_right', QUEUE_LENGTH)
        self.pubLaneDistLeft = self.create_publisher(Float64, '/image_controller/lane_dist_left', QUEUE_LENGTH)
        self.pubTick = self.create_publisher(Empty, '/image_controller/tick', QUEUE_LENGTH)

    # update current baselines and regions of interests based on current lane / overtake
    def switchBaselinesCallback(self, msg):
        if msg.data:
            self.CURRENT_RIGHT_BASELINE = RIGHT_BASELINE
            self.CURRENT_RIGHT_FIELD_BOUND = RIGHT_FIELD_BOUND
            self.CURRENT_LEFT_BASELINE = LEFT_BASELINE
            self.CURRENT_LEFT_FIELD_BOUND = LEFT_FIELD_BOUND
            self.isOvertaking = False
        else:
            self.CURRENT_RIGHT_BASELINE = RIGHT_BASELINE_OVERTAKE
            self.CURRENT_RIGHT_FIELD_BOUND = RIGHT_FIELD_BOUND_OVERTAKE
            self.CURRENT_LEFT_BASELINE = LEFT_BASELINE_OVERTAKE
            self.CURRENT_LEFT_FIELD_BOUND = LEFT_FIELD_BOUND_OVERTAKE
            self.isOvertaking = True

    def handleImage(self, msg):
        self.debugBegin()

        self.pubTick.publish(Empty()) # ticker

        self.rawImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding=RGB_IMAGE)

        # convert to black and white
        self.bwImage = self.bwConverter(self.rawImage)
        self.publishImage(self.pubBwImage, self.bwImage, BW_IMAGE)

        # crop image
        self.croppedBwImage = self.cropping(self.bwImage)
        self.publishImage(self.pubCroppedImage, self.croppedBwImage, BW_IMAGE)

        # transform image
        self.transformedImage = self.transformer(self.croppedBwImage)
        self.publishImage(self.pubTransformedImage, self.transformedImage, BW_IMAGE)

        # line detection
        self.lineDetectionImage = self.lineDetection(self.transformedImage, self.transformedImage)
        self.publishImage(self.pubLineDetection, self.lineDetectionImage, RGB_IMAGE)

        self.debugEnd()

    # publish provided image after converting it to an image message
    def publishImage(self, publisher, image, encoding):
        image_message = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        publisher.publish(image_message)

    # force exiting overtake mode if reset is called
    def resetCallback(self, _):
        self.CURRENT_RIGHT_BASELINE = RIGHT_BASELINE
        self.CURRENT_RIGHT_FIELD_BOUND = RIGHT_FIELD_BOUND
        self.CURRENT_LEFT_BASELINE = LEFT_BASELINE
        self.CURRENT_LEFT_FIELD_BOUND = LEFT_FIELD_BOUND
        self.isOvertaking = False

    # gaussian blur and threshold-based black/white image
    def bwConverter(self, image):
        image = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),(5, 5),0)
        (_, image) = cv2.threshold(image, LOWER_THRESH, UPPER_THRESH, cv2.THRESH_BINARY)
        return image

    # crop image to cut out horizon
    def cropping(self, bwImage):
        (height, width) = (bwImage.shape[0], bwImage.shape[1])

        bwImage = bwImage[int(height * 0.6):height, 0:width]

        self.pdebug("Cropping")
        self.pdebug("\tHeight: %d, Width: %d" % (height, width))
        self.pdebug("")
        
        return bwImage

    # transform the image using predetermined matrices
    def transformer(self, image):
        (height, width) = (image.shape[0] + 400, image.shape[1] + 400)

        self.pdebug("Transformer")
        self.pdebug("\tHeight: %d, Width: %d" % (height, width))
        self.pdebug("")

        return cv2.warpPerspective(image, MATRIX, (width, height))
    
    # detect lanes within regions of intrests
    def lineDetection(self, image, lineImage):
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            
        # [LEFT]
        (yLeft, xLeft, yLeftFar, xLeftFar) = self.processLines(lineImage, image, False)
        
        # calculate incline for left line using yLeft, xLeft
        mLeft = 0.0
        if len(xLeft) > 1:
            A_Left = np.vstack([xLeft, np.ones(len(xLeft))]).T
            mLeft, _ = np.linalg.lstsq(A_Left, yLeft, rcond=None)[0]

        # calculate incline for left line using yLeftFar, xLeftFar    
        mLeftFar = 0.0
        if len(xLeftFar) > 1:
            A_Left_far = np.vstack([xLeftFar, np.ones(len(xLeftFar))]).T
            mLeftFar, _ = np.linalg.lstsq(A_Left_far, yLeftFar, rcond=None)[0]

        # publish inclines for left line
        mLeftFarFloat = Float64()
        mLeftFarFloat.data = mLeftFar
        self.pubMLeftFar.publish(mLeftFarFloat)

        mLeftFloat = Float64()
        mLeftFloat.data = mLeft
        self.pubMLeft.publish(mLeftFloat)

        # calculate distance to desired lane positioning
        distanceToLeft = self.getDistance(image.shape[1], yLeft)
        # publish value
        laneDistFloatLeft = Float64()
        laneDistFloatLeft.data = distanceToLeft
        self.pubLaneDistLeft.publish(laneDistFloatLeft)
        
        # [RIGHT]
        
        (yRight, xRight, yRightFar, xRightFar) = self.processLines(lineImage, image, True)
        # axis values
            
        # calculate incline for left line using yRight, xRight
        mRight = 0.0
        if len(xRight) > 1:
            A_Right = np.vstack([xRight, np.ones(len(xRight))]).T
            mRight, _ = np.linalg.lstsq(A_Right, yRight, rcond=None)[0]
            
        # calculate incline for left line using yRightFar, xRightFar   
        mRightFar = 0.0
        if len(xRightFar) > 1:
            A_Right_far = np.vstack([xRightFar, np.ones(len(xRightFar))]).T
            mRightFar, _ = np.linalg.lstsq(A_Right_far, yRightFar, rcond=None)[0]

        # publish inclines for right lane
        mRightFloat = Float64()
        mRightFloat.data = mRight
        self.pubMRight.publish(mRightFloat)

        mRightFarFloat = Float64()
        mRightFarFloat.data = mRightFar
        self.pubMRightFar.publish(mRightFarFloat)

        # calculate distance to desired lane positioning
        distanceToRight = self.getDistance(image.shape[1], yRight)
        # publish value
        laneDistFloatRight = Float64()
        laneDistFloatRight.data = distanceToRight
        self.pubLaneDistRight.publish(laneDistFloatRight)

        return image
    
    # process array of x positions of one lane
    def processLines(self, input, output, isRightLine):

        # geht height and width of input image
        (height, width) = (input.shape[0], input.shape[1])

        # how many pixels should we skip per step?
        stepDistance = int(height / DETECTION_STEPS)

        # init points
        (pointsX, pointsY) = ([] , [])
        (pointsXFar, pointsYFar) = ([], [])
        (currPointsX, currPointsY) = (pointsX, pointsY)

        # select current base lines
        baseLine = [int(width * self.CURRENT_RIGHT_BASELINE[0]), int(width * self.CURRENT_RIGHT_BASELINE[1])] if isRightLine else [int(width * self.CURRENT_LEFT_BASELINE[0]), int(width * self.CURRENT_LEFT_BASELINE[1])]
        
        # max values for baseline, select current left and right bounds
        leftBound = 0
        rightBound = 0
        if isRightLine:
            (leftBound, rightBound) = (int(width * self.CURRENT_RIGHT_FIELD_BOUND[0]), int(width * self.CURRENT_RIGHT_FIELD_BOUND[1]) - 2)
        else:
            (leftBound, rightBound) = (int(width * self.CURRENT_LEFT_FIELD_BOUND[0]), int(width * self.CURRENT_LEFT_FIELD_BOUND[1]))
            
        # draw bounds for RQT visualization
        output = cv2.line(output, (leftBound, 0), (leftBound, height), (0 if isRightLine else 100, 255, 0), 2)
        output = cv2.line(output, (rightBound, 0), (rightBound, height), (0 if isRightLine else 100, 0, 255), 2)

        # select initial offset
        initialOffset = RIGHT_SEARCH_OFFSET if isRightLine else LEFT_SEARCH_OFFSET
        currentSearchOffset = initialOffset
        # iterate over all steps
        for i in range(1, int(DETECTION_STEPS)):
            if len(pointsX) >= int(DETECTION_STEPS * lookAheadDistance):
                (currPointsX, currPointsY) = (pointsXFar, pointsYFar)
            if(len(currPointsX) >= int(DETECTION_STEPS * max(lookAheadDistance, farLookAheadDistance))):
                break
            # only select the current line to look at
            currentLine = input[height - stepDistance * i, 0:width]
            currentHeight = height - i * stepDistance

            # draw current baselines 
            output = cv2.circle(output, (baseLine[0], currentHeight), 2, (255, 0, 0), 2)
            output = cv2.circle(output, (baseLine[1], currentHeight), 2, (255, 0, 0), 2)

            scanLine = baseLine[0] if isRightLine else baseLine[1]

            # search for first white pixel within baseline
            while(scanLine <= baseLine[1] and scanLine >= baseLine[0] and currentLine[scanLine] != 255):
                if isRightLine:
                    scanLine += 1
                else:
                    scanLine -= 1
            
            # white pixel found
            if(scanLine > baseLine[0] and scanLine < baseLine[1] and currentLine[scanLine] == 255):
                m = 0
                if(len(currPointsX) > 0):
                    m = scanLine - currPointsX[-1]
                # determine next baselines
                baseLine[0] = max(scanLine - currentSearchOffset + int(m / 2), leftBound)
                baseLine[1] = min(scanLine + currentSearchOffset + int(m / 2), rightBound)
                # save found point to list
                currPointsX.append(scanLine)
                currPointsY.append(currentHeight)
                # add visualisation for rqt output
                output = cv2.circle(output, (scanLine, currentHeight), 2, (0, 255, 0), 2)
            else:
                # not white pixel found! Increase baselines (widen)
                baseLine[0] = max(baseLine[0] - int(SEARCH_OFFSET_FACTOR * width), leftBound)
                baseLine[1] = min(baseLine[1] + int(SEARCH_OFFSET_FACTOR * width), rightBound)
        # return all found points
        return (pointsX, pointsY, pointsXFar, pointsYFar)

    # average distance between zeroX and pointsX
    def getDistance(self, zeroX, pointsX):
        distances = []
        for i in range(min(1, len(pointsX))):
            distances.append(abs(zeroX - pointsX[i]))
        
        return sum(distances) / len(distances) if len(distances) > 0.0 else 0.0

    # [DEBUG FUNCTIONS]

    def pdebug(self, str):
        if self.debug: self.lines.append(str)

    def debugBegin(self):
        if not self.debug: return
        self.timestamp = time.time()
        self.lines = []
        self.pdebug("----------------------------------------------------")
        self.pdebug("Debug: %s, Image Only: %s" % (self.debug, self.image_only))
        self.pdebug("")

    def debugEnd(self):
        if not self.debug: return
        self.pdebug("Time taken: %f" % (time.time() - self.timestamp))
        self.pdebug("----------------------------------------------------")
        clear_terminal()
        print('\n'.join(self.lines))

def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')

def default():
    run_with_args()

def debug():
    run_with_args(debug=True)

def image():
    run_with_args(image_only=True)

def image_debug():
    run_with_args(debug=True, image_only=True)

def run_with_args(debug=False, image_only=False):
    rclpy.init()

    remoteController = ImageController(debug, image_only)

    rclpy.spin(remoteController)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    remoteController.destroy_node()
    rclpy.shutdown()




def main(args=None):
    default()

if __name__ == '__main__':
    main()
