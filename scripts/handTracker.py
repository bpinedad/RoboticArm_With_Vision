#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time
import math

#ROS dependencies
import rospy
from std_msgs.msg import Int32MultiArray    
    
class handDetector():

    #Initialization with default parameters
    def __init__(self, mode = False, maxHands = 1, detectionCon = 0.5, trackCon = 0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        
    #
    def findHands(self,img, draw = True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo = 0, draw = True):

        handPoints = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                handPoints.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 3, (255, 0, 255), cv2.FILLED)
        return handPoints

def getDistance (x1, x2, y1, y2):
    return math.floor(math.sqrt( (x2-x1)**2 + (y2-y1)**2 ))

def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0)
    detector = handDetector()

    pub = rospy.Publisher('hand_status', Int32MultiArray, queue_size=10)
    rospy.init_node('hands', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        ret, img = cap.read()
        H, W, channels = img.shape
        mirror = cv2.flip(img, 1)
        img = detector.findHands(mirror)

        handPoints = detector.findPosition(img)

        #Ways to move
        if len(handPoints) != 0:
            #Distance of hand from cam is with wrist point (0) and thumb start (1)
            #Max 50 and min 25? need to test
            distanceCam = getDistance(handPoints[0][1],handPoints[1][1],handPoints[0][2],handPoints[1][2])
            
            #Closed hand use tip of middle finger (12) and relagion to wrsit point (0)
            #More than 100 is open, less than that is closed
            hand = 'Open' if getDistance(handPoints[0][1],handPoints[12][1],handPoints[0][2],handPoints[12][2]) > 100 else 'Close'

            #Using width and height of frame determine hand position using middle point (9)
            height = 'Up' if handPoints[9][2] < H/2 else 'Down'
            side = 'Left' if handPoints[9][1] < W/2 else 'Right'

            #print(f'Distance: {distanceCam} Height: {height} Side: {side} Hand: {hand}')

            #Goes from 0 to W, servo should be from 0 to 180
            s1 = math.floor((handPoints[9][1])*180/(W))
            #Goes from 0 to H, servo should be from 0 to 180
            s2 = math.floor((handPoints[9][2])*180/(H))
            #Goes from 25 to 50, servo should be from 35 to 180
            s3 = math.floor(distanceCam * (180 - 25) / 50)+25
            #Goes from 0 to 1, just open or close
            s4 = 3 if hand == 'Open' else 108

            #Round to multiples of 5
            s1 += s1%3
            s2 += s2%3
            s3 += s3%3

            msg = Int32MultiArray()
            msg.data = [s1,s2,s3,s4]
            pub.publish(msg)

        cv2.imshow("Image", img)
        if cv2.waitKey(1) == 27:
            break


if __name__ == "__main__":
    main()