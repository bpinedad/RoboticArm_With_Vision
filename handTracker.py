import cv2
import mediapipe as mp
import time
import math

class handDetector():

    #Initialization with default parameters
    def __init__(self, mode = False, maxHands = 2, detectionCon = 0.5, trackCon = 0.5):
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
    cap = cv2.VideoCapture(2)
    detector = handDetector()

    while True:
        ret, img = cap.read()
        H, W, channels = img.shape
        mirror = cv2.flip(img, 1)
        img = detector.findHands(mirror)

        handPoints = detector.findPosition(img)

        #Ways to mo
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

            print(f'Distance: {distanceCam} Height: {height} Side: {side} Hand: {hand}')
            
            #print(f'Distance from Open: {distanceMiddle}')
            #print(handPoints[0])
            #print(handPoints[5])

        cv2.imshow("Image", img)
        if cv2.waitKey(1) == 27:
            break


if __name__ == "__main__":
    main()