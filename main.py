import cv2
import cvzone
import numpy as np
import time
from cvzone.HandTrackingModule import HandDetector

cap = cv2.VideoCapture(0)

cap.set(3, 1920)
cap.set(4, 1080)
detector = HandDetector(detectionCon=0.8, maxHands=2, minTrackCon=0.8)
cx, cy = 960, 330
rx, ry = 320, 30
centerPoint = (cx, cy)
moveRadius = 300
rotateRadius = 300
bg = cv2.imread("sources/droneControlBg7.png", cv2.IMREAD_UNCHANGED)
drone_front = cv2.imread("sources/drone.png", cv2.IMREAD_UNCHANGED)
drone_side = cv2.imread("sources/drone_rotate.png", cv2.IMREAD_UNCHANGED)
blkbg = cv2.imread("sources/blkbg.png", cv2.IMREAD_UNCHANGED)
drone_top = cv2.imread("sources/droneUp1.png", cv2.IMREAD_UNCHANGED)
red_ring = cv2.imread("sources/redRing.png", cv2.IMREAD_UNCHANGED)
blue_rec = cv2.imread("sources/blueRec.png", cv2.IMREAD_UNCHANGED)
green_rec = cv2.imread("sources/greenRec.png", cv2.IMREAD_UNCHANGED)
yellow_rec = cv2.imread("sources/yellowRec.png", cv2.IMREAD_UNCHANGED)


class Drone:
    def __init__(self, max_rpm):
        self.max_rpm = max_rpm
        self.rpm = int(max_rpm * 10 / 100)
        self.right_front = self.rpm
        self.left_front = self.rpm
        self.right_back = self.rpm
        self.left_back = self.rpm

    def getRPM(self):
        return self.right_front, self.left_front, self.right_back, self.left_back

    def setRPM(self, rf, lf, rb, lb):
        self.right_front += rf
        self.left_front += lf
        self.right_back += rb
        self.left_back += lb

        h, w = green_rec.shape[:2]

        rpms = [[rf, (0, 140)],
                [lf, (460, 140)],
                [rb, (0, 490)],
                [lb, (460, 490)]]
        for rpm in rpms:
            if rpm[0] > 0:
                cvzone.overlayPNG(img, green_rec, rpm[1])
            if rpm[0] < 0:
                cvzone.overlayPNG(img, yellow_rec, rpm[1])


        # RPM value cannot be less than 0
        reset_rpm = list(map(lambda x: 0 if x <= 0 else x, self.getRPM()))
        self.right_front = reset_rpm[0]
        self.left_front = reset_rpm[1]
        self.right_back = reset_rpm[2]
        self.left_back = reset_rpm[3]

    def moveForward(self, value):
        got_max = True in list(map(lambda a: x >= (self.max_rpm - 100), self.getRPM()))

        if got_max:
            self.setRPM(-value, -value, 0, 0)
        else:
            self.setRPM(0, 0, value, value)

        return "Moving Forward"

    def moveBackward(self, value):
        got_max = True in list(map(lambda x: x >= (self.max_rpm - 100), self.getRPM()))

        if got_max:
            self.setRPM(0, 0, -value, -value)
        else:
            self.setRPM(value, value, 0, 0)

        return "Moving Backward"

    def moveLeft(self, value):
        got_max = True in list(map(lambda x: x >= (self.max_rpm - 100), self.getRPM()))

        if got_max:
            self.setRPM(0, -value, 0, -value)
        else:
            self.setRPM(value, 0, value, 0)

        return "Moving Left"

    def moveRight(self, value):
        got_max = True in list(map(lambda x: x >= (self.max_rpm - 100), self.getRPM()))

        if got_max:
            self.setRPM(-value, 0, -value, 0)
        else:
            self.setRPM(0, value, 0, value)

        return "Moving Right"

    def moveDownward(self, value):
        got_min = True in list(map(lambda x: x == 0, self.getRPM()))

        if not got_min:
            self.setRPM(-value, -value, -value, -value)

        return "Moving Downwards"

    def moveUpward(self, value):
        got_max = True in list(map(lambda x: x >= (self.max_rpm - 100), self.getRPM()))

        if not got_max:
            self.setRPM(value, value, value, value)

        return "Moving Upward"

    def rotateRight(self, value):
        self.setRPM(-value, 0, 0, -value)

        return "Rotating Right"

    def rotateLeft(self, value):
        self.setRPM(0, -value, -value, 0)

        return "Rotating Left"

    def hover(self, img, last_action_rotate=0):
        self.showRPMs(img)
        time.sleep(0.1)
        val = min(self.getRPM())
        if last_action_rotate:
            val = max(self.getRPM())
        self.right_front = val
        self.left_front = val
        self.right_back = val
        self.left_back = val

    def flip(self, value):
        pass

    def showRPMs(self, img):
        cv2.putText(img, str(int(self.right_front)), (50, 210), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(img, str(int(self.left_front)), (510, 210), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(img, str(int(self.right_back)), (50, 560), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(img, str(int(self.left_back)), (510, 560), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)


def pointToRpm(point, radius):
    return abs(point/radius*100)


def droneMovementControl(img, lmList, radius):
    finger1, finger2, finger3 = lmList[8][:2], lmList[12][:2], lmList[16][:2]
    handWidth = detector.findDistance(lmList[5][:2], lmList[17][:2])[0]
    finger1 = finger1[0] - cx, finger1[1] - cy
    finger2 = finger2[0] - cx, finger2[1] - cy
    finger3 = finger3[0] - cx, finger3[1] - cy
    f1_f2 = handWidth/detector.findDistance(finger1, finger2)[0]
    f2_f3 = handWidth/detector.findDistance(finger2, finger3)[0]
    no_of_fingers = 0
    out = centerPoint
    drone_img = drone_front
    mx = 2
    if f1_f2 > mx:
        out = (finger1[0]+finger2[0])//2, (finger1[1]+finger2[1])//2
        no_of_fingers = 2
        if f2_f3 > mx:
            out = finger2
            drone_img = drone_side
            no_of_fingers = 3

        length = int((out[0] ** 2 + out[1] ** 2) ** 0.5)
        if length > radius:
            ratio = length / radius
            out = list(map(lambda e: e / ratio, out))
        out = int((out[0] + cx)), int(out[1] + cy)
        out = out[0] - out[0] % 30, out[1] - out[1] % 30

    return img, drone_img, out, no_of_fingers


def droneRotationControlor(img, lmList, radius):
    finger1, finger2 = lmList[8][:2], lmList[12][:2]
    handWidth = detector.findDistance(lmList[5][:2], lmList[17][:2])[0]
    finger1 = finger1[0] - rx, finger1[1] - ry
    finger2 = finger2[0] - rx, finger2[1] - ry
    try:
        f1_f2 = handWidth/detector.findDistance(finger1, finger2)[0]
    except ZeroDivisionError:
        pass
    else:
        out = [rx, ry]
        if f1_f2 > 2:
            out[0] = (finger1[0]+finger2[0])//2
            if out[0] > radius:
                out[0] = radius
            if out[0] < -radius:
                out[0] = -radius
            out[0] += rx

        return img, out


drone = Drone(5400)


while True:
    run = "-"
    success, img = cap.read()
    cvzone.overlayPNG(img, blkbg)
    img = cv2.flip(img, 1)
    cvzone.overlayPNG(img, bg)
    pointIndex = (cx, cy)
    pointer = [rx, ry]

    hands, img = detector.findHands(img, False, False)
    drone_img = drone_front

    graphBars = 20

    for i in range(graphBars):
        x = int(cx - moveRadius + i * (moveRadius / graphBars * 2))
        y = int(cy - moveRadius + i * (moveRadius / graphBars * 2))
        top = int(cy - (moveRadius ** 2 - abs(cx - x) ** 2) ** 0.5)
        bot = int(cy + (moveRadius ** 2 - abs(cx - x) ** 2) ** 0.5)
        lft = int(cx - (moveRadius ** 2 - abs(cy - y) ** 2) ** 0.5)
        rht = int(cx + (moveRadius ** 2 - abs(cy - y) ** 2) ** 0.5)

        win = np.zeros((720, 1280, 3), np.uint8)
        win = cv2.cvtColor(win, cv2.COLOR_BGR2RGB)
        bar_color = (0, 255, 68)

        if i == graphBars // 2:
            cv2.line(win, (cx, top), (cx, bot), (0, 0, 255), 2)
            cv2.line(win, (lft, y), (rht, y), (0, 0, 255), 2)
        else:
            cv2.line(win, (x, bot), (x, top), bar_color)
            cv2.line(win, (lft, y), (rht, y), bar_color)

        img = cv2.addWeighted(img, 1, win, 0.8, 0.8)

    if hands:
        for hand in hands:
            if hand["type"] == "Right":
                cvzone.overlayPNG(img, red_ring, (cx-red_ring.shape[1]//2, cy-red_ring.shape[0]//2))
                img, drone_img, pointIndex, no_of_fingers = droneMovementControl(img, hand["lmList"], moveRadius)
                xValue = pointToRpm(pointIndex[0]-cx, moveRadius)
                yValue = pointToRpm(pointIndex[1]-cy, moveRadius)

                if no_of_fingers == 2:
                    if pointIndex[0] > cx:
                        run = drone.moveRight(xValue)
                    if pointIndex[0] < cx:
                        run = drone.moveLeft(xValue)

                elif no_of_fingers == 3:
                    if pointIndex[0] > cx:
                        run = drone.moveForward(xValue)
                    if pointIndex[0] < cx:
                        run += drone.moveBackward(xValue)

                if pointIndex[1] < cy:
                    run += ", " + drone.moveUpward(yValue)
                if pointIndex[1] > cy:
                    run += ", " + drone.moveDownward(yValue)
                drone.hover(img)

            if hand["type"] == "Left":
                cvzone.overlayPNG(img, blue_rec, (rx-blue_rec.shape[1]//2, ry-blue_rec.shape[0]//2))
                img, pointer = droneRotationControlor(img, hand["lmList"], rotateRadius)
                value = pointToRpm(pointer[0]-rx, rotateRadius)
                if pointer[0]-rx > 0:
                    drone.rotateRight(value)
                if pointer[0]-rx<0:
                    drone.rotateLeft(value)

                drone.hover(img, 1)

    else:
        drone.hover(img)

    h, w = drone_img.shape[:2]
    pointIndex = pointIndex[0] - w // 2, pointIndex[1] - h // 2
    pointer[1] = ry
    cvzone.overlayPNG(img, drone_img, pointIndex)
    cvzone.overlayPNG(img, drone_top, (245, 330))

    cv2.circle(img, pointer, 25, (255, 0, 0), cv2.FILLED)
    cv2.putText(img, run, (650, 650), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)

    cv2.imshow("Drone Controller", img)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
