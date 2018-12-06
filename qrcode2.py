import cv2
import pyzbar.pyzbar as pyzbar
import math
import RPi.GPIO as GPIO
import time, sys

misstime = 0                    ##global variable to check how many times of frame it misses


def findmin(comp1,comp2,comp3,comp4):
    '''find the minimum vale of four variables'''
    if comp1 > comp2:           ##compare two values first, figure out the larger value
        comp5 = comp2
        comp7 = 2
    else:
        comp5 = comp1
        comp7 = 1
    if comp3 > comp4:           ##compare the rest of two values first, figure out the larger value
        comp6 = comp4
        comp8 = 4
    else:
        comp6 = comp3
        comp8 = 3

    if comp5 > comp6:           ##compare the larger two values and find the largest
        return comp8
    else:
        return comp7


def change_speed1(d,diff):
    k0 = 0.5
    if d <= 130:                        ##move forward at this distance
        GPIO.output(16, False)
        if d <=80:
            d=80

        if d<90:                        ##change duty cycle to control robot rotation
            k = diff*k0                 ##determine duty cycle according to the width and length of detected image
            if abs(k)>2.5:
               k=1.25*k
        elif d<100 and d>=90:
            k = 5*diff/6*k0
            if abs(k)>2.5:
                k=1.25*k
        elif d<110 and d>=100:
            k = 2*diff/3*k0
            if abs(k)>2.5:
                k=1.25*k
        elif d<120 and d>=110:
            k = 5*diff/9*k0
            if abs(k)>2.5:
                k=1.25*k
        else:
            k = 5*diff/11*k0
            if abs(k)>2.5:
                k=1.25*k
        prop = 0.2*d+64 + k
        
    elif d >= 130 and d <= 150:               ##stop motor at this distance
        prop = 100

    else:
        GPIO.output(16, True)                ##robot move backward at this distance
        if d >= 250:
            d = 250
        prop = 122.5 - 0.15*d                

    motor1_speed.ChangeDutyCycle(prop)       ##change duty cycle to change speed

def change_speed2(d,diff):
    k0 = 0.7
    if d <= 130:                            ##move forward at this distance
        GPIO.output(18, True)
        if d <=80:
            d=80

        if d<90:                            ##change duty cycle to control robot rotation
            k = diff*k0                     ##determine duty cycle according to the width and length of detected image
            if abs(k)>2.5:
                k=1.25*k
        elif d<100 and d>=90:
            k = 5*diff/6*k0
            if abs(k)>2.5:
                k=1.25*k
        elif d<110 and d>=100:
            k = 2*diff/3*k0
            if abs(k)>2.5:
                k=1.25*k
        elif d<120 and d>=110:
            k = 5*diff/9*k0
            if abs(k)>2.5:
                k=1.25*k
        else:
            k = 5*diff/11*k0
            if abs(k)>2.5:
                k=1.25*k
        prop = 0.2*d+61 - k
        
    elif d >= 130 and d <= 150:           ##stop motor at this distance
        prop = 100

    else:
        GPIO.output(18, False)            ##robot move backward at this distance
        if d >= 250:
            d = 250
        prop = 130 - 0.2*d       

    motor2_speed.ChangeDutyCycle(prop)          ##change duty cycle to change speed

def avoid_left():
    '''when robot need to turn left to avoid obstacle'''
    motor1_speed.ChangeDutyCycle(100)              ##control two motors to turn right 90 degree
    GPIO.output(18, False)
    motor2_speed.ChangeDutyCycle(75)
    time.sleep(2.4)
    GPIO.output(18, True)
    motor1_speed.ChangeDutyCycle(75)
    motor2_speed.ChangeDutyCycle(75)
    time.sleep(1)

    while True:
        time.sleep(0.1)
        sensorLowerLeft = obstacle(33,35)        ##after turning right, move forward and detect whether it has passed obstacle
        print('Lower left distance is : ', sensorLowerLeft)
        GPIO.output(18, True)
        motor1_speed.ChangeDutyCycle(75)
        motor2_speed.ChangeDutyCycle(75)
        if sensorLowerLeft > 80:                   ##after passing obstacle, turn left to back to its original track
            print('Break distance is : ', sensorLowerLeft)
            break
     
    motor1_speed.ChangeDutyCycle(100)              ##control two motors to turn left 90 degree
    motor2_speed.ChangeDutyCycle(75)
    time.sleep(0.5)
    
def avoid_right():
    '''when robot need to turn right to avoid obstacle'''
    motor2_speed.ChangeDutyCycle(100)              ##control two motors to turn left 90 degree
    GPIO.output(16, True)
    motor1_speed.ChangeDutyCycle(75)
    time.sleep(2.4)
    GPIO.output(16 , False)
    motor1_speed.ChangeDutyCycle(75)
    motor2_speed.ChangeDutyCycle(75)
    time.sleep(1)
    
    while True:
        time.sleep(0.1)
        sensorLowerRight = obstacle(36,38)        ##after turning left, move forward and detect whether it has passed obstacle
        print('Lower right distance is : ', sensorLowerRight)
        GPIO.output(16, False)
        motor1_speed.ChangeDutyCycle(75)
        motor2_speed.ChangeDutyCycle(75)
        if sensorLowerRight > 80:                 ##after passing obstacle, turn right to back to its original track
            print('Break distance is : ', sensorLowerRight)
            break
    
    motor2_speed.ChangeDutyCycle(100)             ##control two motors to turn right 90 degree
    motor1_speed.ChangeDutyCycle(75)
    time.sleep(0.5)
    

def getlen(po1,po2):
    '''find distance between two points'''
    xlen = po1.x - po2.x
    ylen = po1.y - po2.y
    plen = math.sqrt(xlen**2+ylen**2)
    return plen

def obstacle(TRIG, ECHO):
    i=0        
    avgDistance=0
    for i in range(5):
        GPIO.output(TRIG, False)                 #Set TRIG as LOW
        time.sleep(0.001)                        #Delay
        GPIO.output(TRIG, True)                  #Set TRIG as HIGH
        time.sleep(0.00001)                      #Delay of 0.00001 seconds
        GPIO.output(TRIG, False)                 #Set TRIG as LOW
        while GPIO.input(ECHO)==0:
            pass                                #Check whether the ECHO is LOW            
        pulse_start = time.time()
        while GPIO.input(ECHO)==1:              #Check whether the ECHO is HIGH
            pass
        pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start #time to get back the pulse to sensor
        distance = pulse_duration * 17150        #Multiply pulse duration by 17150 (34300/2) to get distance
        distance = round(distance,2)             #Round to two decimal points
        avgDistance=avgDistance+distance
    avgDistance=avgDistance/5                    ##average the distance value and return the value
    return avgDistance
    


def decodeDisplay(image):
    global misstime
    barcodes = pyzbar.decode(image)
    flag = 0
    for barcode in barcodes:                                 ##process every capture of frame of camera
        misstime = 0
        flag = 1
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        if barcodeData == 'zhangyh@seas.upenn.edu':         ##check if the information of QRcode mathces
            (p1,p2,p3,p4) = barcode.polygon                 ##get four points from image
            (x, y, w, h) = barcode.rect
            print("Before obstacle")
            sensorl1 = obstacle(31,29)                     ##get distance from two front ultrasonic sensors
            sensorr1 = obstacle(7,11)
            print("sensorRight Distance: {}  sensorLeft Distance:{}".format(sensorr1, sensorl1))

        
##          for rotation
            p_ref = Point(x,y)
            p1x = getlen(p1,p_ref)
            p2x = getlen(p2,p_ref)
            p3x = getlen(p3,p_ref)
            p4x = getlen(p4,p_ref)
            p_min = findmin(p1x, p2x, p3x, p4x)             ##find the point that is closest to the reference point
            if p_min == 1:                  
                diff = getlen(p1,p2) - getlen(p3,p4)             ##find the difference between two vertical lines of the image
                distance = (getlen(p1,p2) + getlen(p3,p4))/2     ##find distance between target and camera
            elif p_min == 2:
                diff = getlen(p2,p3) - getlen(p4,p1)
                distance = (getlen(p2,p3) + getlen(p2,p3))/2
            elif p_min == 3:
                diff = getlen(p3,p4) - getlen(p1,p2)
                distance = (getlen(p3,p4) + getlen(p1,p2))/2
            else:
                diff = getlen(p4,p1) - getlen(p2,p3)
                distance = (getlen(p4,p1) + getlen(p2,p3))/2


            if sensorr1 >= 40 and sensorl1 >= 40:                ##check if there is obstacle in front of robot
                print("No obstacle")                            
                change_speed1(distance,diff)
                change_speed2(distance,diff)
            elif sensorl1 < 40:                                  ##if there is obstacle, change speed and direction accordingly
                print("Obstacle on the left")
                if distance >= 130:
                    change_speed1(distance,diff)
                    change_speed2(distance,diff)
                else:
                    print("Avoid left")
                    avoid_left()
            else:
                print("Obstacle on the right")
                if distance >= 130:
                    change_speed1(distance,diff)
                    change_speed2(distance,diff)
                else:
                    print("Avoid right")
                    avoid_right()                
                

##            print("[INFO] Length: {}   diff: {}    pmin:{}    point1:{} point2:{} ".format(d1,diff,p_min,p1, p2))
##            print("[INFO] p1x: {}   p2x: {}    p3x:{}    p4x:{}".format(p1x,p2x,p3x,p4x))
##            print("[INFO] Length: {}   diff: {}    ".format(distance,diff))

##            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
##            cv2.circle(image, p1,3,(0,0,255),3)
##            cv2.circle(image, p2,10,(0,0,255),3)
##            cv2.circle(image, (x,y) ,20,(0,0,255),6)
##            text = "{} ({})".format(barcodeData, barcodeType)
##            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,.5, (0, 0, 125), 2)
##            print("[INFO] Found {} barcode: {} {} {} {}".format(barcodeType, barcodeData, distance, p1, p2))

    if flag == 0:
        misstime +=  1
##        print("[INFO] misstime: {}   ".format(misstime))

    if misstime > 5:                                 
        d1 = 140                                ##stop the robot if nothing is detected
        change_speed1(d1,0)
        change_speed2(d1,0)
##    return image


def detect():

    camera = cv2.VideoCapture(-1)                ##initialize camera
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT,400)
    

    while True:
        ret, frame = camera.read()              ##read from camera
        if ret == True:
            c = cv2.waitKey(5)
            if c == 27:                         ##press 'esc' to exit program
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            decodeDisplay(gray)                 ##image processing
            
##            print("[INFO] c: {}   ".format( c ))
##            cv2.imshow("camera", im)
            
        else:
            break

    camera.release()
    cv2.destroyAllWindows()                   ##finish program and close windows
    GPIO.cleanup()

class Point():
    '''create an object class of the Point of the detected figure'''
    def __init__(self,xParam,yParam):
        self.x=xParam
        self.y=yParam


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7,GPIO.OUT)    ##right ultrasonic sensor
    GPIO.setup(11,GPIO.IN)
    GPIO.cleanup()            ##clear all GPIO pins before running program

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7,GPIO.OUT)    ##right ultrasonic sensor
    GPIO.setup(11,GPIO.IN)

    GPIO.setup(31,GPIO.OUT)   ##left ultrasonic sensor
    GPIO.setup(29,GPIO.IN)

    GPIO.setup(33,GPIO.OUT)   ##lower left ultrasonic sensor
    GPIO.setup(35,GPIO.IN)

    GPIO.setup(36,GPIO.OUT)   ##lower right ultrasonic sensor
    GPIO.setup(38,GPIO.IN)

    GPIO.setup(12,GPIO.OUT)   ##12-16, motor on the left
    GPIO.setup(13,GPIO.OUT)   ##13-18, motor on the right
    GPIO.setup(16,GPIO.OUT)
    GPIO.setup(18,GPIO.OUT)
    GPIO.output(16, False)
    GPIO.output(18, True)
    motor1_speed = GPIO.PWM(12, 1000)
    motor2_speed = GPIO.PWM(13, 1000)
    motor1_speed.start(100)
    motor2_speed.start(100)
    print("Setup ok")
    detect()
