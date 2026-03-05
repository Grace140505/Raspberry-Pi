from picamera2 import Picamera2
import cv2 as cv
import numpy as np
import time
import RPi.GPIO as GPIO

'''Pin Declarations (BCM Mode)'''
#Left motors
IN1 = 17
IN2 = 27
#Right motors
IN3 = 22
IN4 = 23
#PWM
ENA = 18
ENB = 19
#Encoders
LEFT_ENCODER = 5
RIGHT_ENCODER = 6

'''Global Constants'''
WHEEL_CIRCUMFERENCE = 21.3
COUNT_PER_REV = 21
TIME_1REV = 1.95
ERROR = 10

'''Global Variables'''
pwm_left = None
pwm_right = None
LEFT_BASE_SPEED = 66
RIGHT_BASE_SPEED = 66
left_encoder_count = 0
right_encoder_count = 0



#Global variables
prev_time = time.monotonic()
# For PID
PID_state = {
    'last_error': 0,
    'integral': 0,
    'last_time': time.monotonic()
}
#PID Constants
KP = 0.45
KI = 0
KD = 0.05
X_CENTRE_REFERENCE = 220
Y_CENTRE_REFERENCE = 120

current_x = 220
current_y = 120

output_x = 0
# prev_dir = 'S'
contour_area = 0

def line_detection(BGR_frame,GRAYSCALE_frame):
    global output_x, prev_dir

    local_BGR_frame = BGR_frame.copy()
    local_BGR_frame = local_BGR_frame[240:480, 100:540]

    local_GRAYSCALE_frame = GRAYSCALE_frame.copy()
    local_GRAYSCALE_frame = local_GRAYSCALE_frame[240:480,100:540]

    # Image pre-processing
    blur_frame = cv.GaussianBlur(local_GRAYSCALE_frame,(3,3),0)

    # Simple Thresholding
    _ ,threshold_image= cv.threshold(blur_frame,100,255,cv.THRESH_BINARY_INV)
    ''' 
    *Note that in general, 0 is black and 255 is white in OpenCV, but with THRESH_BINARY_INV, the logic is reversed:
    Inverted binary threshold (THRESH_BINARY_INV): 
        Pixel > threshold â†’ 0 (black)
        Pixel â‰¤ threshold â†’ 255 (white)
    '''

    # Masking to get the region of interest
    ROI_frame = cv.bitwise_and(local_BGR_frame, local_BGR_frame, mask=threshold_image)
    ROI_frame = cv.bitwise_not(ROI_frame) #Black is region of interest
    cv.imshow("Binary", ROI_frame)
    
    # Contour detection
    contours, _ = cv.findContours(threshold_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Find center of the line
    if contours: # If got contours, pick the largest one and fit a line to it
        global current_x, current_y, contour_area

        # Select largest contour
        largest_contour = max(contours, key=cv.contourArea)
        contour_area = cv.contourArea(largest_contour)

        if (cv.contourArea(largest_contour) > 5500):
            #Detecting image center using moments
            M = cv.moments(largest_contour)

            if M['m00'] != 0:
                current_x = int(M['m10'] / M['m00'])
                current_y = int(M['m01'] / M['m00'])
            
            # cv.line(local_BGR_frame, (current_x, 0), (current_x, local_BGR_frame.shape[0]), (0,0,255), 3)
            processed_image = cv.drawContours(local_BGR_frame, [largest_contour], -1, (0,255,0), 2)
            PID_control()
            moveForward(LEFT_BASE_SPEED - output_x, RIGHT_BASE_SPEED + output_x)
            # if (current_y >= 220):
            #     if (output_x < 0.00):
            #         moveForward(-LEFT_BASE_SPEED,RIGHT_BASE_SPEED)
            #     elif (output_x > 0.00):
            #         moveForward(LEFT_BASE_SPEED,-RIGHT_BASE_SPEED)
            #     else:
            #         while(1):
            #             moveBackward(50)
            #             time.sleep(0.5)
            #             stop()
            # else:
            #     moveForward(LEFT_BASE_SPEED - output_x, RIGHT_BASE_SPEED + output_x)
            return processed_image
        else:
            # if ((current_y >= 220.0) and (current_x <= 220.0)):
            #     moveForward(RIGHT_BASE_SPEED + output_x , LEFT_BASE_SPEED - output_x)
            # else:
            #     moveForward(LEFT_BASE_SPEED - output_x, RIGHT_BASE_SPEED + output_x)
            # if (current_y >= 200):
            #     if (current_x > 320.0): #
            #         moveForward(-40,60)
            #         time.sleep(5)
            #         prev_dir = 'L'
            #     elif (current_x <= 320.00): #
            #         moveForward(60,-40)
            #         time.sleep(0.5)
            #         prev_dir = 'R'
            moveForward(LEFT_BASE_SPEED - output_x, RIGHT_BASE_SPEED + output_x)
            return local_BGR_frame
    else: # If no contours, return the cropped frame itself
        if (output_x > 0):
            moveForward(-40,50)
        else:
            moveForward(50,-40)

        # moveForward(LEFT_BASE_SPEED - output_x, RIGHT_BASE_SPEED 
        # if (current_y >= 
        #     if (output_x > 0): #
        #         while(1):
        #             stop()
        #         moveForward(-40,60)
        #         time.sleep(5)
        #         prev_dir = 'L'
        #     elif (output_x <= 0): #
        #         moveForward(60,-40)
        #         time.sleep(0.5)
        #         prev_dir = 'R'
        # else:
        #     None

        # moveForward(LEFT_BASE_SPEED - output_x, RIGHT_BASE_SPEED + output_x)
        return local_BGR_frame

def cal_FPS(BGR_frame):
    global prev_time

    current_time = time.monotonic()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time
    cv.putText(BGR_frame,f"FPS: {int(fps)}",(480, 50),cv.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)

def PID_control():
    global PID_state, KP, KI, KD,output_x

    # Calculate error
    error = X_CENTRE_REFERENCE - current_x
    # Calculate time difference
    current_time = time.monotonic()
    dt = current_time - PID_state['last_time']
    PID_state['last_time'] = current_time

    # Proportional term
    P_x = KP * error
    # Integral term
    PID_state['integral'] += error * dt
    I_x = KI * PID_state['integral']
    # Derivative term
    D_x = KD * (error - PID_state['last_error']) / dt if dt > 0 else 0

    # Update last error
    PID_state['last_error'] = error

    # Control output (for example, motor speed)
    output_x = P_x + I_x + D_x
    

'''Motor movement'''
def func_init():
    global pwm_left, pwm_right
    #Pin GPIO mode
    GPIO.setmode(GPIO.BCM)
    #Pin Setup
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)
    GPIO.setup(ENA, GPIO.OUT)
    GPIO.setup(ENB, GPIO.OUT)
    #PWM control

    pwm_left = GPIO.PWM(ENA, 500)
    pwm_right = GPIO.PWM(ENB, 500)
    pwm_left.start(0)
    pwm_right.start(0)
    
    #Encoder (With pull-down resistors)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    #Attached interrupt
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=encoder_callback, bouncetime=3)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=encoder_callback, bouncetime=3)

def moveForward(left_speed, right_speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    # Clamp speeds
    if (left_speed < 0):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        left_speed = max(0, min(90, abs(left_speed)))
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        left_speed = max(0, min(90, abs(left_speed)))
        
    if (right_speed < 0):
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        right_speed = max(0, min(90, abs(right_speed)))
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        right_speed = max(0, min(90, abs(right_speed)))


    pwm_left.ChangeDutyCycle(left_speed)
    pwm_right.ChangeDutyCycle(right_speed)

def moveBackward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    assert 0 <= speed <= 100, "Speed must be between 0 and 100"

    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def turn_left(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    assert 0 <= speed <= 100, "Speed must be between 0 and 100"

    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)

def turn_right(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    assert 0 <= speed <= 100, "Speed must be between 0 and 100"

    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)

def stop(): #With dynamic breaking
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
'''==============='''


'''Encoder ISR'''
def encoder_callback(channel):
    global left_encoder_count, right_encoder_count
    
    # If both encoders trigger ISR, no race condition will occur as only one will be handled at a time and both encoders increment independent variables
    if channel == LEFT_ENCODER:
        left_encoder_count += 1
    elif channel == RIGHT_ENCODER:
        right_encoder_count += 1

'''==============='''

def main():
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_preview_configuration(
            main={"size": (640, 480)}
        )
    )
    picam2.start()
    func_init()
    try:
        while True:
            frame = picam2.capture_array()
            BGR_frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
            GRAYSCALE_frame = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
            
            processed_image = line_detection(BGR_frame, GRAYSCALE_frame)

            cal_FPS(processed_image)
            # Reference center
            cv.circle(processed_image, (X_CENTRE_REFERENCE, Y_CENTRE_REFERENCE), 5, (0, 0, 255), -1)
            # Current center
            cv.circle(processed_image, (current_x, current_y), 5, (0, 255, 255), -1)
            cv.putText(processed_image,f"Area: {contour_area:.0f}",(180, 200),cv.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
            cv.putText(processed_image,f"Output: {output_x:.0f}",(180, 220),cv.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)


            #cv.imshow("Original", cv.cvtColor(frame, cv.COLOR_RGB2BGR))
            # cv.imshow("BGR_Frame", BGR_frame)
            cv.imshow("Countour", processed_image)
            
            if cv.waitKey(1) & 0xFF == 27:
                break
    finally:
        stop()
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
        cv.destroyAllWindows()
        picam2.stop()


if __name__ == "__main__":
    main()