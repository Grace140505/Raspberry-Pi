'''
PROBLEMS:
1) The encoder have the tendency to over calculate, which leads to:
-> Over calculation of speed
-> Over calculation of distance
'''





import RPi.GPIO as GPIO
import time


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
COUNT_PER_REV = 21 # 2X Resolution
TIME_1REV = 1.95
ERROR = 10
POS_KP = 2.6

'''Global Variables'''
left = None
right = None
left_encoder_count = 0
right_encoder_count = 0




def moveForward(speedL, speedR):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    assert 0 <= speedL <= 100, "Speed must be between 0 and 100"
    assert 0 <= speedR <= 100, "Speed must be between 0 and 100"

    left.ChangeDutyCycle(speedL)
    right.ChangeDutyCycle(speedR)

def moveBackward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    assert 0 <= speed <= 100, "Speed must be between 0 and 100"

    left.ChangeDutyCycle(speed)
    right.ChangeDutyCycle(speed)

def turn_left(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    assert 0 <= speed <= 100, "Speed must be between 0 and 100"

    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)

def turn_right(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

    assert 0 <= speed <= 100, "Speed must be between 0 and 100"

    right.ChangeDutyCycle(speed)
    left.ChangeDutyCycle(speed)

def stop(): #With dynamic breaking
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

    left.ChangeDutyCycle(100)
    right.ChangeDutyCycle(100)

def func_initGPIO():
    global left,right
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
    left = GPIO.PWM(ENA, 100)
    right = GPIO.PWM(ENB, 100)
    left.start(0)
    right.start(0)
    #Encoder (With pull-down resistors)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    #Attached interrupt
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=encoder_callback, bouncetime=2)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=encoder_callback, bouncetime=2)

def encoder_callback(channel):
    global left_encoder_count, right_encoder_count
    
    # If both encoders trigger ISR, no race condition will occur as only one will be handled at a time and both encoders increment independent variables
    if channel == LEFT_ENCODER:
        left_encoder_count += 1
    elif channel == RIGHT_ENCODER:
        right_encoder_count += 1

def func_calculateDistance():
    #Calculate distance traveled based on encoder counts
    distance = float((left_encoder_count + right_encoder_count)) / 4.0 * WHEEL_CIRCUMFERENCE / COUNT_PER_REV
    return distance

def func_calculateSpeed(distance, prev_distance, time):
    #Calculate speed based on time for one revolution
    return ((distance - prev_distance) / time)

def main():    
    try:
        func_initGPIO()

        print("Enter desired mode:\n1) Target distance to travel\n2) Target angle to turn\n")
        status = input(">>")

        match status:
            case '1':
                target_distance = float(input("Enter distance in cm (Min Distance: 80cm)\n>> "))
                target_speed = float(input("Enter speed in cm/s (Max Speed: 55cm/s | Min Speed: 50cm/s)\n*Note that min and max speed " \
                "will increase as distance increases\n>> "))


                start_time = time.monotonic()
                prev_time = start_time
                prev_distance = 0.0

                while ((current_distance := func_calculateDistance()) < (target_distance - ERROR - 0.2)):

                    # if (time.monotonic() - prev_time) >= 0.1:
                        current_speed = func_calculateSpeed(current_distance, prev_distance, time.monotonic() - prev_time)
                        delta = current_speed - target_speed

                        if (delta == 0): 
                            pass
                        elif (delta < 0): #Slower than target speed
                            speedR = round(abs(60 + (delta * 3.0)))
                            speedR = max(45, min(speedR, 90))

                            speedL = round(abs(65 + (delta * 3.0)))
                            speedL = max(55, min(speedL, 100))
                            moveForward(speedL, speedR)
                        elif (delta > 0): #Faster than target speed
                            speedR = round(abs(60 + (delta * 2.6)))
                            speedR = max(45, min(speedR, 90))

                            speedL = round(abs(65 + (delta * 2.6)))
                            speedL = max(55, min(speedL, 100))
                            moveForward(speedL, speedR)
                        else:
                            moveForward(75,70)

                        prev_distance = current_distance
                        prev_time = time.monotonic()
                
                stop()
                avg_speed = func_calculateSpeed(func_calculateDistance(), 0, time.monotonic() - start_time)

                print(f"\n\nTarget distance: {target_distance:.2f} cm")
                print(f"Actual distance traveled: {(func_calculateDistance() + ERROR):.2f} cm")

                print(f"Target speed: {target_speed:.2f} cm/s")
                print(f"Actual speed: {avg_speed:.2f} cm/s")

                print(f"Time taken = {time.monotonic() - start_time:.2f} seconds\n\n")
            case '2':
                angle = float(input("Enter angle in degrees: "))
                
                start_time = time.monotonic()

                while((time.monotonic() - start_time) <= (angle / 360.0 * TIME_1REV)):
                    turn_right(70)
                stop()

                print(f"Turn executed for {angle:.2f} degrees!")
            case _:
                print("Error! Invalid option selected. Terminating program...")
                return
    except KeyboardInterrupt:
        print("Program stopped by User")
        stop()
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
