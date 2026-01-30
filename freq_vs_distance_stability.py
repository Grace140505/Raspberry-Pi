import RPi.GPIO as GPIO
import time
import statistics

'''Pin Declarations (BCM Mode)'''
# Left motors
IN1 = 17
IN2 = 27
# Right motors
IN3 = 22
IN4 = 23
# PWM
ENA = 18
ENB = 19

# Encoders (D0 pins)
LEFT_ENCODER = 5
RIGHT_ENCODER = 6

# Wheel/encoder constants
WHEEL_CIRCUMFERENCE = 21.3   # cm
COUNT_PER_REV = 21# pulses per revolution

'''Global Variables'''
left = None
right = None
left_encoder_count = 0
right_encoder_count = 0

# ---------- Encoder callbacks ----------
def encoder_callback(channel):
    global left_encoder_count, right_encoder_count
    
    # If both encoders trigger ISR, no race condition will occur as only one will be handled at a time and both encoders increment independent variables
    if channel == LEFT_ENCODER:
        left_encoder_count += 1
    elif channel == RIGHT_ENCODER:
        right_encoder_count += 1

# def left_encoder_callback(channel):
#     global left_encoder_count
#     left_encoder_count += 1

# def right_encoder_callback(channel):
#     global right_encoder_count
#     right_encoder_count += 1

def reset_encoder_counts():
    global left_encoder_count, right_encoder_count
    left_encoder_count = 0
    right_encoder_count = 0

# ---------- Movement functions ----------
def moveForward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

    left.ChangeDutyCycle(speed)
    right.ChangeDutyCycle(speed)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

    left.ChangeDutyCycle(0)
    right.ChangeDutyCycle(0)

# ---------- Calculations ----------
def counts_to_distance_cm(L, R):
    avg_counts = float(L + R) / 4.0
    rotations = avg_counts / COUNT_PER_REV
    return rotations * WHEEL_CIRCUMFERENCE

def straightness_error(L, R):
    avg = (L + R) / 4.0
    if avg == 0:
        return 1.0
    return abs(L - R) / avg

# ---------- GPIO + PWM init ----------
def func_initGPIO(initial_pwm_freq_hz):
    global left, right

    GPIO.setmode(GPIO.BCM)

    # Motor pins
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)

    GPIO.setup(ENA, GPIO.OUT)
    GPIO.setup(ENB, GPIO.OUT)

    # Encoder pins (D0)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Encoder event detection
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=encoder_callback, bouncetime=3)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=encoder_callback, bouncetime=3)

    # Create PWM ONCE
    left = GPIO.PWM(ENA, initial_pwm_freq_hz)
    right = GPIO.PWM(ENB, initial_pwm_freq_hz)
    left.start(0)
    right.start(0)

# ---------- Single trial ----------
def run_trial(pwm_freq_hz, duty_speed, run_time_s):
    global left, right

    reset_encoder_counts()

    # Change PWM frequency (DON'T recreate PWM objects)
    left.ChangeFrequency(pwm_freq_hz)
    right.ChangeFrequency(pwm_freq_hz)

    start_time = time.monotonic()
    while (time.monotonic() - start_time) < run_time_s:
        moveForward(duty_speed)
        # time.sleep(0.01)

    stop()

    L = left_encoder_count
    R = right_encoder_count
    dist = counts_to_distance_cm(L, R)
    stab = straightness_error(L, R)

    return L, R, dist, stab

def main():
    FREQUENCIES = [50, 100, 200, 500, 1000, 2000]  # manipulated variable
    DUTY_SPEED = 100                          # constant
    RUN_TIME_S = 1.0                         # constant
    TRIALS_PER_FREQ = 2

    try:
        func_initGPIO(initial_pwm_freq_hz=200)

        print("\n=== PWM Frequency Experiment ===")
        print(f"Constant duty = {DUTY_SPEED}% | Run time = {RUN_TIME_S}s | Trials/freq = {TRIALS_PER_FREQ}\n")

        summary_rows = []

        for freq in FREQUENCIES:
            dists = []
            stabs = []

            print(f"--- Frequency: {freq} Hz ---")
            for t in range(1, TRIALS_PER_FREQ + 1):
                L, R, dist, stab = run_trial(freq, DUTY_SPEED, RUN_TIME_S)
                dists.append(dist)
                stabs.append(stab)

                print(f"Trial {t}: L= {L} R= {R}  Distance= {dist:.2f}cm  StraightnessErr= {stab:.3f}")
                #Delay between trials
                time.sleep(8.0)

            dist_mean = statistics.mean(dists)
            dist_sd = statistics.pstdev(dists) if len(dists) > 1 else 0.0
            stab_mean = statistics.mean(stabs)

            summary_rows.append((freq, dist_mean, dist_sd, stab_mean))
            print(f"Summary @ {freq} Hz: MeanDist = {dist_mean:.2f} cm | DistSD = {dist_sd:.2f} | MeanErr = {stab_mean:.3f}\n")

        print("\n=== FINAL SUMMARY (copy into report) ===")
        print("Freq(Hz)\tMeanDist(cm)\tDistSD\tMeanStraightnessErr")
        for freq, dm, ds, sm in summary_rows:
            if (freq >= 1000):
                print(f"{freq}\t \t {dm:.2f}\t \t {ds:.2f}\t\t{sm:.3f}")
            else:
                print(f"{freq}\t \t {dm:.2f}\t \t {ds:.2f}\t\t{sm:.3f}")

    except KeyboardInterrupt:
        print("\nProgram stopped by User")

    finally:
        try:
            stop()
            left.stop()
            right.stop()
        except:
            stop()
            pass
        GPIO.cleanup()

if __name__ == "__main__":
    main()