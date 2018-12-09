# ----------------------------
# LIBRARIES

# External libraries
import time
# Import the PCA9685 module.
import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# ----------------------------
# PARAMETERS
# 
# -------------------------------------------------------------------
# | REMEMBER TO ADJUST THEM TO FIT YOUR SERVO MOTOR SPECIFICATION ! |
# -------------------------------------------------------------------

SERVO_MOTOR_FREQUENCY = 50 #In Hz
SERVO_MOTOR_DUTY_CYCLE_MIN = 2.5  # Min duty cycle, in %
SERVO_MOTOR_DUTY_CYCLE_MAX = 12.5  # Max duty cycle, in %
SERVO_MOTOR_ANGLE_MIN = 0  # Min angle, in degrees
SERVO_MOTOR_ANGLE_MAX = 270  # Max angle, in degrees
PWM_BOARD_RESOLUTION = 4096 # PWM control board resolution

servo_channel = 0

# ----------------------------
# CODE

def convert_angle_to_pwm_board_step(angle):
    '''
    Transform a given angle to the proper step for the PWB board.
    Usually boards like the PCA9685 uses a discrete representation for the PWM.
    They divide the Pulse Cycle in 4096 steps, and we have to say from
    which step to which step we want the pulse to be high.

    We assume that, for servo motors, the first step is 0,
    so we just have to calculate the ending step, so to represent the given angle
    as a discrete PWM.

    To calculate the step, we first calculate the relative duty cycle of the angle.
    We then multiply the PWM_BOARD_RESOLUTION by the duty cycle (which is a percentage).
    '''
    # Check that the angle is within the limits
    if (angle < SERVO_MOTOR_ANGLE_MIN) | (angle > SERVO_MOTOR_ANGLE_MAX):
        raise ValueError('The given angle ({}) is outside the limits!'.format(angle))

    # We make sure angle is treated as a float
    servo_duty_cycle = float(angle)/(SERVO_MOTOR_ANGLE_MAX-SERVO_MOTOR_ANGLE_MIN)*(SERVO_MOTOR_DUTY_CYCLE_MAX-SERVO_MOTOR_DUTY_CYCLE_MIN) + SERVO_MOTOR_DUTY_CYCLE_MIN

    # Check that the duty cycle is within the limits
    # This check is just to be sure that also the angle parameters are set correctly
    if (servo_duty_cycle < SERVO_MOTOR_DUTY_CYCLE_MIN) | (servo_duty_cycle > SERVO_MOTOR_DUTY_CYCLE_MAX):
        raise ValueError('The given servo duty cycle ({}) is outside the limits!'.format(servo_duty_cycle))

    # The step must be an integer, as the board does not accept floats
    pwm_step = int(PWM_BOARD_RESOLUTION*servo_duty_cycle/100)

    print('angle: {}'.format(angle))
    print('servo_duty_cycle: {}'.format(servo_duty_cycle))
    print('pwm_step: {}'.format(pwm_step))

    return pwm_step

def move_servo(pwm, servo_channel, angle):
    '''
    We move the specified servo to the given angle.
    '''

    pwm_step = convert_angle_to_pwm_board_step(angle)

    pwm.set_pwm(servo_channel, 0, pwm_step)


print('Initialize PWM board controller')
# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

print('Set frequency')
# Set the frequency
pwm.set_pwm_freq(SERVO_MOTOR_FREQUENCY)

print('Move the servo')
move_servo(pwm, servo_channel, 0)
time.sleep(3)
move_servo(pwm, servo_channel, 180)
time.sleep(3)
move_servo(pwm, servo_channel, 90)
time.sleep(3)
move_servo(pwm, servo_channel, 110)
time.sleep(3)