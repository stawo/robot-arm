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

# ----------------------------
# CODE

class Servo:

    # PARAMETERS
    SERVO_MOTOR_DUTY_CYCLE_MIN = 2.5  # Min duty cycle, in %
    SERVO_MOTOR_DUTY_CYCLE_MAX = 12.5  # Max duty cycle, in %
    SERVO_MOTOR_ANGLE_MIN = 0  # Min angle, in degrees
    SERVO_MOTOR_ANGLE_MAX = 270  # Max angle, in degrees
    
    def __init__(self, pwm_board, pwm_board_servo_channel, pwm_board_resolution):
        
        if not isinstance(pwm_board, Adafruit_PCA9685.PCA9685):
            raise ValueError('The pwm_board parameter must be a Adafruit_PCA9685.PCA9685 instance.')
        
        if not isinstance(pwm_board_servo_channel, int) and pwm_board_servo_channel >= 0:
            raise ValueError('The pwm_board_servo_channel parameter must be a positive integer >= 0.')
        
        if not isinstance(pwm_board_resolution, int) and pwm_board_resolution > 0:
            raise ValueError('The pwm_board_resolution parameter must be a positive integer > 0.')
        
        self.PWM_BOARD = pwm_board
        self.PWM_BOARD_SERVO_CHANNEL = pwm_board_servo_channel
        self.PWM_BOARD_RESOLUTION = pwm_board_resolution

    def __convert_angle_to_pwm_board_step(self, angle):
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
        if (angle < self.SERVO_MOTOR_ANGLE_MIN) | (angle > self.SERVO_MOTOR_ANGLE_MAX):
            raise ValueError('The given angle ({}) is outside the limits!'.format(angle))

        # We make sure angle is treated as a float
        servo_duty_cycle = float(angle)/(self.SERVO_MOTOR_ANGLE_MAX-self.SERVO_MOTOR_ANGLE_MIN)*(self.SERVO_MOTOR_DUTY_CYCLE_MAX-self.SERVO_MOTOR_DUTY_CYCLE_MIN) + self.SERVO_MOTOR_DUTY_CYCLE_MIN

        # Check that the duty cycle is within the limits
        # This check is just to be sure that also the angle parameters are set correctly
        if (servo_duty_cycle < self.SERVO_MOTOR_DUTY_CYCLE_MIN) | (servo_duty_cycle > self.SERVO_MOTOR_DUTY_CYCLE_MAX):
            raise ValueError('The given servo duty cycle ({}) is outside the limits!'.format(servo_duty_cycle))

        # The step must be an integer, as the board does not accept floats
        pwm_step = int(self.PWM_BOARD_RESOLUTION*servo_duty_cycle/100)

        print('angle: {}'.format(angle))
        print('servo_duty_cycle: {}'.format(servo_duty_cycle))
        print('pwm_step: {}'.format(pwm_step))

        return pwm_step

    def move(self, angle):
        '''
        We move the specified servo to the given angle.
        '''

        pwm_step = self.__convert_angle_to_pwm_board_step(angle)

        self.PWM_BOARD.set_pwm(self.PWM_BOARD_SERVO_CHANNEL, 0, pwm_step)

class RobotArm:

    # PARAMETERS
    PWM_BOARD_RESOLUTION = 4096 # PWM control board resolution
    SERVO_MOTOR_FREQUENCY = 50 #In Hz
    
    def __init__(self):
        
        print('Initialize PWM board controller')
        # Initialise the PCA9685 using the default address (0x40).
        self.__pwm = Adafruit_PCA9685.PCA9685()

        print('Set frequency')
        # Set the frequency
        self.__pwm.set_pwm_freq(self.SERVO_MOTOR_FREQUENCY)

        self.__joints = []

        self.__joints.append(Servo(self.__pwm,0,self.PWM_BOARD_RESOLUTION))
        self.__joints.append(Servo(self.__pwm,6,self.PWM_BOARD_RESOLUTION))

    def move(self):
        
        print('Move the servo')
        self.__joints[0].move(0)
        self.__joints[1].move(0)
        time.sleep(3)
        
        self.__joints[0].move(180)
        self.__joints[1].move(90)
        time.sleep(3)
        
        self.__joints[0].move(90)
        self.__joints[1].move(270)
        time.sleep(3)
        
        self.__joints[0].move(120)
        self.__joints[1].move(200)
        time.sleep(3)

if __name__ == "__main__":
    
    robot_arm = RobotArm()
    robot_arm.move()