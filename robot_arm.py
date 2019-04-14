# ----------------------------
# LIBRARIES

# External libraries
import time
# Import the PCA9685 module.
import Adafruit_PCA9685
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import pandas

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
    
    # Angle to cover at each step of the calibration
    # We define it so we can have clear ads values between each step,
    # as the ads readings fluctuate, and too close angles will not be
    # easily distinguishable.
    SERVO_MOTOR_ANGLE_CALIBRATION_STEP = 10

    # We define how many seconds the feedback loop will
    # wait while the servo tries to reach the designated angle
    # before intervening if no movement is detected.
    # This is needed in those situations where the servo is
    # blocked but keeps trying to reach the desired angle.
    MAX_TIME_BEFORE_EMERGENCY_STOP = 1

    # Value for the alpha parameter in the calculation of
    # the exponential moving average (ema)
    # https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
    CALIBRATION_EMA_ALPHA = 0.8
    # Percentage that says when the ema converged to a
    # value that can be considered stable
    CALIBRATION_EMA_CONVERGENCE_PERCENTAGE = 0.001
    # CALIBRATION_EMA_CONVERGENCE_ABSOLUTE = 0.001

    def __init__(
        self
        , pwm_board
        , pwm_board_servo_channel
        , pwm_board_resolution
        , angle_min
        , angle_max
        , ads_board
        , ads_board_channel
        , angle_ads_value_map):
        
        if not isinstance(pwm_board, Adafruit_PCA9685.PCA9685):
            raise ValueError('The pwm_board parameter must be a Adafruit_PCA9685.PCA9685 instance.')
        
        if not isinstance(pwm_board_servo_channel, int) and pwm_board_servo_channel >= 0:
            raise ValueError('The pwm_board_servo_channel parameter must be a positive integer >= 0.')
        
        if not isinstance(pwm_board_resolution, int) and pwm_board_resolution > 0:
            raise ValueError('The pwm_board_resolution parameter must be a positive integer > 0.')
        
        if not isinstance(ads_board, ADS.ADS1115):
            raise ValueError('The ads_board parameter must be a ADS.ADS1115 instance.')
        
        if not isinstance(ads_board_channel, int) and ads_board_channel >= 0:
            raise ValueError('The ads_board_channel parameter must be a positive integer >= 0.')
        
        self.PWM_BOARD = pwm_board
        self.PWM_BOARD_SERVO_CHANNEL = pwm_board_servo_channel
        self.PWM_BOARD_RESOLUTION = pwm_board_resolution
        self.ADS_BOARD = ads_board
        self.ADS_BOARD_CHANNEL = ads_board_channel
        
        # If the Servo motor doesn't have a valid angle_ads_value_map
        # we do not allow it to move
        self.BLOCKED = True

        self.__ads_chan = AnalogIn(self.ADS_BOARD, self.ADS_BOARD_CHANNEL)

        if isinstance(angle_ads_value_map, pandas.DataFrame) \
            and angle_ads_value_map.columns.values.tolist() == ['angle','ads_value']:
            self.BLOCKED = False
            self.ANGLE_ADS_VALUE_MAP = angle_ads_value_map

            # Try to evaluate the current position of the servo
            self.current_angle = self.__evaluate_current_angle()
    
    def __evaluate_current_angle(self):
        """
        Function to evaluate the current angle based on the ads value.

        To evaluate the current angle of the servo we:
        * read the current ads value
        * use the angle_ads_value_map to estimate the angle

        Returns
		-------
            int
            The estimated angle of the servo.

        """

        current_ads_value = self.__ads_chan.value
        closest_angle = self.ANGLE_ADS_VALUE_MAP.ix[(self.ANGLE_ADS_VALUE_MAP.ads_value-current_ads_value).abs().argsort()[:1]].angle.values[0]
        
        return closest_angle
        
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
        
        # Round the angle to the closest valid one w.r.t. SERVO_MOTOR_ANGLE_CALIBRATION_STEP
        angle = int(angle) - int(angle)%self.SERVO_MOTOR_ANGLE_CALIBRATION_STEP
         
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

        # print('angle: {}\tservo_duty_cycle: {}\tpwm_step: {}'.format(angle,servo_duty_cycle,pwm_step))

        return pwm_step

    def __move(self, angle, raise_if_out_of_range = False):
        '''
        Function that moves the specified servo to the given angle.
        It has no feedback control!

        We use this function as the low-level instruction to move
        the servo used by other functions of the class if needed.
        '''

        if raise_if_out_of_range:
            if angle < self.SERVO_MOTOR_ANGLE_MIN: raise ValueError('The given angle ({}) is outside the limits!'.format(angle))
            if angle > self.SERVO_MOTOR_ANGLE_MAX: raise ValueError('The given angle ({}) is outside the limits!'.format(angle))
        else:
            # If the specified angle is out of range, we block it to the closest limit
            if angle < self.SERVO_MOTOR_ANGLE_MIN: angle = self.SERVO_MOTOR_ANGLE_MIN
            if angle > self.SERVO_MOTOR_ANGLE_MAX: angle = self.SERVO_MOTOR_ANGLE_MAX
        
        pwm_step = self.__convert_angle_to_pwm_board_step(angle)
        self.PWM_BOARD.set_pwm(self.PWM_BOARD_SERVO_CHANNEL, 0, pwm_step)

    def calibrate(self, calibration_map_path):
        
        print('CALIBRATION OF THE SERVO MOTOR')
        print('This function will move the servo motor from {} (SERVO_MOTOR_ANGLE_MIN) to {} (SERVO_MOTOR_ANGLE_MAX),'.format(self.SERVO_MOTOR_ANGLE_MIN,self.SERVO_MOTOR_ANGLE_MAX))
        print('and register the output value of the ADS board.')
        print()
        print('MAKE SURE THE SERVO IS FREE TO MOVE!')
        answer = input("Enter YES to proceed: ")
        if answer.lower() != "yes":
            print("I will not continue the calibration.")
            return

        print('Starting calibration.')

        calibration_map_angle_to_ads_value = []

        # Move to the minimum angle, in case the servo is at another angle
        self.__move(self.SERVO_MOTOR_ANGLE_MIN)

        # Initialize the exponential moving average        
        ema = self.__ads_chan.value
        
        TIME_STEP_DURATION = 0.2
        NUM_OF_TIME_STEPS = 10

        # We let the ems to be estimated over some cycles
        # before checking if it converged
        for counter in range(NUM_OF_TIME_STEPS):
            old_ema = ema
            ema = self.CALIBRATION_EMA_ALPHA*self.__ads_chan.value + (1-self.CALIBRATION_EMA_ALPHA)*ema
            
            print('value: {:.2f}\t\tema: {:.2f}\t\tdiff percentage: {:.4f}'.format(self.__ads_chan.value,ema,abs((ema-old_ema)/ema)))
            
            time.sleep(TIME_STEP_DURATION)
        
        while True:
            old_ema = ema
            ema = self.CALIBRATION_EMA_ALPHA*self.__ads_chan.value + (1-self.CALIBRATION_EMA_ALPHA)*ema
            
            print('value: {:.2f}\t\tema: {:.2f}\t\tdiff percentage: {:.4f}'.format(self.__ads_chan.value,ema,abs((ema-old_ema)/ema)))
            
            if abs((ema-old_ema)/ema) <= self.CALIBRATION_EMA_CONVERGENCE_PERCENTAGE: break
            
            time.sleep(TIME_STEP_DURATION)
            
        
        print('Moved to 0.')
        time.sleep(2)

        # Move to the minimum angle, in case the servo is at another angle
        # self.__move(self.SERVO_MOTOR_ANGLE_MIN)
        # print('Moved to 0.')
        # time.sleep(2)
        
        for angle in range(self.SERVO_MOTOR_ANGLE_MIN,self.SERVO_MOTOR_ANGLE_MAX+1, self.SERVO_MOTOR_ANGLE_CALIBRATION_STEP):
            
            self.__move(angle)

            # Initialize the exponential moving average
            ema = self.__ads_chan.value

            # We let the ems to be estimated over some cycles
            # before checking if it converged
            for counter in range(NUM_OF_TIME_STEPS):
                old_ema = ema
                ema = self.CALIBRATION_EMA_ALPHA*self.__ads_chan.value + (1-self.CALIBRATION_EMA_ALPHA)*ema
                
                print('value: {:.2f}\t\tema: {:.2f}\t\tdiff percentage: {:.4f}'.format(self.__ads_chan.value,ema,abs((ema-old_ema)/ema)))
                
                time.sleep(TIME_STEP_DURATION)
            
            while True:
                old_ema = ema
                ema = self.CALIBRATION_EMA_ALPHA*self.__ads_chan.value + (1-self.CALIBRATION_EMA_ALPHA)*ema
                
                print('value: {:.2f}\t\tema: {:.2f}\t\tdiff percentage: {:.4f}'.format(self.__ads_chan.value,ema,abs((ema-old_ema)/ema)))
                
                if abs((ema-old_ema)/ema) <= self.CALIBRATION_EMA_CONVERGENCE_PERCENTAGE: break
                
                time.sleep(TIME_STEP_DURATION)
            
            
            self.current_angle = angle

            # Read voltage and use it to evaluate if the servo is stuck
            calibration_map_angle_to_ads_value.append([angle,self.__ads_chan.value])

            print('Moved to angle {}, ads value is {}'.format(angle,self.__ads_chan.value))
        
        calibration_map_angle_to_ads_value_df = pandas.DataFrame(calibration_map_angle_to_ads_value, columns = ['angle','ads_value'])
        calibration_map_angle_to_ads_value_df.to_csv(calibration_map_path, index=False)

    def move(self, angle):
        '''
        We move the specified servo to the given angle.
        We check the ads value for feedback and use it
        to either continuing the movement or block it.

        The process is as follow:
        - divide the desired angle in steps
        - for each step:
            - move to the current step
            - check every X seconds if the servo actually moved
            by reading the ads value
            - if no movement is detected, try to move back to the
            last step

        '''

        if self.BLOCKED:
            raise ValueError('The Servo motor is blocked. Cannot perform movements!')

        print('Move to angle {}'.format(angle))

        starting_angle = self.current_angle
        versus = 'UP' if angle > starting_angle else 'DOWN'

        for partial_angle in range(starting_angle, angle, self.SERVO_MOTOR_ANGLE_CALIBRATION_STEP):
            
            self.__move(partial_angle)
            
            start_time = time.time()

            while True:
                time.sleep(0.2)
                
                evaluated_angle = self.__evaluate_current_angle()

                if (time.time()-start_time) >= MAX_TIME_BEFORE_EMERGENCY_STOP:
                    
                    if (versus == 'UP' and self.current_angle <= evaluated_angle):
                        pass

            
            self.current_angle = partial_angle

class RobotArm:

    # PARAMETERS
    PWM_BOARD_RESOLUTION = 4096 # PWM control board resolution
    SERVO_MOTOR_FREQUENCY = 50 #In Hz
    
    JOINT_0_ANGLE_ADS_VALUE_MAP_PATH = 'data/angle_ads_value_map_joint_0.csv'

    def __init__(self):
        
        print('Initialize PWM board controller')
        # Initialise the PCA9685 using the default address (0x40).
        self.__pwm = Adafruit_PCA9685.PCA9685()

        print('Set frequency')
        # Set the frequency
        self.__pwm.set_pwm_freq(self.SERVO_MOTOR_FREQUENCY)

        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__ads1 = ADS.ADS1115(self.__i2c)
        # self.__ads2 = ADS.ADS1115(i2c)
        
        self.__joints = []
        self.__joints.append(Servo(self.__pwm,0,self.PWM_BOARD_RESOLUTION,0,270,self.__ads1,0, pandas.read_csv(self.JOINT_0_ANGLE_ADS_VALUE_MAP_PATH)))
        # self.__joints.append(Servo(self.__pwm,1,self.PWM_BOARD_RESOLUTION,0,270,self.__ads1,1))
        # self.__joints.append(Servo(self.__pwm,2,self.PWM_BOARD_RESOLUTION,0,270,self.__ads1,2))
        # self.__joints.append(Servo(self.__pwm,3,self.PWM_BOARD_RESOLUTION,0,270,self.__ads1,3))
        # self.__joints.append(Servo(self.__pwm,4,self.PWM_BOARD_RESOLUTION,0,270,self.__ads2,0))
        # self.__joints.append(Servo(self.__pwm,5,self.PWM_BOARD_RESOLUTION,0,270,self.__ads2,1))

    def move(self):
        
        print('Move the servo')
        self.__joints[0].move(0)
        # self.__joints[1].move(0)
        time.sleep(3)
        
        self.__joints[0].move(180)
        # self.__joints[1].move(90)
        time.sleep(3)
        
        self.__joints[0].move(90)
        # self.__joints[1].move(270)
        time.sleep(3)
        
        self.__joints[0].move(120)
        # self.__joints[1].move(200)
        time.sleep(3)

def calibrate_servos():
    
    PWM_BOARD_RESOLUTION = 4096 # PWM control board resolution
    SERVO_MOTOR_FREQUENCY = 50 #In Hz
    JOINT_0_ANGLE_ADS_VALUE_MAP_PATH = 'data/angle_ads_value_map_joint_0.csv'

    print('Initialize PWM board controller')
    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685()

    print('Set frequency')
    # Set the frequency
    pwm.set_pwm_freq(SERVO_MOTOR_FREQUENCY)

    i2c = busio.I2C(board.SCL, board.SDA)
    ads1 = ADS.ADS1115(i2c)

    servo0 = Servo(pwm,0,PWM_BOARD_RESOLUTION,0,270,ads1,0, None)

    servo0.calibrate(JOINT_0_ANGLE_ADS_VALUE_MAP_PATH)

if __name__ == "__main__":
    
    # robot_arm = RobotArm()
    # robot_arm.move()

    calibrate_servos()
