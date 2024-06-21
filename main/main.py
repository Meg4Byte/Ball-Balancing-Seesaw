import time
import VL53L0X
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

SERVO_PIN = 21                  #Servo pin (you can use any free GPIO pin just make sure to change this )
DEFAULT_DUTY_CYCLE = 7.5        #Default duty cycle
MIN_DUTY_CYCLE = 7.30           #Minimum duty cycle
MAX_DUTY_CYCLE = 7.850          #Maximum duty cycle

GPIO.setup(SERVO_PIN, GPIO.OUT)
SERVO_PWM = GPIO.PWM(SERVO_PIN, 50)
SERVO_PWM.start(DEFAULT_DUTY_CYCLE)

tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

# I2C Address can change before tof.open()
# tof.change_address(0x32)
tof.open()

# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
timing = tof.get_timing()

class PIDController:

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def calculate(self, current_value : float , target_value : float ):
        """
            calculating the PID coefficents
        """

        error = target_value - current_value

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.last_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        self.last_error = error

        return output



target_pos = 200                    #Target position
current_pos = 0                 #Current position

# Initialize the PID controller
pid = PIDController(Kp=1.200, Ki=0.018, Kd=12.000)      #you may need to tune this for your project
servo_pwm.ChangeDutyCycle(DEFAULT_DUTY_CYCLE)
start_kick = False 

while True:
    
    if not start_kick :
        
        servo_pwm.ChangeDutyCycle(MAX_DUTY_CYCLE)
        time.sleep(2.5)                      #adjust as needed
        start_kick = not start_kick

    try:
        current_pos = tof.get_distance()

        # Calculate the control signal
        control_signal = pid.calculate(current_pos, target_pos)
        control_new = abs(control_signal/10)

        print(f"{control_new}")
        print(f"{current_pos}mm")
        print(f"PID = {control_signal}")

        duty_cycle = DEFAULT_DUTY_CYCLE + control_new / 13
        duty_cycle = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, control_new))

        if duty_cycle <= MIN_DUTY_CYCLE :
            duty_cycle = MIN_DUTY_CYCLE

        if duty_cycle >= MAX_DUTY_CYCLE:
            duty_cycle = MAX_DUTY_CYCLE

        print(f"Applying duty cycle: {duty_cycle}")

        servo_pwm.ChangeDutyCycle(duty_cycle)

        print(f"Diff: {target_pos - current_pos}")
    
        # Simulate a delay (replace this with the actual delay required for your system)
        time.sleep(0.05)

        if abs(target_pos-current_pos) < 0.705 :  #this is absolute tolerance , adjust as needed
            print("Target position reached")


    except KeyboardInterrupt:
        servo_pwm.ChangeDutyCycle(DEFAULT_DUTY_CYCLE)
        servo_pwm.stop()
        GPIO.cleanup()

