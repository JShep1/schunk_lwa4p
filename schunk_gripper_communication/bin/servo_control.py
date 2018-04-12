#Example: "python servo_control 7" to close
import Adafruit_BBIO.PWM as PWM
import sys

servoPin = "P8_13"

def servo_control(a):
        PWM.start(servoPin, 10.7,50)
        #while(1):
        dutyCycle=a
        if dutyCycle >= 6.8 and dutyCycle <= 10.7:
                PWM.set_duty_cycle(servoPin,dutyCycle)
                #PWM.set_frequency(servoPin,0.01)
        else:
                 print('Invalid Entry: enter value between 6.8 and 10.7')
if __name__ == "__main__":
        a = float(sys.argv[1])
        servo_control(a)


