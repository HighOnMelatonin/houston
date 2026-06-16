import gpiozero
import time

BUTTONPIN = 14              ## Pin number of input button
ACTUATORPIN = 4             ## Pin number of the actuator (pin 4 is 5V pwr)
FLY = 2                     ## Flying time in seconds
HOLD = 10                   ## Holding time in seconds
DOWN = 5                    ## Landing time
DELAY = 0.5          ## Delay time      

def main():
    button = gpiozero.Button(BUTTONPIN)
    actuator = gpiozero.Motor(ACTUATORPIN)
    
    if button:
        ## Time to fly
        print("LAUNCHING")
        time.sleep(DELAY)
        actuator.forward()
        time.sleep(FLY)

        ## Hol up
        time.sleep(HOLD)        ## Holding time

        ## Time to land
        print("LANDING")
        time.sleep(DELAY)
        actuator.backward()
        time.sleep(DOWN)

if __name__ == "__main__":
    while True:
        main()
