#!/usr/bin/env python

# Test script for reading from and writing to the roboclaw

from roboclaw import Roboclaw
from time import sleep

if __name__ == "__main__":
    
    address = 0x80
    roboclaw = Roboclaw("/dev/serial0", 38400)
    roboclaw.Open()
    
    try:  
        while True:
            roboclaw.ForwardM1(address,10)
            sleep(1)
            roboclaw.ForwardM1(address,0)
            sleep(0.25)
            print(roboclaw.ReadVersion(address))

    except KeyboardInterrupt:
        roboclaw.ForwardM1(address,0)
        
