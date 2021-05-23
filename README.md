# J_robot
Arduino code for 2WD robot using ros using encoder, dc motor.

## How to run

> please type "sudo dmesg | grep tty " to find com port numer of you board.

> please run "rosrun rosserial_python serial_node.py /dev/ttyUSB1 "  in one terminal.
> please run  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py "  in another terminal to view wheels spinning.


## Hardware I used 
> motor driver : BTS7960  (43Amps)
> Motors: custom gearbox using RS775 motors with 25:1 gear ratio
> encoder : currently using absolute encoder with 20 Counts per revolution (rotary encoder ky-040)
> volatge regulator : LM2596 DC DC Voltage Regulator with display
> battery : 12V 7.2AH Battery 

> PC : Currently using my laptop, later will be using raspberry pi/ Jetson Nano


## Issues both the motors are not running smoothly, please try it on your robot and commit fix to thes code as well
