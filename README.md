# 2WD_ros_robot
This code used here is from " NOX a House wandering robot  (https://create.arduino.cc/projecthub/robinb/nox-a-house-wandering-robot-ros-652315) " and have been updated to move without " Adafruit motor shield "

base controller sketches for a 2WD robot using Ros using encoders, dc motor.

## How to run

> first upload the sketch on your development board (esp32/ Arduino mega), do hardware connections

> please type "sudo dmesg | grep tty " to find the comm port number of your board.

> please run "rosrun rosserial_python serial_node.py /dev/ttyUSB1 "  in one terminal.

> please run  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py "  in another terminal to view wheels spinning.


## Hardware I used 

> motor driver : BTS7960  (43Amps).

> Motors: custom gearbox using RS775 motors with 25:1 gear ratio.

> encoder : currently using absolute encoder with 20 Counts per revolution (rotary encoder ky-040).

> volatge regulator : LM2596 DC DC Voltage Regulator with display.

> battery : 12V 7.2AH Battery .

> PC: Currently using my laptop, later will be using raspberry pi/ Jetson Nano.
