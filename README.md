# 2WD_ros_robot
This code used here is from [ NOX a House wandering robot ](https://create.arduino.cc/projecthub/robinb/nox-a-house-wandering-robot-ros-652315 "Original post") with  [Git repo](https://github.com/RBinsonB/Nox_robot "GitHub Repository"), updated to move on "motor controller BTS7960"  instead of  " Adafruit motor shield "

base controller sketches for a 2WD robot using Ros using encoders, dc motor.
 [I'm an inline-style link with title](https://www.google.com "Google's Homepage")

## How to run

1. first upload the sketch on your development board (esp32/ Arduino mega), do hardware connections.

2. please type "sudo dmesg | grep tty " to find the comm port number of your board.

3. please run "rosrun rosserial_python serial_node.py /dev/ttyUSB1 "  in one terminal.

4. please run  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py "  in another terminal to view wheels spinning.


## Hardware I used 

* motor driver : BTS7960  (43Amps).

* Motors: custom gearbox using RS775 motors with 25:1 gear ratio.

* encoder : currently using absolute encoder with 20 Counts per revolution (rotary encoder ky-040).

* volatge regulator : LM2596 DC DC Voltage Regulator with display.

* battery : 12V 7.2AH Battery .

* PC: Currently using my laptop, later will be using raspberry pi/ Jetson Nano.
