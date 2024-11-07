Robot Controlled by NUCLEO-L476RG Board

The robot is controlled by a NUCLEO-L476RG board and includes the following components:

- Sonar: Used to detect obstacles and guide evasion.
- Servomotors: Four servos control the movement of the left and right "ankles."
- Bluetooth Module: Enables remote control of the robot.
- 
Main Objectives of the Project

1) Bluetooth Control: The robot can move forward, backward, turn left, and turn right.
2) Fixed Speed: The robot operates at a constant speed of 20 cm/s.
3) Obstacle Avoidance: The robot uses the sonar to detect obstacles, adjusts its trajectory based on the angle of impact, and follows obstacles.
4) Emergency Stop: A button allows the robot to be stopped or started at any time for safety.
5) Battery Level Check: A LED lights up if the control board’s voltage drops below 3V.
6) Debugging via UART: Debugging messages are sent to monitor the robot’s status.

General Operation

After initialization, the robot enters an infinite loop where it:

- Activates/Deactivates the Wheels: Based on the state of the emergency stop button (B1).
- Detects and Avoids Obstacles: Continuously, the sonar detects obstacles and a function adjusts the robot's path.
- Responds to Interruptions: For Bluetooth control.
- Monitors Battery: Voltage is constantly monitored, and if it drops below 3V, a LED signals the low battery.

This system ensures autonomous navigation, secure control of the robot, and effective diagnostic feedback.
