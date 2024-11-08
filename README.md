# Robot Controlled by NUCLEO-L476RG Board

This robot is controlled by a **NUCLEO-L476RG board** and includes the following key components:

- **Sonar**: Detects obstacles and assists in evasive maneuvers.
- **Servomotors**: Four servos control the movement of the left and right "ankles."
- **Bluetooth Module**: Enables remote control of the robot.

## Main Objectives of the Project

1. **Bluetooth Control**: Allows the robot to move forward, backward, turn left, and turn right.
2. **Fixed Speed**: Operates at a constant speed of **20 cm/s**.
3. **Obstacle Avoidance**: Utilizes the sonar to detect obstacles, adjust trajectory based on angle, and follow obstacles when needed.
4. **Emergency Stop**: A button enables stopping or starting the robot anytime for safety.
5. **Battery Level Check**: A LED lights up if the control board's voltage drops below **3V**.
6. **Debugging via UART**: Debugging messages are sent to monitor the robot's status.

## General Operation

After initialization, the robot enters an infinite loop where it:

- **Activates/Deactivates Wheels**: Controlled by the emergency stop button (B1).
- **Detects and Avoids Obstacles**: Continuously monitors for obstacles using the sonar and adjusts its path accordingly.
- **Responds to Interruptions**: For Bluetooth-based control commands.
- **Monitors Battery Level**: Constantly checks the voltage, with a LED signaling low battery when it falls below 3V.

This system provides autonomous navigation, safe control, and effective diagnostic feedback, ensuring reliable and secure operation of the robot.
