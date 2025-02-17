# Example of Robot Control Implementation Using the CrowBot-BOLT Robot 

[File of the program](https://github.com/Grovety/CrowBot_GRC_program/blob/main/Bolt_grc_program.ino)

[How to flash the program](https://github.com/Grovety/CrowBot_GRC_program/edit/main/how_to_install_the_program.md)

## How it works

Once you flash our program to the CrowBot  (see [how to do it](https://github.com/Grovety/CrowBot_GRC_program/blob/main/how_to_install_the_program.md)), the robot can receive commands from the DevBoard via Bluetooth.  

1. Turn on the robot and the DevBoard.  
2. Enable Bluetooth on the robot by sliding the switch.  
3. The robot will automatically connect to the DevBoard, and a connection icon will appear on the board.  

![RC2](https://github.com/user-attachments/assets/907467d8-6e76-4b28-b7d1-531149297eba)

After the connection is established, the robot can process voice commands, which are then converted into movement commands or other actions.

## Commands
| Voice Command     | Action |
|------------------|----------------------------------------------------------------------------------------------------------------------------------|
| Robot Wake Up   | The DevBoard wakes up. No action is performed by the robot. |
| Robot Sleep     | The DevBoard goes into sleep mode and stops responding to voice commands. No action is performed by the robot. |
| Start Control   | The DevBoard switches to tilt control mode. The robot enters tilt control mode and stops responding to voice movement commands. |
| Stop Control    | Tilt control mode is disabled, returning to voice control mode. |
| Go Right       | Turn right. |
| Go Left        | Turn left. |
| Go Forward     | Move forward and then stop. |
| Go Back        | Move backward and then stop. |
| Go Home        | No action. |
| Slower Speed   | No action. |
| Faster Speed   | No action. |
| Lights On      | Turn on the LED backlight on the Ultrasonic Sensor (simulating headlights). |
| Lights Off     | Turn off the LED backlight on the Ultrasonic Sensor (simulating headlights). |
| Play Music     | The robot plays a preset melody. |

## Tilt mode

To activate tilt control mode, say 'Start Control' or press the USER button. 

Once activated, the display on the DevBoard will update, and the robot will no longer respond to voice commands for movement.

> Make sure to place the DevBoard on a flat surface before starting, as it will calibrate when entering tilt control mode. When you see a circle on two intersecting lines appear on the screen, the calibration is done, and you’re ready to start controlling the robot.

Tilt the DevBoard forward, backward, left, or right to control the robot's movement. The tilt angle determines the robot's speed.

You can use voice commands 'Play Music' and 'Lights On' or 'Lights Off' in Tilt mode.

To quit say 'Stop Control' or press USER button.



