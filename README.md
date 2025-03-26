# Упраление роботом the CrowBot-BOLT

[Быстрый старт](https://grovety.com/CrowBot/)

[Файл программы](https://github.com/Grovety/CrowBot_GRC_program/blob/main/Bolt_grc_program.ino)

[Как прошить робота](https://github.com/Grovety/CrowBot_GRC_program/blob/main/how_to_install_the_program.md)

[Список команд](https://grovety.com/CrowBot/)

## Инструкция

После того, как вы "прошьете" CrowBot  ([как это сделать](https://github.com/Grovety/CrowBot_GRC_program/blob/main/how_to_install_the_program.md)), робот станет способен принимать команды от платы по Bluetooth.  

1. Включите робота и Плату управления (например подключите к пауэрбанку или любому USB адаптеру).  
2. Включите Bluetooth на самом роботе (переключатель с обратной сторны платы).  
3. Робот автоматически подключится к плате, а на экрне платы появится значок блютус, это значит, что подключение установлено.  

![RC2](https://github.com/user-attachments/assets/907467d8-6e76-4b28-b7d1-531149297eba)

Робот готов принимать и обрабатывать голосовые команды.

Есть два режима управления CrowBot-BOLT:
1. Распознавание голосовых команд (английский язык, список команд зафиксирован). 
2. Управление наклоном платы по осям X и Y.

![RCax](https://github.com/user-attachments/assets/df23ef1b-fbf7-4044-8efd-d63a35861690)

## Голосовое управление
The board "listens" to user commands through its built-in microphones and recognizes them using a neural network. The recognized command is transmitted to CrowBOT.

### Commands
| Voice Command     | Action |
|------------------|----------------------------------------------------------------------------------------------------------------------------------|
| **ROBOT WAKE UP**   | The DevBoard wakes up. The robot performs no action. |  
| **ROBOT SLEEP**     | The DevBoard goes into sleep mode and stops responding to voice commands. The robot performs no action. |  
| **MANUAL CONTROL**  | The DevBoard switches to tilt control mode. The robot enters tilt control mode and stops responding to voice movement commands. |  
| **VOICE CONTROL**   | Tilt control mode is disabled, returning to voice control mode. |  
| **GO RIGHT**       | Turn right. |  
| **GO LEFT**        | Turn left. |  
| **GO FORWARD**     | Move forward and then stop. |  
| **GO BACK**        | Move backward and then stop. |  
| **GO HOME**        | No action. |  
| **SLOWER SPEED**   | No action. |  
| **FASTER SPEED**   | No action. |  
| **LIGHTS ON**      | Turn on the LED backlight on the Ultrasonic Sensor (simulating headlights). |  
| **LIGHTS OFF**     | Turn off the LED backlight on the Ultrasonic Sensor (simulating headlights). |  
| **PLAY MUSIC**     | The robot plays a preset melody. |  

## Tilt mode

To activate tilt control mode, say 'Manual Control' or press the USER button. 

Once activated, the display on the DevBoard will update, and the robot will no longer respond to voice commands for movement.

> Make sure to place the DevBoard on a flat surface before starting, as it will calibrate when entering tilt control mode. When you see a circle on two intersecting lines appear on the screen, the calibration is done, and you’re ready to start controlling the robot.

Tilt the DevBoard forward, backward, left, or right to control the robot's movement. The tilt angle determines the robot's speed.

You can use voice commands 'Play Music' and 'Lights On' or 'Lights Off' in Tilt mode.

To quit say 'Voice Control' or press USER button.









