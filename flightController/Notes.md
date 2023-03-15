
# Notes for Flight Controller

## Channel - Command
1. Throttle | (Left hand Y-axis)
2. Aileron/Roll | (Left hand X-axis)
3. Elevators/Pitch | (Right-Hand Y-axis)
4. Rudder/Yaw | (Right-Hand X-axis)
5. Gear/FailSafe Throttle Cut | (Has 3 switches but only 2 modes (OFF-ON-ON))
6. Aux1/No current function (Switch ON/OFF)
Will Add channel 7, so that 6 and 7 can be used to changed the mode 

## Mode Idea (Vertical,Horizontal,Spinning)
The modes will need to be triggered using 2 Binary switches (Channel 6 and 7)
| No. |              Mode             | Channel 6 | Channel 7 |
|-----|:-----------------------------:|-----------|:---------:|
| 0   | Horizontal/Regular Quadcopter | 0         | 0         |
| 1   | Vertical/X Wing               | 1         | 0         |
| 2   | Spinning Mode                 | 1         | 1         |


