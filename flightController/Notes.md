
# Notes for Flight Controller

## Channel - Command
1. Throttle | (Left hand Y-axis)
2. Aileron/Roll | (Left hand X-axis)
3. Elevators/Pitch | (Right-Hand Y-axis)
4. Rudder/Yaw | (Right-Hand X-axis)
5. FailSafe Throttle Cut | (Has 3 switches but only 2 modes (OFF-ON-ON))
6. Aux1
7. Gear
8. Rudder | Kill Switch

## Mode Idea (Vertical,Horizontal,Spinning)
The modes will need to be triggered using 2 Binary switches (Channel 6 and 7)
| No. |              Mode             | Channel 6 | Channel 7 |
|-----|:-----------------------------:|-----------|:---------:|
| 0   | Horizontal/Regular Quadcopter | 0         | 0         |
| 1   | Vertical/X Wing               | 1         | 0         |
| 2   | Spinning Mode                 | 1         | 1         |

A

## Controller Parameter Defaults (Add here just in case...)

- float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
- float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
- float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
- float maxYaw = 160.0;     //Max yaw rate in deg/sec

- float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
- float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
- float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
- float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
- float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
- float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
- float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
- float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

- float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
- float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
- float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
- float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
- float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
- float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
