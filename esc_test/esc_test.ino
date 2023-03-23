#include <sbus.h> // SBus Communication
#include <Servo.h>

#define MAX_VALUE_SBUS 1811
#define MIN_VALUE_SBUS 171
#define MIN_VALUE_THROTTLE 0
#define MIN_VALUE_ANGLE -1
#define MAX_VALUE_PWM 1

const int SBUSPin = 9;
// OneShot125 ESC pin outputs:
const int m1Pin = 3; // Motor forward
const int m2Pin = 4; // Motor right
const int m3Pin = 5; // Motor backward
const int m4Pin = 6; // Motor left

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
// General stuff
float dt;
unsigned long current_time, prev_time;
float channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;
float channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
float thro_des, roll_des, pitch_des, yaw_des;

unsigned long blink_counter, blink_delay;
bool blinkAlternate;

bfs::SbusRx sbus(&Serial3);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

// Mixer
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;

/*FUNCTIONS NEEDED*/
void getDesState()
{
    // DESCRIPTION: Normalizes desired control values to appropriate values
    /*
     * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
     * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
     * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
     * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
     * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
     */

    // Needs to changed the range, as the range of our values are betweeen 171 and 1811

    thro_des = ((channel_1_pwm - MIN_VALUE_SBUS) / (MAX_VALUE_SBUS - MIN_VALUE_SBUS)) * (MAX_VALUE_PWM - MIN_VALUE_THROTTLE) + MIN_VALUE_THROTTLE; // Between 0 and 1
    // roll_des = ((channel_2_pwm - MIN_VALUE_SBUS) / (MAX_VALUE_SBUS - MIN_VALUE_SBUS)) * (MAX_VALUE_PWM - MIN_VALUE_ANGLE) + MIN_VALUE_ANGLE;       // Between -1 and 1
    // pitch_des = ((channel_3_pwm - MIN_VALUE_SBUS) / (MAX_VALUE_SBUS - MIN_VALUE_SBUS)) * (MAX_VALUE_PWM - MIN_VALUE_ANGLE) + MIN_VALUE_ANGLE;      // Between -1 and 1
    // yaw_des = ((channel_4_pwm - MIN_VALUE_SBUS) / (MAX_VALUE_SBUS - MIN_VALUE_SBUS)) * (MAX_VALUE_PWM - MIN_VALUE_ANGLE) + MIN_VALUE_ANGLE;        // Between -1 and 1

    // roll_passthru = roll_des / 2.0;   // Between -0.5 and 0.5
    // pitch_passthru = pitch_des / 2.0; // Between -0.5 and 0.5
    // yaw_passthru = yaw_des / 2.0;     // Between -0.5 and 0.5

    // // Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0); // Between 0 and 1
    // roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;    // Between -maxRoll and +maxRoll
    // pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; // Between -maxPitch and +maxPitch
    // yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;       // Between -maxYaw and +maxYaw
    // roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    // pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    // yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void scaleCommands()
{
    // DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
    /*
     * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
     * the mixer function are scaled to 0-180 for the servo library using standard PWM.
     * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated
     * which are used to command the servos.
     */
    // Scaled to 125us - 250us for oneshot125 protocol
    m1_command_PWM = thro_des * 2200;
    m2_command_PWM = thro_des * 2200;
    m3_command_PWM = thro_des * 2200;
    m4_command_PWM = thro_des * 2200;
    // Constrain commands to motors within oneshot125 bounds
    m1_command_PWM = constrain(m1_command_PWM, 1100, 2200);
    m2_command_PWM = constrain(m2_command_PWM, 1100, 2200);
    m3_command_PWM = constrain(m3_command_PWM, 1100, 2200);
    m4_command_PWM = constrain(m4_command_PWM, 1100, 2200);
}

void getCommands()
{
    // DESCRIPTION: Get raw PWM values for every channel from the radio
    /*
     * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
     * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
     * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
     * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
     */

    bfs::SbusData data;
    int number_of_channels = 8;
    if (sbus.Read())
    {
        data = sbus.data();
        for (int i = 0; i < number_of_channels; i++)
        {
            sbusChannels[i] = data.ch[i];
        }
        sbusFailSafe = data.lost_frame;
        sbusLostFrame = data.failsafe;

        // sBus scaling below is for Taranis-Plus and X4R-SB
        //  float scale = 0.615;
        //  float bias  = 895.0;
        float scale = 1.0;
        float bias = 0.0;
        channel_1_pwm = sbusChannels[0] * scale + bias;
        channel_2_pwm = sbusChannels[1] * scale + bias;
        channel_3_pwm = sbusChannels[2] * scale + bias;
        channel_4_pwm = sbusChannels[3] * scale + bias;
        channel_5_pwm = sbusChannels[4] * scale + bias;
        channel_6_pwm = sbusChannels[5] * scale + bias;
        channel_7_pwm = sbusChannels[6] * scale + bias;
        channel_8_pwm = sbusChannels[7] * scale + bias;
    }

    // Low-pass the critical commands and update previous values
    float b = 0.7; // Lower=slower, higher=noiser
    channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
    channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
    channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
    channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
    channel_1_pwm_prev = channel_1_pwm;
    channel_2_pwm_prev = channel_2_pwm;
    channel_3_pwm_prev = channel_3_pwm;
    channel_4_pwm_prev = channel_4_pwm;
}

void commandMotors()
{
    // // DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
    // /*
    //  * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
    //  * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
    //  */
    // int wentLow = 0;
    // int pulseStart, timer;
    // int flagM1 = 0;
    // int flagM2 = 0;
    // int flagM3 = 0;
    // int flagM4 = 0;
    // int flagM5 = 0;
    // int flagM6 = 0;

    // // Write all motor pins high
    // digitalWrite(m1Pin, HIGH);
    // digitalWrite(m2Pin, HIGH);
    // digitalWrite(m3Pin, HIGH);
    // digitalWrite(m4Pin, HIGH);
    // pulseStart = micros();

    // // Write each motor pin low as correct pulse length is reached
    // while (wentLow < 6)
    // { // Keep going until final (6th) pulse is finished, then done
    //     timer = micros();
    //     if ((m1_command_PWM <= timer - pulseStart) && (flagM1 == 0))
    //     {
    //         digitalWrite(m1Pin, LOW);
    //         wentLow = wentLow + 1;
    //         flagM1 = 1;
    //     }
    //     if ((m2_command_PWM <= timer - pulseStart) && (flagM2 == 0))
    //     {
    //         digitalWrite(m2Pin, LOW);
    //         wentLow = wentLow + 1;
    //         flagM2 = 1;
    //     }
    //     if ((m3_command_PWM <= timer - pulseStart) && (flagM3 == 0))
    //     {
    //         digitalWrite(m3Pin, LOW);
    //         wentLow = wentLow + 1;
    //         flagM3 = 1;
    //     }
    //     if ((m4_command_PWM <= timer - pulseStart) && (flagM4 == 0))
    //     {
    //         digitalWrite(m4Pin, LOW);
    //         wentLow = wentLow + 1;
    //         flagM4 = 1;
    //     }
    // }
    // servo1.writeMicroseconds(m1_command_PWM); // Send signal to ESC.
    servo2.writeMicroseconds(m2_command_PWM); // Send signal to ESC.
    // servo3.writeMicroseconds(m3_command_PWM); // Send signal to ESC.
    // servo4.writeMicroseconds(m4_command_PWM); // Send signal to ESC.
}

void throttleCut()
{
    // DESCRIPTION: Directly set actuator outputs to minimum value if triggered
    /*
     * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
     * minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
     * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
     * the motors to anything other than minimum value. Safety first.
     */
    if (channel_5_pwm > 1500)
    {
        m1_command_PWM = 1500;
        m2_command_PWM = 1500;
        m3_command_PWM = 1500;
        m4_command_PWM = 1500;

        // // Uncomment if using servo PWM variables to control motor ESCs
        // s1_command_PWM = 0;
        // s2_command_PWM = 0;
        // s3_command_PWM = 0;
        // s4_command_PWM = 0;
    }
}

void loopRate(int freq)
{
    // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
     * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
     * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
     * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
     * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
     * and remain above 2kHz, without needing to retune all of our filtering parameters.
     */
    float invFreq = 1.0 / freq * 1000000.0;
    unsigned long checker = micros();

    // Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time))
    {
        checker = micros();
    }
}

void loopBlink()
{
    // DESCRIPTION: Blink LED on board to indicate main loop is running
    /*
     * It looks cool.
     */
    if (current_time - blink_counter > blink_delay)
    {
        blink_counter = micros();
        digitalWrite(13, blinkAlternate); // Pin 13 is built in LED

        if (blinkAlternate == 1)
        {
            blinkAlternate = 0;
            blink_delay = 100000;
        }
        else if (blinkAlternate == 0)
        {
            blinkAlternate = 1;
            blink_delay = 2000000;
        }
    }
}

void setupBlink(int numBlinks, int upTime, int downTime)
{
    // DESCRIPTION: Simple function to make LED on board blink as desired
    for (int j = 1; j <= numBlinks; j++)
    {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

/*MAIN CODE*/

void setup()
{
    Serial.begin(500000); // USB serial
    delay(500);
    pinMode(13, OUTPUT); // Pin 13 LED blinker on board, do not modify
                         // pinMode(m1Pin, OUTPUT);
                         // pinMode(m2Pin, OUTPUT);
                         // pinMode(m3Pin, OUTPUT);
                         // pinMode(m4Pin, OUTPUT);
    // servo1.attach(m1Pin);
    servo2.attach(m2Pin);
    // servo3.attach(m3Pin);
    // servo4.attach(m4Pin);

    // Set built in LED to turn on to signal startup
    digitalWrite(13, HIGH);
    sbus.Begin();
    // Serial.println("Sending stop signal #1...");
    // servo1.writeMicroseconds(1500); // send "stop" signal to ESC.
    // delay(7000);                    // delay to allow the ESC to recognize the stopped signal
    Serial.println("Sending stop signal #2...");
    servo2.writeMicroseconds(1500); // send "stop" signal to ESC.
    delay(7000);                    // delay to allow the ESC to recognize the stopped signal
    // Serial.println("Sending stop signal #3...");
    // servo3.writeMicroseconds(1500); // send "stop" signal to ESC.
    // delay(7000);                    // delay to allow the ESC to recognize the stopped signal
    // Serial.println("Sending stop signal #4...");
    // servo4.writeMicroseconds(1500); // send "stop" signal to ESC.
    // delay(7000);                    // delay to allow the ESC to recognize the stopped signal

    setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
}

void loop()
{
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;

    loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

    getCommands();
    Serial.print(F("Channel 5: "));
    Serial.println(channel_5_pwm);
    delay(10);

    getDesState();

    scaleCommands();
    // Serial.print(F("m1_command_PWM"));
    // Serial.print(m1_command_PWM);
    // Serial.print(F("m2_command_PWM"));
    // Serial.print(m2_command_PWM);
    // Serial.print(F("m3_command_PWM"));
    // Serial.print(m3_command_PWM);
    // Serial.print(F("m4_command_PWM"));
    // Serial.println(m4_command_PWM);
    // delay(10);
    throttleCut();

    Serial.print(F("m1_command_PWM"));
    Serial.print(m1_command_PWM);
    Serial.print(F("m2_command_PWM"));
    Serial.print(m2_command_PWM);
    Serial.print(F("m3_command_PWM"));
    Serial.print(m3_command_PWM);
    Serial.print(F("m4_command_PWM"));
    Serial.println(m4_command_PWM);
    commandMotors();
    loopRate(2000);
}