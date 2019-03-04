#include "deltaRobot.h"
#include <Iibrary.h>
#include <math.h>
#include <Servo.h> 

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

Coordinate end_effector;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

float servo_1_angle;
float servo_2_angle;
float servo_3_angle;
int servo_gripper_rotation;

int servo_1_pulse_count = 0;
int servo_2_pulse_count = 0;
int servo_3_pulse_count = 0;
int servo_4_pulse_count = 0;

int moves_array[ARRAY_LENGTH][4];//{x, y, z, gripper}
int moves_array_index = 0;

int step_delay = 0; //0ms 
float step_increment = 0.4; //0.4mm
int step_pulses = 1; //1us increments

String stringText = "";

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float boundFloat(float value, float lower, float upper){
    if(value < lower){
        value = lower;
    }
    else if(value > upper){
        value = upper;
    }
    return value;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void attach_servos(void){
    servo1.attach(SERVO_1_PIN, SERVO_1_MIN, SERVO_1_MAX); // TODO: set correct min/max values
    servo2.attach(SERVO_2_PIN, SERVO_2_MIN, SERVO_2_MAX);
    servo3.attach(SERVO_3_PIN, SERVO_3_MIN, SERVO_3_MAX);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics_1(float xt, float yt, float zt){
    zt -= SERVO_OFFSET_Z;
    
    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(L2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / L2);
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range.
//        printi("ERROR: Ball joint 1 out of range: l2pAngle = ", radsToDeg(l2pAngle));
        return false;
    }

    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2));

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //This limit assumes that L2 is greater than L1
//       printi("ERROR: Extension 1 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext));
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x);
    float theta = phi + omega;

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){
//        printi("ERROR: Servo angle 1 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_1_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics_2(float xt, float yt, float zt){
    zt -= SERVO_OFFSET_Z;
    float x = xt;
    float y = yt;
    xt = x * cos(2.0943951023931954923084289221863f) - y * sin(2.0943951023931954923084289221863f); //Rotate coordinate frame 120 degrees
    yt = x * sin(2.0943951023931954923084289221863f) + y * cos(2.0943951023931954923084289221863f);
    
    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(L2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / L2);
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range.
//        printi("ERROR: Ball joint 2 out of range: l2pAngle = ", radsToDeg(l2pAngle));        
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2));

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //This limit assumes that L2 is greater than L1
//        printi("ERROR: Extension 2 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext));
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x);
    float theta = phi + omega;

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){
//        printi("ERROR: Servo angle 2 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_2_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics_3(float xt, float yt, float zt){
    zt -= SERVO_OFFSET_Z;

    float x = xt;
    float y = yt;
    xt = x * cos(4.1887902047863909846168578443727f) - y * sin(4.1887902047863909846168578443727f); //Rotate coordinate frame 240 degrees
    yt = x * sin(4.1887902047863909846168578443727f) + y * cos(4.1887902047863909846168578443727f);

    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(L2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / L2);
    if(!(abs(l2pAngle) < 0.59341194567807205615405486128613f)){ //Prevents the angle between the ball joints and link 2 (L2) going out of range.
//        printi("ERROR: Ball joint 1 out of range: l2pAngle = ", radsToDeg(l2pAngle));
        return false;
    }
    
    float ext = sqrt(pow (zt, 2) + pow(SERVO_OFFSET_X - arm_end_x, 2));

    if(ext <= l2p - L1 || ext >= L1 + l2p){ //This limit assumes that L2 is greater than L1
//        printi("ERROR: Extension 3 out of range: ext = ", ext);
        return false;
    }
       
    float phi = acos((pow(L1, 2) + pow(ext, 2) - pow(l2p, 2)) / (2 * L1 * ext));
    float omega = atan2(zt, SERVO_OFFSET_X - arm_end_x);
    float theta = phi + omega;

    if(!(theta >= SERVO_ANGLE_MIN && theta <= SERVO_ANGLE_MAX)){
//        printi("ERROR: Servo angle 3 out of range: Angle = ", radsToDeg(theta));
        return false;
    }
    
    servo_3_angle = theta;
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inverse_kinematics(float xt, float yt, float zt){    
    if(inverse_kinematics_1(xt, yt, zt) && inverse_kinematics_2(xt, yt, zt) && inverse_kinematics_3(xt, yt, zt)){ //Calculates checks the positions are valid.
        
        end_effector.x = xt;
        end_effector.y = yt;
        end_effector.z = zt;
        
        servo_1_pulse_count = round(mapNumber(servo_1_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_1_MAX, SERVO_1_MIN));
        servo_2_pulse_count = round(mapNumber(servo_2_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_2_MAX, SERVO_2_MIN));
        servo_3_pulse_count = round(mapNumber(servo_3_angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_3_MAX, SERVO_3_MIN));

        return true;
    }
    return false;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void linear_move(float x1, float y1, float z1, float stepDist, int stepDelay){//interpolates between two points to move in a stright line (beware of physical and kinematic limits)
    //Sets the initial position variables
    float x0 = end_effector.x;
    float y0 = end_effector.y;
    float z0 = end_effector.z;
    
    //Distance change in each axis
    float xDist = x1 - x0;
    float yDist = y1 - y0;
    float zDist = z1 - z0;
    
    double totalDist = sqrt(sq(xDist) + sq(yDist) + sq(zDist));//Absolute magnitute of the distance
    int numberOfSteps = round(totalDist / stepDist);//Number of steps required for the desired step distance

    //Step size of each axis
    if(numberOfSteps == 0){
//        printi("ERROR: No change in position: numberOfSteps = ", numberOfSteps);
        return;
    }
    
    float xStep = xDist / (float)numberOfSteps;
    float yStep = yDist / (float)numberOfSteps;
    float zStep = zDist / (float)numberOfSteps;

    //Interpolation variables
    float xInterpolation;
    float yInterpolation;
    float zInterpolation;

    for(int i = 1; i <= numberOfSteps; i++){//Interpolate the points
        xInterpolation = x0 + i * xStep;
        yInterpolation = y0 + i * yStep;
        zInterpolation = z0 + i * zStep;

        inverse_kinematics(xInterpolation, yInterpolation, zInterpolation);//calculates the inverse kinematics for the interpolated values
        move_servos();
        if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void joint_move(float xt, float yt, float zt, int stepPulses, int stepDelay){//interpolates between the current and tartget joint pule counts. All joints will reach the target position at the same time.
    if(stepPulses <= 0) return;//Checks that the step size if valid
    
    //Sets variables to store the current pulse counts
    int servo1PulseCount0 = servo_1_pulse_count;
    int servo2PulseCount0 = servo_2_pulse_count;
    int servo3PulseCount0 = servo_3_pulse_count;

    if(inverse_kinematics(xt, yt, zt) == false) return;//Exit the function if the position is out of the workspace
   
    //Sets variables to store the differences in pulse counts
    int servo1PulseDiff = servo_1_pulse_count - servo1PulseCount0;
    int servo2PulseDiff = servo_2_pulse_count - servo2PulseCount0;
    int servo3PulseDiff = servo_3_pulse_count - servo3PulseCount0;
    
    int maxDiff;

    //Gets the biggest difference in pulse count
    if(abs(servo1PulseDiff) >= abs(servo2PulseDiff) && abs(servo1PulseDiff) >= abs(servo3PulseDiff)){
        maxDiff = abs(servo1PulseDiff);
    }
    else if(abs(servo2PulseDiff) >= abs(servo1PulseDiff) && abs(servo2PulseDiff) >= abs(servo3PulseDiff)){
        maxDiff = abs(servo2PulseDiff);
    }
    else if(abs(servo3PulseDiff) >= abs(servo1PulseDiff) && abs(servo3PulseDiff) >= abs(servo2PulseDiff)){
        maxDiff = abs(servo3PulseDiff);
    }
    else{
//        printi("ERROR: No difference in pulse counts. ");
        return;
    }

    float servo1Step = (float)servo1PulseDiff / (float)maxDiff;
    float servo2Step = (float)servo2PulseDiff / (float)maxDiff;
    float servo3Step = (float)servo3PulseDiff / (float)maxDiff;

    for(int i = 1; i <= maxDiff; i += stepPulses){
        servo_1_pulse_count = round(servo1PulseCount0 + i * servo1Step);
        servo_2_pulse_count = round(servo2PulseCount0 + i * servo2Step);
        servo_3_pulse_count = round(servo3PulseCount0 + i * servo3Step);
        move_servos();
        if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
    }

    //Sets the correct final position
    servo_1_pulse_count = servo1PulseCount0 + servo1PulseDiff;
    servo_2_pulse_count = servo2PulseCount0 + servo2PulseDiff;
    servo_3_pulse_count = servo3PulseCount0 + servo3PulseDiff;
    move_servos();
    if(stepDelay > 0) delay(stepDelay);//If there is a delay then delay
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void move_servos(void){
    servo1.writeMicroseconds(servo_1_pulse_count);
    servo2.writeMicroseconds(servo_2_pulse_count);
    servo3.writeMicroseconds(servo_3_pulse_count);
    servo4.writeMicroseconds(servo_4_pulse_count);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void gripper_servo(char rotation){ //The gripper servo has been modified for continuous rotation. Detaching the servo ensures it will stop when rotation is set to 0
    if(rotation < -128 || rotation > 127){
        return;
    }

    if(rotation != 0){
        if(!servo4.attached()){
            servo4.attach(SERVO_4_PIN, SERVO_4_MIN, SERVO_4_MAX);     
        }  
        servo_gripper_rotation = rotation;
        servo_4_pulse_count = mapNumber(rotation, -128, 127, SERVO_4_MIN, SERVO_4_MAX);
    }
    else if(rotation == 0 && servo4.attached()){
        servo_gripper_rotation = 0;
        servo4.detach();
        digitalWrite(SERVO_4_PIN, LOW);
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float get_battery_level_percentage(void){ //TODO: Calibrate the values for your battery
    return boundFloat(mapNumber(analogRead(BATTERY_PIN), 780, 1023, 0, 100), 0, 100); //780 = 9V = 0%, 1023 = 12.6V = 100%
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float get_battery_level_voltage(void){ //TODO: Calibrate the values for your battery
    return mapNumber(analogRead(BATTERY_PIN), 780, 1023, 9, 12.6);//780 = 9V, 1023 = 12.6V
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void report_status(){
    printi("\n---Delta robot status---\n\n");
    printi("X coordinate:\t", end_effector.x);
    printi("Y coordinate:\t", end_effector.y);
    printi("Z coordinate:\t", end_effector.z);

    printi("\nServo 1 angle:\t", radsToDeg(servo_1_angle));
    printi("Servo 2 angle:\t", radsToDeg(servo_2_angle)); 
    printi("Servo 3 angle:\t", radsToDeg(servo_3_angle));
    printi("Servo 4 rotation:\t", servo_gripper_rotation); 

    printi("\n\nServo 1 pulses:\t", servo_1_pulse_count);
    printi("Servo 2 pulses:\t", servo_2_pulse_count);
    printi("Servo 3 pulses:\t", servo_3_pulse_count);
    printi("Servo 4 pulses:\t", servo_4_pulse_count);
    
    printi("\nStep delay:\t", step_delay);
    printi("Step inrement:\t", step_increment);
    printi("Step pulse:\t", step_pulses);
    
    printi("\nArray index:\t", moves_array_index);
 
    printi("\nBattery:\t", get_battery_level_voltage(), 1, "V");
    printi("\t(", get_battery_level_percentage(), 1, "%)\n");

    printi("\nFirmware Version:\t");
    printi(FIRMWARE_VERSION);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void report_commands(void){
    printi("\n---Delta robot commands---\n\n");
    printi("Cartesian\t0x1+6 bytes\t(Format: 1xxyyzz)\n");
    printi("Abslute cartesian\t0x2+6 bytes\t(Format: 2xxyyzz)\n");
    printi("Gripper\t0x3+1 byte\t(Format: 3b)\n"); 
    printi("Jog X-axis\tx+str\t(Format: x123.4)\n");
    printi("Jog Y-axis\ty+str\t(Format: y123.4)\n");
    printi("Jog Z-axis\tz+str\t(Format: z123.4)\n");
    printi("Gripper ascii\tg+str\t(Format: g123)\n"); 
    printi("Add position\tp\t(Format: p)\n");
    printi("Clear array\tc\t(Format: c)\n");
    printi("Status\ts\t(Format: s)\n");
    printi("Set delay (ms)\td+string\t(Format: d123)\n"); 
    printi("Set step increment (mm)\ti+str\t(Format: i123.4)\n"); 
    printi("Set pulse increment (us)\tu+str\t(Format: u123.4)\n"); 
    printi("Linear execute x times\te+str\t(Format: e123)\n");
    printi("Joint execute x times\tj+str\t(Format: j123)");   
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void serialFlush(void){
    while(Serial.available() > 0) {
        char c = Serial.read();
    }
} 

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void detach_servos(void){
    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int add_position(void){
    if(moves_array_index >= 0 && moves_array_index < ARRAY_LENGTH){
        moves_array[moves_array_index][0] = end_effector.x;
        moves_array[moves_array_index][1] = end_effector.y;
        moves_array[moves_array_index][2] = end_effector.z;
        moves_array[moves_array_index][3] = get_gripper_rotation();
        moves_array_index++;//increment the index
        return 0;
    }
    return -1;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void clear_array(void){
    for(int row = 0; row < ARRAY_LENGTH; row++){
        for(int col = 0; col < 4; col++){
            moves_array[row][col] = NULL;
        }
    }
    moves_array_index = 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void execute_moves(int repeat){
    for(int i = 0; i < repeat; i++){
        for(int row = 0; row < moves_array_index; row++){
            gripper_servo(moves_array[row][3]);
            linear_move(moves_array[row][0], moves_array[row][1], moves_array[row][2], step_increment, step_delay);
        }
        if(get_battery_level_voltage() < 9.5){//9.5V is used as the cut off to allow for inaccuracies and be on the safe side.
            delay(200);
            if(get_battery_level_voltage() < 9.5){//Check voltage is still low and the first wasn't a miscellaneous reading
                printi("Battery low! Turn the power off.");
                detach_servos();
                while(1){}//loop and do nothing
            }
        }
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void execute_moves_joint(int repeat){
    for(int i = 0; i < repeat; i++){
        for(int row = 0; row < moves_array_index; row++){
            gripper_servo(moves_array[row][3]);
            joint_move(moves_array[row][0], moves_array[row][1], moves_array[row][2], step_pulses, step_delay);
        }
        if(get_battery_level_voltage() < 9.5){//9.5V is used as the cut off to allow for inaccuracies and be on the safe side.
            delay(200);
            if(get_battery_level_voltage() < 9.5){//Check voltage is still low and the first wasn't a miscellaneous reading
                printi("Battery low! Turn the power off.");
                detach_servos();
                while(1){}//loop and do nothing
            }
        }
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
void set_step_increment(float stepIncrement){
    if(stepIncrement > 0 && stepIncrement < 250){//step increments must be positive and arbitrarily less than 250
        step_increment = stepIncrement;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_step_delay(int stepDelay){
    if(stepDelay > 0 && stepDelay < 32,767){//step delay must be positive and 32,767ms
        step_delay = stepDelay;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_step_pulses(int stepPulseIncrement){
    if(stepPulseIncrement > 0 && stepPulseIncrement < 2000){//step pulse increments must be positive and arbitrarily less than 2000
        step_pulses = stepPulseIncrement;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int get_gripper_rotation(void){
    return servo_gripper_rotation;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float get_serial_float(void){ //TODO: The delay may need to be changed depending on the set baud rate
    delay(2); //wait to make sure all data in the serial message has arived
    stringText = "";//clear stringText
    while(Serial.available()){//set elemetns of stringText to the serial values sent
        char digit = Serial.read();
        stringText += digit; //Adds the char to the end of the string
        if(stringText.length() >= MAX_STRING_LENGTH) break;//exit the loop when the stringText array is full
    }
    serialFlush();//Clear any excess data in the serial buffer
    
    return stringText.toFloat();
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int get_serial_int(void){ //TODO: The delay may need to be changed depending on the set baud rate
    delay(2); //wait to make sure all data in the serial message has arived
    stringText = "";//clear stringText
    while(Serial.available()){//set elemetns of stringText to the serial values sent
        char digit = Serial.read();
        stringText += digit; //Adds the char to the end of the string
        if(stringText.length() >= MAX_STRING_LENGTH) break;//exit the loop when the stringText array is full
    }
    serialFlush();//Clear any excess data in the serial buffer
    
    return stringText.toInt();
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

