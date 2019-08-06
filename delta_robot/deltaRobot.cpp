#include "deltaRobot.h"
#include <Iibrary.h>
#include <math.h>
#include <Servo.h> 
#include <EEPROM.h>

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

Coordinate_f end_effector;
Coordinate_f home_position;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

float servo_1_angle;
float servo_2_angle;
float servo_3_angle;
int gripper_servo_value = 0;

int servo_1_pulse_count = 0;
int servo_2_pulse_count = 0;
int servo_3_pulse_count = 0;
int servo_4_pulse_count = 0;

Program_element program_elements[ARRAY_LENGTH];

int moves_array_elements = 0;
int current_moves_array_index = -1;

int step_delay_linear = 0; //0ms 
float step_increment = 0.4; //0.4mm
int step_pulses = 1; //1us increments
int step_delay_joint = 3; //3ms 

String stringText = "";

byte link_2 = L2;//Default value of link 2
byte end_effector_type = NONE;
byte axis_direction = 0;                             
float servo_offset_z = SERVO_OFFSET_Z;
int gripper_home = 0;
int gripper_servo_min = SERVO_4_MIN;//Default values
int gripper_servo_max = SERVO_4_MAX;

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
    zt -= servo_offset_z;
    
    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(link_2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / link_2);
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
    zt -= servo_offset_z;
    float x = xt;
    float y = yt;
    xt = x * cos(2.0943951023931954923084289221863f) - y * sin(2.0943951023931954923084289221863f); //Rotate coordinate frame 120 degrees
    yt = x * sin(2.0943951023931954923084289221863f) + y * cos(2.0943951023931954923084289221863f);
    
    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(link_2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / link_2);
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
    zt -= servo_offset_z;

    float x = xt;
    float y = yt;
    xt = x * cos(4.1887902047863909846168578443727f) - y * sin(4.1887902047863909846168578443727f); //Rotate coordinate frame 240 degrees
    yt = x * sin(4.1887902047863909846168578443727f) + y * cos(4.1887902047863909846168578443727f);

    float arm_end_x = xt + L3;
    float l2p = sqrt(pow(link_2, 2) - pow(yt, 2));
    
    float l2pAngle = asin(yt / link_2);
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
    if(axis_direction == 1){//if axis are inverted
        xt = -xt;
        zt = -zt;
    }
    
    if(inverse_kinematics_1(xt, yt, zt) && inverse_kinematics_2(xt, yt, zt) && inverse_kinematics_3(xt, yt, zt)){ //Calculates checks the positions are valid.
        if(axis_direction == 1){//if axis are inverted
            end_effector.x = -xt;
            end_effector.z = -zt;
        }
        else{
            end_effector.x = xt;
            end_effector.z = zt;
        }
        end_effector.y = yt;
        
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
        delay(stepDelay);
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
    delay(stepDelay);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void move_servos(void){
    servo1.writeMicroseconds(servo_1_pulse_count);
    servo2.writeMicroseconds(servo_2_pulse_count);
    servo3.writeMicroseconds(servo_3_pulse_count);
    servo4.writeMicroseconds(servo_4_pulse_count);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void gripper_servo(char gripperValue){ //The gripper servo has been modified for continuous rotation. Detaching the servo ensures it will stop when rotation is set to 0
//limit in eeprom for each gripper
    switch(end_effector_type){
        case NONE:{
            servo4.detach();
            digitalWrite(SERVO_4_PIN, LOW);
        }
        break;
        case CONTINUOUS_ROTATION:{
            if(gripperValue != 0){
                if(!servo4.attached()){
                    servo4.attach(SERVO_4_PIN, gripper_servo_min, gripper_servo_max);     
                }  
                gripper_servo_value = gripperValue;
                servo_4_pulse_count = mapNumber(gripperValue, -128, 127, gripper_servo_min, gripper_servo_max);
            }
            else if(gripperValue == 0 && servo4.attached()){
                gripper_servo_value = 0;
                servo4.detach();
                digitalWrite(SERVO_4_PIN, LOW);
            }
        }
        break;
        case CLAW_GRIPPER:{
            if(!servo4.attached()){
                servo4.attach(SERVO_4_PIN, gripper_servo_min, gripper_servo_max);     
            }
            gripper_servo_value = (byte)gripperValue;
            servo_4_pulse_count = mapNumber((byte)gripperValue, 0, 255, gripper_servo_min, gripper_servo_max);
        }
        break;
        case VACUUM_GRIPPER:{
            if(!servo4.attached()){
                servo4.attach(SERVO_4_PIN, gripper_servo_min, gripper_servo_max);     
            }
            gripper_servo_value = (byte)gripperValue;
            servo_4_pulse_count = mapNumber((byte)gripperValue, 0, 255, gripper_servo_min, gripper_servo_max);
        }
        break;
        case ELECTROMAGNET:{
            if(servo4.attached()){
                servo4.detach();
            }
            analogWrite(SERVO_4_PIN, gripperValue);
            gripper_servo_value = gripperValue;
        }
        break;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void position_gripper_servo(char value){
    gripper_servo(value);
    servo4.writeMicroseconds(servo_4_pulse_count);
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
    printi(F("\n---Delta robot status---\n\n"));
    printi(F("X coordinate:\t"), end_effector.x);
    printi(F("Y coordinate:\t"), end_effector.y);
    printi(F("Z coordinate:\t"), end_effector.z);

    printi(F("\nServo 1 angle:\t"), radsToDeg(servo_1_angle));
    printi(F("Servo 2 angle:\t"), radsToDeg(servo_2_angle)); 
    printi(F("Servo 3 angle:\t"), radsToDeg(servo_3_angle));
    printi(F("Servo 4 value:\t"), gripper_servo_value); 

    printi(F("\n\nServo 1 pulses:\t"), servo_1_pulse_count);
    printi(F("Servo 2 pulses:\t"), servo_2_pulse_count);
    printi(F("Servo 3 pulses:\t"), servo_3_pulse_count);
    printi(F("Servo 4 pulses:\t"), servo_4_pulse_count);

    printi(F("\nStep delay:\t"), step_delay_linear);
    printi(F("Step increment:\t"), step_increment);
    printi(F("Step pulse:\t"), step_pulses);
    printi(F("Step delay joint:\t"), step_delay_joint);
    
    printi(F("\nArray elements:\t"), moves_array_elements);
    printi(F("Current array index:\t"), current_moves_array_index);
    print_moves_array();

    print_eeprom();
 
    printi(F("\nBattery:\t"), get_battery_level_voltage(), 1, F("V"));
    printi(F("\t("), get_battery_level_percentage(), 1, F("%)\n"));

    printi(F("\nFirmware Version:\t"));
    printi(FIRMWARE_VERSION);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void report_commands(void){
    printi(F("\n---Delta robot commands---\n\n"));
    printi(F("Cartesian\t0x1+6 bytes\t(Format: 1xxyyzz)\n"));
    printi(F("Abslute cartesian\t0x2+6 bytes\t(Format: 2xxyyzz)\n"));
    printi(F("Gripper\t0x3+1 byte\t(Format: 3b)\n")); 
    printi(F("Jog X-axis\tx+str\t(Format: x123.4)\n"));
    printi(F("Jog Y-axis\ty+str\t(Format: y123.4)\n"));
    printi(F("Jog Z-axis\tz+str\t(Format: z123.4)\n"));
    printi(F("Gripper ascii\tg+str\t(Format: g123)\n")); 
    printi(F("Add position\tp\t(Format: p)\n"));
    printi(F("Clear array\tc\t(Format: c)\n"));
    printi(F("Status\ts\t(Format: s)\n"));
    printi(F("Set delay (ms)\td+string\t(Format: d123)\n")); 
    printi(F("Set step increment (mm)\ti+str\t(Format: i123.4)\n")); 
    printi(F("Set pulse increment (us)\tu+str\t(Format: u123.4)\n")); 
    printi(F("Linear execute x times\te+str\t(Format: e123)\n"));
    printi(F("Joint execute x times\tj+str\t(Format: j123)\n"));   
    printi(F("Set joint delay (ms)\tU+str\t(Format: U123)\n"));  
    printi(F("Step to the next position \t>\t(Format: >)\n"));  
    printi(F("Step to the previous position \t<\t(Format: <)\n"));  
    printi(F("Edit the current array position \tP\t(Format: P)\n"));  
    printi(F("Add a delay to the current array position \tD+str\t(Format: D123\n"));  
    printi(F("EEPROM: Set link 2\tL+str\t(Format: L123.4)\n"));  
    printi(F("EEPROM: Set end effector type\tE+str\t(Format: E123)\n"));  
    printi(F("EEPROM: Set Axis direction\tA+str\t(Format: A123)\n"));  
    printi(F("EEPROM: Set X home position\tX+str\t(Format: X123.4)\n"));  
    printi(F("EEPROM: Set Y home position\tY+str\t(Format: Y123.4)\n"));  
    printi(F("EEPROM: Set Z home position\tZ+str\t(Format: Z123.4)\n"));     
    printi(F("EEPROM: Set gripper home position\tG+str\t(Format: G123)\n"));
    printi(F("EEPROM: Set gripper servo\tM+str+m(min) or M(max)+str\t(Format: M1m123)\n"));           
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void print_moves_array(void){
    printi(F("\n---Move Array---\n"));
    for(int row = 0; row < moves_array_elements; row++){
        printi(F("\n"), row, F(" |"));
        printi(F(" X: "), program_elements[row].x, 3, F("\t"));
        printi(F("Y: "), program_elements[row].y, 3, F("\t"));
        printi(F("Z: "), program_elements[row].z, 3, F("\t"));
        printi(F("G: "), program_elements[row].gripper, F(" \t"));  
        printi(F("D: "), program_elements[row].delay_ms, F(" |\n"));    
    }
    printi(F("\n"));
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void print_eeprom(void){
    printi(F("\n---Saved Values---\n\n"));
    printi(F("Link 2 length: "), EEPROM.read(EEPROM_ADDRESS_LINK_2));
    printi(F("End effector type: "), EEPROM.read(EEPROM_ADDRESS_END_EFFECTOR_TYPE));
    printi(F("Axis Direction Inversion: "), EEPROM.read(EEPROM_ADDRESS_AXIS_DIRECTION));

    int temp;
    EEPROM.get(EEPROM_ADDRESS_GRIPPER_ROTATION_MIN, temp);
    printi(F("Gripper rotation servo min: "), temp);
    EEPROM.get(EEPROM_ADDRESS_GRIPPER_ROTATION_MAX, temp);
    printi(F("Gripper rotation servo max: "), temp);
    EEPROM.get(EEPROM_ADDRESS_GRIPPER_CLAW_MIN, temp);
    printi(F("Gripper claw servo min: "), temp);
    EEPROM.get(EEPROM_ADDRESS_GRIPPER_CLAW_MAX, temp);
    printi(F("Gripper claw servo max: "), temp);
    EEPROM.get(EEPROM_ADDRESS_GRIPPER_VACUUM_MIN, temp);
    printi(F("Gripper vacuum servo min: "), temp);
    EEPROM.get(EEPROM_ADDRESS_GRIPPER_VACUUM_MAX, temp);
    printi(F("Gripper vacuum servo max: "), temp);

    printi(F("X Home: "), home_position.x);
    printi(F("Y Home: "), home_position.y);
    printi(F("Z Home: "), home_position.z);
    printi(F("Gripper Home: "), gripper_home);
    printi(F("Z offset: "), servo_offset_z);
    printi(F("---------------------\n"));
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void serialFlush(void){
    while(Serial.available() > 0) {
        char c = Serial.read();
    }
} 

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void detach_servos(void){//Stops the signals to the servos so they will not draw power to hold their positions.
    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
    digitalWrite(SERVO_1_PIN, LOW);
    digitalWrite(SERVO_2_PIN, LOW);
    digitalWrite(SERVO_3_PIN, LOW);
    digitalWrite(SERVO_4_PIN, LOW);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int add_position(void){
    if(moves_array_elements >= 0 && moves_array_elements < ARRAY_LENGTH){
        program_elements[moves_array_elements].x = end_effector.x;
        program_elements[moves_array_elements].y = end_effector.y;
        program_elements[moves_array_elements].z = end_effector.z;
        program_elements[moves_array_elements].gripper = get_gripper_rotation();
        current_moves_array_index = moves_array_elements;
        moves_array_elements++;//increment the index
        return 0;
    }
    return -1;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void clear_array(void){
    for(int row = 0; row < ARRAY_LENGTH; row++){
        program_elements[row].x = 0;
        program_elements[row].y = 0;
        program_elements[row].z = 0;
        program_elements[row].gripper = 0;
        program_elements[row].delay_ms = 0;
    }
    moves_array_elements = 0;
    current_moves_array_index = -1;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void execute_moves(int repeat){
    for(int i = 0; i < repeat; i++){
        for(int row = 0; row < moves_array_elements; row++){
            linear_move(program_elements[row].x, program_elements[row].y, program_elements[row].z, step_increment, step_delay_linear);
            position_gripper_servo(program_elements[row].gripper);
            delay(program_elements[row].delay_ms);
        }
        if(get_battery_level_voltage() < 9.5){//9.5V is used as the cut off to allow for inaccuracies and be on the safe side.
            delay(200);
            if(get_battery_level_voltage() < 9.5){//Check voltage is still low and the first wasn't a miscellaneous reading
                printi(F("Battery low! Turn the power off."));
                detach_servos();
                while(1){}//loop and do nothing
            }
        }
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void execute_moves_joint(int repeat){
    for(int i = 0; i < repeat; i++){
        for(int row = 0; row < moves_array_elements; row++){            
            joint_move(program_elements[row].x, program_elements[row].y, program_elements[row].z, step_pulses, step_delay_joint);
            position_gripper_servo(program_elements[row].gripper);
            delay(program_elements[row].delay_ms);
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

void goto_moves_array_index(int index){
    if(index < moves_array_elements && index >= 0){
        linear_move(program_elements[index].x, program_elements[index].y, program_elements[index].z, step_increment, step_delay_linear);
        position_gripper_servo(program_elements[index].gripper);
        current_moves_array_index = index;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void goto_moves_array_start(void){
    linear_move(program_elements[0].x, program_elements[0].y, program_elements[0].z, step_increment, step_delay_linear);
    position_gripper_servo(program_elements[0].gripper);
    current_moves_array_index = 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void goto_moves_array_end(void){
    if(0 < moves_array_elements){
        linear_move(program_elements[moves_array_elements - 1].x, program_elements[moves_array_elements - 1].y, program_elements[moves_array_elements - 1].z, step_increment, step_delay_linear);
        position_gripper_servo(program_elements[moves_array_elements - 1].gripper);
        current_moves_array_index = moves_array_elements - 1;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void edit_moves_array_index(int index){
    if(index >= 0 && index < moves_array_elements){
        program_elements[index].x = end_effector.x;
        program_elements[index].y = end_effector.y;
        program_elements[index].z = end_effector.z;
        program_elements[index].gripper = get_gripper_rotation();
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void add_delay(int index, int delayms){
    if(index < moves_array_elements && index >= 0 && delayms > 0 && delayms <= 32767){
        program_elements[index].delay_ms = delayms;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int get_moves_array_index(void){
    return current_moves_array_index;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void increment_moves_array_index(void){
    if(current_moves_array_index < moves_array_elements - 1){
        current_moves_array_index++;
    }
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void decrement_moves_array_index(void){
    if(current_moves_array_index > 0){
        current_moves_array_index--;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_step_increment(float stepIncrement){
    if(stepIncrement > 0 && stepIncrement < 250){//step increments must be positive and arbitrarily less than 250
        step_increment = stepIncrement;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_step_delay_linear(int stepDelay){
    if(stepDelay > 0 && stepDelay < 32,767){//step delay must be positive and 32,767ms
        step_delay_linear = stepDelay;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_step_delay_joint(int stepDelay){
    if(stepDelay > 0 && stepDelay <= 32767){//step delay must be positive and 32,767ms
        step_delay_joint = stepDelay;
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
    return gripper_servo_value;
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

void set_link_2_length(int len){
    EEPROM.update(EEPROM_ADDRESS_LINK_2, len);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_end_effector_type(int type){
    if(type < 0 || type > 4) return;
    EEPROM.update(EEPROM_ADDRESS_END_EFFECTOR_TYPE, type);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_axis_direction(int dir){
    if(dir == 0){
        set_z_offset(SERVO_OFFSET_Z);
        EEPROM.update(EEPROM_ADDRESS_AXIS_DIRECTION, dir);
    }
    else if(dir == 1){
        set_z_offset(SERVO_OFFSET_Z_INVERTED);
        EEPROM.update(EEPROM_ADDRESS_AXIS_DIRECTION, dir);
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_z_offset(float offset){
    EEPROM.put(EEPROM_ADDRESS_Z_OFFSET, offset);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_home_x(float x){
    EEPROM.put(EEPROM_ADDRESS_HOME_X, x);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_home_y(float y){
    EEPROM.put(EEPROM_ADDRESS_HOME_Y, y);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_home_z(float z){
    EEPROM.put(EEPROM_ADDRESS_HOME_Z, z);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_home_gripper(int val){
    EEPROM.put(EEPROM_ADDRESS_HOME_GRIPPER, val);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_gripper_min_max(char gripperType, char min_max, int gripperValue){
    switch(gripperType){
        case CONTINUOUS_ROTATION:{
            if(min_max == MIN){
                EEPROM.put(EEPROM_ADDRESS_GRIPPER_ROTATION_MIN, gripperValue);
            }
            else if(min_max == MAX){
                EEPROM.put(EEPROM_ADDRESS_GRIPPER_ROTATION_MAX, gripperValue);
            }
        }
        break;
        case CLAW_GRIPPER:{
            if(min_max == MIN){
                EEPROM.put(EEPROM_ADDRESS_GRIPPER_CLAW_MIN, gripperValue);
            }
            else if(min_max == MAX){
                EEPROM.put(EEPROM_ADDRESS_GRIPPER_CLAW_MAX, gripperValue);
            }
        }
        break;
        case VACUUM_GRIPPER:{
           if(min_max == MIN){
                EEPROM.put(EEPROM_ADDRESS_GRIPPER_VACUUM_MIN, gripperValue);
            }
            else if(min_max == MAX){
                EEPROM.put(EEPROM_ADDRESS_GRIPPER_VACUUM_MAX, gripperValue);
            }
        }
        break;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void set_eeprom_values(void){
    link_2 = EEPROM.read(EEPROM_ADDRESS_LINK_2);
    end_effector_type = EEPROM.read(EEPROM_ADDRESS_END_EFFECTOR_TYPE);
    axis_direction = EEPROM.read(EEPROM_ADDRESS_AXIS_DIRECTION);
    
    switch(end_effector_type){
        case CONTINUOUS_ROTATION:{
            EEPROM.get(EEPROM_ADDRESS_GRIPPER_ROTATION_MIN, gripper_servo_min);
            EEPROM.get(EEPROM_ADDRESS_GRIPPER_ROTATION_MAX, gripper_servo_max);
        }
        break;
        case CLAW_GRIPPER:{
            EEPROM.get(EEPROM_ADDRESS_GRIPPER_CLAW_MIN, gripper_servo_min);
            EEPROM.get(EEPROM_ADDRESS_GRIPPER_CLAW_MAX, gripper_servo_max);
        }
        break;
        case VACUUM_GRIPPER:{
            EEPROM.get(EEPROM_ADDRESS_GRIPPER_VACUUM_MIN, gripper_servo_min);
            EEPROM.get(EEPROM_ADDRESS_GRIPPER_VACUUM_MAX, gripper_servo_max);
        }
        break;
    }    
    
    EEPROM.get(EEPROM_ADDRESS_HOME_X, home_position.x);
    EEPROM.get(EEPROM_ADDRESS_HOME_Y, home_position.y);
    EEPROM.get(EEPROM_ADDRESS_HOME_Z, home_position.z);
    EEPROM.get(EEPROM_ADDRESS_HOME_GRIPPER, gripper_home);
    EEPROM.get(EEPROM_ADDRESS_Z_OFFSET, servo_offset_z);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void move_home(void){
    linear_move(home_position.x, home_position.y, home_position.z, step_increment, step_delay_linear);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
