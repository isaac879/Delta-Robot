#ifndef DELTAROBOT_H
#define DELTAROBOT_H

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Link lengths (mm)
#define L1 100
#define L2 150
#define L3 48

#define END_EFFECTOR_Z_OFFSET 0
#define SERVO_OFFSET_X 75
#define SERVO_OFFSET_Y 0
#define SERVO_OFFSET_Z (41.231 + END_EFFECTOR_Z_OFFSET)

#define SERVO_ANGLE_MIN 0.78539816339744830961566084581988f //45 degrees
#define SERVO_ANGLE_MAX 3.9269908169872415480783042290994f //225 degrees

//Pins the servos are connected to
#define SERVO_1_PIN 9
#define SERVO_2_PIN 10
#define SERVO_3_PIN 11
#define SERVO_4_PIN 6

//Analog input to read the battery voltage from
#define BATTERY_PIN A0

//Servo microsecond pulse limits
#define SERVO_1_MIN 540//TODO: pulse limits will need to be calibrated for your specific servos
#define SERVO_1_MAX 2440
#define SERVO_2_MIN 560
#define SERVO_2_MAX 2480
#define SERVO_3_MIN 460
#define SERVO_3_MAX 2350
#define SERVO_4_MIN 540
#define SERVO_4_MAX 2400

#define HOME_POSITION 0, 0, 230 //x, y, z

#define MAX_STRING_LENGTH 8 //Maxium number of chars that can be received with a command

#define COMMAND_CARTESIAN 1
#define COMMAND_ABSOLUTE_CARTESIAN 2
#define COMMAND_GRIPPER 3

#define COMMAND_JOG_X 'x'
#define COMMAND_JOG_Y 'y'
#define COMMAND_JOG_Z 'z'
#define COMMAND_GRIPPER_ASCII 'g'
#define COMMAND_STATUS 's' 
#define COMMAND_REPORT_COMMANDS 'r'
#define COMMAND_ADD_POSITION 'p'
#define COMMAND_SET_STEP_DELAY 'd'
#define COMMAND_SET_STEP_INCREMENT 'i'
#define COMMAND_CLEAR_ARRAY 'c'
#define COMMAND_EXECUTE 'e'
#define COMMAND_EXECUTE_JOINT 'j'
#define COMMAND_SET_US_INCREMENT 'u'

#define ARRAY_LENGTH 50 //Maximum number of positions to store

#define FIRMWARE_VERSION "1.3.1"

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float boundFloat(float, float, float);
void attach_servos(void);
bool inverse_kinematics_1(float, float, float);
bool inverse_kinematics_2(float, float, float);
bool inverse_kinematics_3(float, float, float);
bool inverse_kinematics(float, float, float);
void linear_move(float, float, float, float, int);
void joint_move(float, float, float, int, int);
void move_servos(void);
void gripper_servo(char);
float get_battery_level_percentage(void);
float get_battery_level_voltage(void);
void report_status(void);
void serialFlush(void);
void gripper_open_close(bool);
void report_commands(void);
void detach_servos(void);
int add_position(void);
void clear_array(void);
void execute_moves(int);
void set_step_increment(float);
void set_step_delay(int);
int get_gripper_rotation(void);
float get_serial_float(void);
int get_serial_int(void);
void execute_moves_joint(int);
void set_step_pulses(int);

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#endif
