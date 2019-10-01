#ifndef DELTA_ROBOT_CONTROLLER_HPP_
#define DELTA_ROBOT_CONTROLLER_HPP_ 

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <windows.h>
#include <string>

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define PI 3.14159265

//Byte commands
#define COMMAND_CARTESIAN 1
#define COMMAND_ABSOLUTE_CARTESIAN 2
#define COMMAND_GRIPPER 3
#define COMMAND_ABSOLUTE_CARTESIAN_LINEAR 4
#define COMMAND_SET_PROGRAM_ARRAY 5
#define COMMAND_REQUEST_READY_FLAG 6

//String commands
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
#define COMMAND_SET_US_INCREMENT_LINEAR 'u'
#define COMMAND_SET_US_INCREMENT_JOINT 'U'
#define COMMAND_STEP_FORWARD '>'
#define COMMAND_STEP_BACKWARD '<'
#define COMMAND_JUMP_TO_START '['
#define COMMAND_JUMP_TO_END ']'
#define COMMAND_EDIT_ARRAY 'P'
#define COMMAND_ADD_DELAY 'D'
#define COMMAND_MOVE_HOME 'h'
#define COMMAND_PRINT_FILE 'f'
#define COMMAND_PING_PONG 'o'

//EEPROM commands
#define COMMAND_SET_LINK_2 'L'
#define COMMAND_SET_END_EFFECTOR_TYPE 'E'
#define COMMAND_SET_AXIS_DIRECTION 'A'
#define COMMAND_SET_HOME_X 'X'
#define COMMAND_SET_HOME_Y 'Y'
#define COMMAND_SET_HOME_Z 'Z'
#define COMMAND_SET_HOME_GRIPPER 'G'

#define READY_FLAG "##"
#define PATH_PROGRAM_ARRAYS "D:\\Documents\\Projects\\Delta_robot\\Program_arrays.txt"
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

byte getBytesFromInt(short, unsigned int);
void openSerial(LPCSTR);
int closeSerial(void);
int serialConnect(LPCSTR);
int serialWrite(char [], int);
void printIncomingData(void);
int writeToFile(char*, char*);
int saveProgramToFile(char*);
int waitForReadyFlag(unsigned int);
int sendCarteasian(int, double, double, double);
int sendCarteasianLinearWait(int, int, int, int);
void sendGripper(short);
void sendCommand(char);
void sendCharArray(char*);
void circle_xy(int, int);
void jab(int, int);
int execute_program_file(std::string, int);
int move_and_send_program_file(std::string, int);
std::string getPortName(std::string);
int deltaRobotInit(void);
int send_program_file(std::string, int);

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#endif  //DELTA_ROBOT_CONTROLLER_HPP_