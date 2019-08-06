/*--------------------------------------------------------------------------------------------------------------------------------------------------------
 * 
 * CHECK THE CODE FOR "TODO:" AND EDIT APPROPRIATELY 
 * 
 * The code is developed for a Delta robot. The robot is controlled by an Arduino Nano.
 * 
 * Delta Robot STL files: https://www.thingiverse.com/thing:3465651
 * 
 * Project video: https://www.youtube.com/watch?v=vONuJPu1z3s
 * 
 * All measurements are in SI units unless otherwise specified.
 * 
 * The arm's coordinate frame: Servo1 is alligned with the X-axis. Z-axis is vertically up.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE
 * 
 * Code written by isaac879
 *
 *--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//TODO:
//IMPORTANT: When the robot is first turned on after having the code uploaded you may need to set all the EEPROM values be for it will work, then switch it off and on again.

#include "deltaRobot.h"
#include <Iibrary.h> //TODO: Add my custom library or remove dependant functions. Available at: https://github.com/isaac879/Iibrary

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

extern Coordinate_f end_effector;//Stores the end effector coordinates (declared in deltaRobot.cpp)
extern Coordinate_f home_position;
extern int gripper_home;

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){
    Serial.begin(57600); //TODO: Baud rate for the HC-05 module is 9600 by default so needs to be set to 57600 via AT commands to work.
    set_eeprom_values();
    attach_servos();
    gripper_servo(gripper_home);
    if(inverse_kinematics(home_position.x, home_position.y, home_position.z)){
        move_servos();
    }
    clear_array();//initialise the moves array with 0s
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop(){    
    while(Serial.available() < 1){ //Wait for serial data to be available
        if(get_battery_level_voltage() < 9.5){//9.5V is used as the cut off to allow for inaccuracies and be on the safe side.
            delay(200);
            if(get_battery_level_voltage() < 9.5){//Check voltage is still low and the first wasn't a miscellaneous reading
                printi(F("Battery low! Turn the power off."));
                detach_servos();
                while(1){}//loop and do nothing
            }
        }
    } 
    
    byte instruction = Serial.read(); 
    int count = 0;
    
    switch(instruction){
        case COMMAND_CARTESIAN:{//Moves relitive to the current position
            while(Serial.available() != 6){//Wait for six bytes to be available. Breaks after ~200ms if bytes are not received.
                delayMicroseconds(200); 
                count++;
                if(count > 1000){
                    serialFlush();//Clear the serial buffer
                    break;   
                }
            }
            int xTarget = (Serial.read() << 8) + Serial.read();  
            int yTarget = (Serial.read() << 8) + Serial.read(); 
            int zTarget = (Serial.read() << 8) + Serial.read(); 
         
            inverse_kinematics(end_effector.x + xTarget, end_effector.y + yTarget, end_effector.z + zTarget);//Calculates servo positions 
            move_servos();//Moves the robot's servos
        }
        break;
        case COMMAND_ABSOLUTE_CARTESIAN:{//Moves to the specified position
            while(Serial.available() != 6){//Wait for six bytes to be available. Breaks after ~200ms if bytes are not received.
                delayMicroseconds(200); 
                count++;
                if(count > 1000){
                    serialFlush();//Clear the serial buffer
                    break;   
                }
            }
            int xTarget = (Serial.read() << 8) + Serial.read();  
            int yTarget = (Serial.read() << 8) + Serial.read(); 
            int zTarget = (Serial.read() << 8) + Serial.read(); 
         
            inverse_kinematics(xTarget, yTarget, zTarget);//Calculates servo positions 
            move_servos();//Moves the robot's servos
        }
        break;
        case COMMAND_STATUS:{
            report_status();
        }
        break;
        case COMMAND_REPORT_COMMANDS:{
            report_commands();
        }
        break;
        case COMMAND_GRIPPER:{
            while(Serial.available() < 1){
                delay(1);
                count++;
                if(count > 1000){ //Breaks after ~1 second if bytes are not received.
                    serialFlush(); //Clear the serial buffer
                    break;   
                }
            }
            char rotation = Serial.read();
            gripper_servo(rotation);
            move_servos();
        }break;
        case COMMAND_JOG_X:{
            inverse_kinematics(end_effector.x + get_serial_float(), end_effector.y, end_effector.z);//Calculates servo positions 
            move_servos();//Moves the robot's servos
        }
        break;
        case COMMAND_JOG_Y:{
            inverse_kinematics(end_effector.x, end_effector.y + get_serial_float(), end_effector.z);//Calculates servo positions 
            move_servos();//Moves the robot's servos
        }
        break;
        case COMMAND_JOG_Z:{
            inverse_kinematics(end_effector.x, end_effector.y, end_effector.z + get_serial_float());//Calculates servo positions 
            move_servos();//Moves the robot's servos
        }
        break;
        case COMMAND_GRIPPER_ASCII:{
            gripper_servo(get_serial_int());
            move_servos();
        }
        break;
        case COMMAND_ADD_POSITION:{
            add_position();
        }
        break;
        case COMMAND_CLEAR_ARRAY:{
            clear_array();
        }
        break;
        case COMMAND_SET_STEP_DELAY:{           
             set_step_delay_linear(get_serial_int());
        }
        break;
        case COMMAND_SET_STEP_INCREMENT:{
            set_step_increment(get_serial_float());
        }
        break;
        case COMMAND_EXECUTE:{
            execute_moves(get_serial_int());
        }
        break;
        case COMMAND_EXECUTE_JOINT:{
            execute_moves_joint(get_serial_int());
        }
        break;
        case COMMAND_SET_US_INCREMENT_LINEAR:{
            set_step_pulses(get_serial_int());
        }
        break;
        case COMMAND_SET_US_INCREMENT_JOINT:{
            set_step_delay_joint(get_serial_int());
        }
        break;
        case COMMAND_SET_LINK_2:{
            set_link_2_length(get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_SET_END_EFFECTOR_TYPE:{
            set_end_effector_type(get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_SET_AXIS_DIRECTION:{
            set_axis_direction(get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_SET_HOME_X:{
            set_home_x(get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_SET_HOME_Y:{
            set_home_y(get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_SET_HOME_Z:{
            set_home_z(get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_SET_HOME_GRIPPER:{
            set_home_gripper(get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_SET_GRIPPER_MIN_MAX:{
            delay(2);
            char gripperType = (char)Serial.read() - '0';
            char min_max = Serial.read();
            set_gripper_min_max(gripperType, min_max, get_serial_int());
            set_eeprom_values();
        }
        break;
        case COMMAND_STEP_FORWARD:{
            increment_moves_array_index();
            goto_moves_array_index(get_moves_array_index());
        }
        break;
        case COMMAND_STEP_BACKWARD:{
            decrement_moves_array_index();
            goto_moves_array_index(get_moves_array_index());
        }
        break;
        case COMMAND_JUMP_TO_START:{
            goto_moves_array_start();
        }
        break;
        case COMMAND_JUMP_TO_END:{
            goto_moves_array_end();
        }
        break;
        case COMMAND_ADD_DELAY:{
            add_delay(get_moves_array_index(), get_serial_int());
        }
        break;
        case COMMAND_EDIT_ARRAY:{
            edit_moves_array_index(get_moves_array_index());
        }
        break;
        case COMMAND_MOVE_HOME:{
            move_home();
        }
        break;
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
