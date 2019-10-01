/*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE
*
* Code written by isaac879
*
* IMPORTANT: This code was written quickly an botched together so I could test my delta robot. As such is may be buggy, poorly documented, follow bad practices,
* not work at all or explode... That being said: "It worked on my machine."
*/

/*
* Features to add:
* Read program array from a text file
* [path]\\moves_array_[number].txt
* parse and send vales then 'p' //this would require some medium delays to calculate ikine.
*/

//save array to robot for interpolation
//send movement, delay for 15ms, send p, delay (15), send d delay(15)

#include "delta_robot_controller.hpp"

#include <strsafe.h>
#include <tchar.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <cstring>
#include <math.h>
#include <chrono>

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Flags
bool portOpenFlag = false; //The flag used to determin if the serial port for the Arduino Nano is opened

HANDLE hSerial; //Serial handle

byte getBytesFromInt(short sourceInt, unsigned int byteNumber){//Gets a byte from an int. byte number should be 0 for the 8 least signigicant bits or 1 for the most fignificant bits
	byte byteValue = (sourceInt >> (8 * byteNumber)) & 0xFF;
	return byteValue;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void openSerial(LPCSTR port){
	if (portOpenFlag == false){//If the port is not already open
		hSerial = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);//Set the Com port to the correct arduino one

		if (hSerial == INVALID_HANDLE_VALUE){
			std::cout << "Error: invalid com port handle. Attempted to open: " << port << std::endl;
		}
		else{
			portOpenFlag = true;
			std::cout << "Successfully opened: " << port << std::endl;
		}
	}
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int closeSerial(){
	if (portOpenFlag == true){//If the port is open
		if (CloseHandle(hSerial) == 0){
			std::cout << "Error\n" << stderr << std::endl;
			return -1;
		}
		portOpenFlag = false;
		printf("Serial port closed...\n");
		return 1;
	}
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int serialConnect(LPCSTR portName){
	std::cout << "Attempting to open serial port " << portName << std::endl;
	openSerial(portName);//Open the serial port

	// Declare variables and structures
	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);//why do i need this?
	if (GetCommState(hSerial, &dcbSerialParams) == 0){
		closeSerial();
		return -1;
	}
	else{
		dcbSerialParams.BaudRate = CBR_57600; //Set device parameters (57600 baud, 1 start bit, 1 stop bit, no parity)
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;
		dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE; //Stops the Arduino resetting after connecting
	}

	if(SetCommState(hSerial, &dcbSerialParams) == 0){
		closeSerial();
		return -1;
	}

	// Set COM port timeout settings
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if(SetCommTimeouts(hSerial, &timeouts) == 0){
		closeSerial();
		return 1;
	}
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int serialWrite(char data[], int lengthOfArray){
	DWORD bytes_written;
	if (!WriteFile(hSerial, data, lengthOfArray, &bytes_written, NULL)) {
		closeSerial();
		return -1;
	}
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void printIncomingData(void){
	COMSTAT status;
	DWORD errors;
	DWORD bytesRead;//Number of bytes that have been read

	while(1){
		ClearCommError(hSerial, &errors, &status); //Use the ClearCommError function to get status info on the Serial port	
		if(status.cbInQue == 0) break;

		char* dataBuffer = new char[status.cbInQue + 1]; //buffer the size of the data available to be read
		dataBuffer[status.cbInQue] = '\0'; //adds a null terminator to the end of the buffer

		if(ReadFile(hSerial, dataBuffer, status.cbInQue, &bytesRead, NULL)){ //Try to read the require number of chars, and return the number of read bytes on success
			std::cout << dataBuffer;// << std::endl;
		}
		delete[] dataBuffer;
		Sleep(5); //Wait for 2ms before checking if more data has come in
	} 
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int writeToFile(char* fileName, char* text){
	std::ofstream file(fileName, std::ios::app);
	if (file.is_open()){
		file << text;
		file.close();
		return 1;
	}
	else {
		std::cout << "Error: Unable to open " << fileName << std::endl;
		return -1;
	}
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int saveProgramToFile(char* path){
	sendCommand(COMMAND_REQUEST_READY_FLAG); //request ready flag
	if(waitForReadyFlag(3000) == 1){ //if flag is received
		int progNum = 0;
		//gets the maximum program number currently in the file
		std::ifstream programFile(path);
		if(programFile.is_open()){
			std::string line;
			while(std::getline(programFile, line)){ //Find correct program
				std::size_t found = line.find("program: ");
				if(found != std::string::npos){ //if "program: " was found increment the new programs number
					progNum++;
					std::cout << "prognum: " << progNum << std::endl;
				}		
			}
			programFile.close();
		}

		sendCommand(COMMAND_PRINT_FILE); //request program array to be sent
		
		std::string programName = "program: " + std::to_string(progNum) + "\n";

		writeToFile(path, (char*)programName.c_str());
		
		COMSTAT status;
		DWORD errors;
		DWORD bytesRead;//Number of bytes that have been read

		Sleep(100);

		while(1){
			ClearCommError(hSerial, &errors, &status); //Use the ClearCommError function to get status info on the Serial port	
			if(status.cbInQue == 0) break;

			char* dataBuffer = new char[status.cbInQue + 1]; //buffer the size of the data available to be read
			dataBuffer[status.cbInQue] = '\0'; //adds a null terminator to the end of the buffer

			if(ReadFile(hSerial, dataBuffer, status.cbInQue, &bytesRead, NULL)){ //Try to read the require number of chars, and return the number of read bytes on success
				//std::cout << dataBuffer;// << std::endl;
				//print to the file
				writeToFile(path, dataBuffer);
			}
			delete[] dataBuffer;
			Sleep(10); //Wait for 5ms before checking if more data has come in
		} 

		//prints the program arrays text file
		std::cout << "\n-----Program_arrays.txt-----\n" << std::endl;
		std::ifstream programFile2(path);
		if(programFile2.is_open()){
			std::string line;
			while(std::getline(programFile2, line)){
				std::cout << line << std::endl;
			}
			programFile2.close();
		}	
		std::cout << "-----End of Program_arrays.txt-----\n" << std::endl;
	}
	else{
		std::cout << "Waiting for the ready flag timed out." << std::endl;
		return -1;
	}
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/


int waitForReadyFlag(unsigned int timeOutMs){
	auto start = std::chrono::steady_clock::now();

	COMSTAT status;
	DWORD errors;
	DWORD bytesRead;//Number of bytes that have been read
	int statusFlag = -1;

	while(((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count()) < timeOutMs) && statusFlag != 1){
		while(statusFlag != 1){
			ClearCommError(hSerial, &errors, &status); //Use the ClearCommError function to get status info on the Serial port

			if(status.cbInQue == 0){
				break;
			}
			else if(status.cbInQue >= 2){ //wait for enough characters to be read so they could contain the flag
				char* dataBuffer = new char[status.cbInQue + 1]; //buffer the size of the data available to be read
				dataBuffer[status.cbInQue] = '\0'; //adds a null terminator to the end of the buffer

				if(ReadFile(hSerial, dataBuffer, status.cbInQue, &bytesRead, NULL)){ //Try to read the require number of chars, and return the number of read bytes on success
					//std::cout << "\n" << dataBuffer << "\n" << std::endl;
				
					std::string dataBufferStr(dataBuffer);
				
					if(dataBufferStr.find(READY_FLAG) != std::string::npos){//if the read data contains the ready flag
						statusFlag = 1;
						std::cout << "ReadyFlag Received." << std::endl;
					}
					else{
						statusFlag = 0;
					}
				}
				delete[] dataBuffer;
			}
			Sleep(5); //Wait for 5ms before checking if more data has come in
		} 
	}
	return statusFlag;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int sendProgramFileLine(int x, int y, int z, char griper, int delay_ms){
	short xShort = x;
	short yShort = y;
	short zShort = z;
	short delayShort = delay_ms;

	char data[9]; //Data array to send

	data[0] = getBytesFromInt(xShort, 1);//Gets the 8 MSBs
	data[1] = getBytesFromInt(xShort, 0);//Gets the 8 LSBs
	data[2] = getBytesFromInt(yShort, 1);//Gets the 8 MSBs
	data[3] = getBytesFromInt(yShort, 0);//Gets the 8 LSBs
	data[4] = getBytesFromInt(zShort, 1);//Gets the 8 MSBs
	data[5] = getBytesFromInt(zShort, 0);//Gets the 8 LSBs
	data[6] = griper;
	data[7] = getBytesFromInt(delayShort, 1);//Gets the 8 MSBs
	data[8] = getBytesFromInt(delayShort, 0);//Gets the 8 LSBs

	serialWrite(data, sizeof(data));

	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int send_program_file(std::string path, int programNum){ //Sleep() may need the delays increased if the valuses are not being sent/stored correctly.
	std::ifstream programFile(path);

	if(programFile.is_open()){
		std::string line;
		while(std::getline(programFile, line)){ //Find correct program
			std::size_t found = line.find("program: " + std::to_string(programNum));
			if(found != std::string::npos) break;
		}
	
		sendCommand(COMMAND_SET_PROGRAM_ARRAY);
		while(std::getline(programFile, line)){
			if(line == "") break;
			std::size_t end;
			end = line.find(",");
			int x = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int y = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int z = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int g = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			int d = std::stoi(line.substr(0, 5));

			std::cout << "int x = " << x << "\tint y = " << y <<"\tint z = " << z <<"\tint g = " << g <<"\tint d = " << d <<std::endl;

			if(waitForReadyFlag(3000) == 1){
				sendProgramFileLine(x, y, z, g, d);
			}
			else{
				std::cout << "ReadyFlag timeout: The program array may not have been set correctly..." << std::endl;
				break;
			}	
		}
	}
	else{
		std::cout << "Error: Unable to open test_arrays.txt" << std::endl;
		return -1;
	}
	programFile.close();
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int sendCarteasian(int command, double x, double y, double z){
	short xShort = (short)round(x);
	short yShort = (short)round(y);
	short zShort = (short)round(z);
	char data[7]; //Data array to send

	data[0] = command;
	data[1] = getBytesFromInt(xShort, 1);//Gets the 8 MSBs
	data[2] = getBytesFromInt(xShort, 0);//Gets the 8 LSBs
	data[3] = getBytesFromInt(yShort, 1);//Gets the 8 MSBs
	data[4] = getBytesFromInt(yShort, 0);//Gets the 8 LSBs
	data[5] = getBytesFromInt(zShort, 1);//Gets the 8 MSBs
	data[6] = getBytesFromInt(zShort, 0);//Gets the 8 LSBs

	serialWrite(data, sizeof(data)); //Send the move carteasian command and the 6 bytes of data

	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int sendCarteasianLinearWait(int x, int y, int z, int timeOutMs){
	sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN_LINEAR, x, y, z);	
	return waitForReadyFlag(timeOutMs);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void sendGripper(short rotation){
	char data[2]; //Data array to send

	data[0] = COMMAND_GRIPPER;
		
	data[1] = getBytesFromInt(rotation, 0);//Gets the 8 LSBs

	serialWrite(data, sizeof(data));
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void sendCommand(char command){
	char data[] = {command}; //Data array to send
	serialWrite(data, sizeof(data));
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void sendCharArray(char *array){
	serialWrite(array, (int)strlen(array));
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void circle_xy(int radius, int inc){
	for (int i = 0; i < 360; i += inc) {
		double x = radius * sin(i * PI /180);
		double y = radius * cos(i * PI / 180);
		double z = 30; 
		sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, x, y, z);
		Sleep(10);
	}
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void jab(int hight, int delay){
	sendCarteasian(COMMAND_CARTESIAN, 0, 0, hight);
	Sleep(delay);
	sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, 0, 0, 110);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int execute_program_file(std::string path, int programNum){
	std::ifstream programFile(path); //"D:\\Documents\\Projects\\Delta_robot\\test_arrays.txt"

	if(programFile.is_open()){
		std::string line;
		while(std::getline(programFile, line)){ //Find correct program
			std::size_t found = line.find("program: " + std::to_string(programNum));
			if(found != std::string::npos) break;
		}
	
		while(std::getline(programFile, line)){
			if(line == "") break;
			std::size_t end;
			end = line.find(",");
			int x = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int y = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int z = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int g = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			int d = std::stoi(line.substr(0, 5));

			std::cout << "int x = " << x << "\tint y = " << y <<"\tint z = " << z <<"\tint g = " << g <<"\tint d = " << d <<std::endl;
			sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, x, y, z);
			Sleep(15);// Gives the arduino time to read the sent data
			sendGripper(g);
			Sleep(d);
		}
	}
	else{
		std::cout << "Error: Unable to open test_arrays.txt" << std::endl;
		return -1;
	}
	programFile.close();
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int move_and_send_program_file(std::string path, int programNum){ //Sleep() may need the delays increased if the valuses are not being sent/stored correctly.
	std::ifstream programFile(path);

	if(programFile.is_open()){
		std::string line;
		while(std::getline(programFile, line)){ //Find correct program
			std::size_t found = line.find("program: " + std::to_string(programNum));
			if(found != std::string::npos) break;
		}

		sendCommand(COMMAND_CLEAR_ARRAY); //clear privious program
		Sleep(200);
	
		while(std::getline(programFile, line)){
			if(line == "") break;
			std::size_t end;
			end = line.find(",");
			int x = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int y = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int z = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			end = line.find(",");
			int g = std::stoi(line.substr(0, end));
			line.erase(0, end + 2);
			int d = std::stoi(line.substr(0, 5));

			std::cout << "int x = " << x << "\tint y = " << y <<"\tint z = " << z <<"\tint g = " << g <<"\tint d = " << d <<std::endl;

			sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, x, y, z);
			Sleep(200); //Gives the arduino time to read the sent data

			sendGripper(g);
			Sleep(200); //Gives the arduino time to read the sent data

			sendCommand(COMMAND_ADD_POSITION);
		}
	}
	else{
		std::cout << "Error: Unable to open test_arrays.txt" << std::endl;
		return -1;
	}
	programFile.close();
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

std::string getPortName(std::string filePath){
	std::string line;
	std::cout << "Reading port name from: " << filePath << std::endl;
	std::ifstream readfile(filePath);

	if(readfile.is_open()){
		if (getline(readfile, line)){
			std::cout << "Port read from the file: " << line << std::endl;
		}
		readfile.close();
	}
	else {
		std::cout << "Error: Unable to open serial_port.txt" << std::endl;
	}
	return line;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int deltaRobotInit(void){
	//Connects to the com port that the Arduino is connected to.
	for(int i = 0; (i < 5) && (serialConnect(getPortName("D:\\Documents\\Projects\\Delta_robot\\serial_port.txt").c_str()) != 0 ); i++){
		Sleep(1000);
		if(i == 4){
			std::cout << "Error: Unable to open serial port after 5 attempts..." << std::endl;
			return -1;
		}
	}
	sendCommand(COMMAND_MOVE_HOME);
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int jmain(void){
	BOOL click_flag = FALSE;
	BOOL z_flag = FALSE;
	BOOL g_flag = FALSE;
	BOOL g_click_flag = FALSE;
	int x = 0;
	int y = 0;
	int z = 280;
	POINT p;
	POINT o;

	serialConnect(getPortName("D:\\Documents\\Projects\\Delta_robot\\serial_port.txt").c_str());//Connects to the com port that the Arduino is connected to.

	while(1) {
		if (GetKeyState(VK_F8) & 0x8000){
			//while (GetKeyState(VK_F8) & 0x8000) {}//block while the keys are held down
			char data[10];
			std::cin.getline(data, 10);
			std::cin.clear();
			serialWrite(data, sizeof(data));
		}
		if (GetKeyState(VK_F2) & 0x8000) {
			//while (GetKeyState(VK_F8) & 0x8000) {}//block while the keys are held down
			char data[10];
			std::cin.getline(data, 10);
			std::cin.clear();
			send_program_file("D:\\Documents\\Projects\\Delta_robot\\test_arrays.txt", atoi(data));
		}
		if (GetKeyState(VK_ESCAPE) & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			break;
		}
		if (GetKeyState('P') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCommand(COMMAND_ADD_POSITION);
			std::cout << "p" << std::endl;
			while(GetKeyState('P') & 0x8000){}
		}
		if (GetKeyState('O') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCommand(COMMAND_EDIT_ARRAY);
			std::cout << "P" << std::endl;
			while (GetKeyState('O') & 0x8000){}
		}
		if (GetKeyState('5') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"D500");
			while (GetKeyState('5') & 0x8000){}
		}
		if (GetKeyState('1') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"D1000");
			//send_program_file("D:\\Documents\\Projects\\Delta_robot\\test_arrays.txt", 0);
			while (GetKeyState('1') & 0x8000){}
		}
		if (GetKeyState('0') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			//execute_program_file("D:\\Documents\\Projects\\Delta_robot\\test_arrays.txt", 0);
			sendCharArray((char*)"D5000");
			while (GetKeyState('0') & 0x8000){}
		}
		if (GetKeyState('C') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCommand(COMMAND_CLEAR_ARRAY);
		}
		if (GetKeyState(VK_DOWN) & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCommand(COMMAND_STEP_FORWARD);
			while (GetKeyState(VK_DOWN) & 0x8000){}
		}
		if (GetKeyState(VK_UP) & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCommand(COMMAND_STEP_BACKWARD);
			while (GetKeyState(VK_UP) & 0x8000){}
		}
		if (GetKeyState('E') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"e1");
			while (GetKeyState('E') & 0x8000){}
		}
		if (GetKeyState('J') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"j1");
			while (GetKeyState('J') & 0x8000){}
		}
		if (GetKeyState('W') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"y-1");
		}
		if (GetKeyState('A') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"x-1");
		}
		if (GetKeyState('S') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"y1");
		}
		if (GetKeyState('D') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"x1");
		}
		if (GetKeyState('Q') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"z1");
		}
		if (GetKeyState('Z') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"z-1");
		}
		if (GetKeyState('9') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			circle_xy(105, 1);
		}
		if (GetKeyState('B') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"g255");
		}
		if (GetKeyState('N') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"g0");
		}
		if (GetKeyState('M') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"g-100");
		}
		if (GetKeyState('H') & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, 0, 0, 240);
		}
		if (GetKeyState(VK_LEFT) & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"[");
		}
		if (GetKeyState(VK_RIGHT) & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray((char*)"]");
		}

		if (GetKeyState(VK_SPACE) & 0x8000){ //MSB is key state (1 if pressed). LSB is the toggle state
			jab(130, 200);
		}
		if (GetCursorPos(&p)) {
			if ((GetKeyState(VK_LBUTTON) & 0x100) != 0 && !click_flag){
				o.x = p.x;
				o.y = p.y;
				click_flag = TRUE;
				if (p.x < 1080) {//Mouse on the left side of the screen
					z_flag = FALSE;
				}
				else if (p.x >= 1080){//mouse on the right side of the screen
					z_flag = TRUE;
				}
			}
			if ((GetKeyState(VK_LBUTTON) & 0x100) == 0 && click_flag){
				click_flag = FALSE;
			}

			if ((GetKeyState(VK_RBUTTON) & 0x100) != 0 && !g_click_flag){//Toggles the gripper of the delta robot when right mouse button is clicked
				g_click_flag = TRUE;
				if ((GetKeyState(VK_RBUTTON) & 0x100) != 0 && g_flag){
					//sendGripper(255); //-120 vacuum gripper on
					//sendGripper(60); //+- 100 for rotation end effectors
					sendGripper(0); //electromagnet off
					g_flag = !g_flag;
				}
				else if ((GetKeyState(VK_RBUTTON) & 0x100) != 0 && !g_flag){
					//sendGripper(-90);//3 finger gripper
					//sendGripper(0); //100 vacuum gripper off
					sendGripper(255); //electromagnet on
					g_flag = !g_flag;
				}
			}

			if ((GetKeyState(VK_RBUTTON) & 0x100) == 0 && g_click_flag){
				g_click_flag = FALSE;
			}

			if (click_flag) {
				if (p.x < 1080 && !z_flag){
					x = int((int(p.x) - int(o.x)) / 3.6);//scaled change in mouse position (3.6 is arbitrary)
					y = int((int(p.y) - int(o.y)) / 3.6);
				}
				else if (p.x >= 1080 && z_flag){//When the mouse is on the right side of the screen
					z = -(p.y - o.y) / 6;//scaled change in mouse position (6 is arbitrary)
				}
				o.x = p.x;
				o.y = p.y;
				std::cout << "x: " << x << "\ty: " << y << "\tz: " << z << std::endl;
				sendCarteasian(COMMAND_CARTESIAN, x, y, z);//Sends scaled change in mouse pointer position.
				x = 0;
				y = 0;
				z = 0; 
			}
			Sleep(15);//Give the Arduino time to process the commands
		}
	}
	closeSerial();
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/