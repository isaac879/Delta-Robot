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

#include <strsafe.h>
#include <tchar.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <windows.h>
#include <fstream>
#include <cstring>
#include <math.h>

#define PI 3.14159265

#define COMMAND_CARTESIAN 1
#define COMMAND_ABSOLUTE_CARTESIAN 2
#define COMMAND_GRIPPER 3

#define COMMAND_JOG_X 'x'
#define COMMAND_JOG_Y 'y'
#define COMMAND_JOG_Z 'z'
#define COMMAND_GRIPPER_ASCII 'g'
#define COMMAND_STATUS 's' 
#define COMMAND_ADD_POSITION 'p'
#define COMMAND_SET_STEP_DELAY 'd'
#define COMMAND_SET_STEP_INCREMENT 'i'
#define COMMAND_CLEAR_ARRAY 'c'
#define COMMAND_EXECUTE 'e'

//Flags
bool portOpenFlag = false; //The flag used to determin if the serial port for the Arduino Nano is opened

byte getBytesFromInt(short sourceInt, unsigned int byteNumber) {//Gets a byte from an int. byte number should be 0 for the 8 least signigicant bits or 1 for the most fignificant bits
	byte byteValue = (sourceInt >> (8 * byteNumber)) & 0xFF;
	return byteValue;
}

HANDLE hSerial;// Serial handle

void openSerial(char port[]) {
	if (portOpenFlag == false) {//If the port is not already open
		wchar_t wtext[16];
		mbstowcs(wtext, port, strlen(port) + 1);
		LPCWSTR portName = wtext;
		hSerial = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);//Set the Com port to the correct arduino one

		if (hSerial == INVALID_HANDLE_VALUE)
		{
			std::cout << "INVALID_HANDLE_VALUE" << stderr << std::endl;
		}
		else {
			portOpenFlag = true;
		}
	}
}

int closeSerial() {
	if (portOpenFlag == true) {//If the port is open
		if (CloseHandle(hSerial) == 0)
		{
			std::cout << "Error\n" << stderr << std::endl;
			return -1;
		}
		portOpenFlag = false;
	}
}

int serialConnect(char portName[]) {
	openSerial(portName);//Open the serial port

	// Declare variables and structures
	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);//why do i need this?
	if (GetCommState(hSerial, &dcbSerialParams) == 0) {
		closeSerial();
		return -1;
	}
	else {
		dcbSerialParams.BaudRate = CBR_57600; //Set device parameters (57600 baud, 1 start bit, 1 stop bit, no parity)
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;
		dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE; //Stops the Arduino resetting after connecting
	}

	if (SetCommState(hSerial, &dcbSerialParams) == 0)
	{
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

	if (SetCommTimeouts(hSerial, &timeouts) == 0) {
		closeSerial();
		return 1;
	}
	return 0;
}

int serialWrite(char data[], int lengthOfArray) {
	DWORD bytes_written;
	if (!WriteFile(hSerial, data, lengthOfArray, &bytes_written, NULL)) {
		closeSerial();
		return -1;
	}
	return 0;
}

int sendCarteasian(int command, double x, double y, double z) {
	short xShort = round(x);
	short yShort = round(y);
	short zShort = round(z);
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

void sendGripper(short rotation){
	char data[2]; //Data array to send

	data[0] = COMMAND_GRIPPER;
		
	data[1] = getBytesFromInt(rotation, 0);//Gets the 8 LSBs

	serialWrite(data, sizeof(data));
}

void sendCommand(char command){
	char data[] = {command}; //Data array to send
	serialWrite(data, sizeof(data));
}

void sendCharArray(char *array) {
	serialWrite(array, strlen(array));

}

void circle_xy(int radius, int inc) {
	for (int i = 0; i < 360; i += inc) {
		int x = radius * sin(i * PI /180);
		int y = radius * cos(i * PI / 180);
		int z = 230; 
		sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, x, y, z);
		Sleep(10);
	}
}

void jab(int hight, int delay) {
	sendCarteasian(COMMAND_CARTESIAN, 0, 0, hight);
	Sleep(delay);
	sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, 0, 0, 110);
}

int main(void){
	BOOL click_flag = FALSE;
	BOOL z_flag = FALSE;
	BOOL g_flag = FALSE;
	BOOL g_click_flag = FALSE;
	int x = 0;
	int y = 0;
	int z = 280;
	POINT p;
	POINT o;

	std::string line;
	char strFile[15];//buffer for the com port name
	std::ifstream readfile("C:\\Users\\isaac879\\Documents\\Projects\\Delta_robot\\serial_port.txt");//Location of the text file that contains the com port to connect to. To connect via COM port 3 contents of the file should be: \\.\COM3

	if (readfile.is_open())
	{
		if (getline(readfile, line)) {
			std::strcpy(strFile, line.c_str());
		}
		readfile.close();
	}
	else {
		std::cout << "Error: Unable to open serial_port.txt" << std::endl;
	}

	serialConnect(strFile);//Connects to the com port that the Arduino is connected to.
	//serialConnect("\\\\.\\com3");//Connects to com3
	while(1) {
		if (GetKeyState(VK_ESCAPE) & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			break;
		}
		if (GetKeyState('P') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCommand(COMMAND_ADD_POSITION);
			std::cout << "p" << std::endl;
			while(GetKeyState('P') & 0x8000){}
		}
		if (GetKeyState('C') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCommand(COMMAND_CLEAR_ARRAY);
		}
		if (GetKeyState('E') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("e1");
			while (GetKeyState('E') & 0x8000) {}
		}
		if (GetKeyState('J') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("j1");
			while (GetKeyState('J') & 0x8000) {}
		}
		if (GetKeyState('W') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("y-1");
		}
		if (GetKeyState('A') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("x-1");
		}
		if (GetKeyState('S') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("y1");
		}
		if (GetKeyState('D') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("x1");
		}
		if (GetKeyState('Q') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("z1");
		}
		if (GetKeyState('Z') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("z-1");
		}
		if (GetKeyState('O') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			circle_xy(80, 1);
		}
		if ((GetKeyState(VK_LBUTTON) & 0x100) != 0 && (GetKeyState(VK_RBUTTON) & 0x100) != 0) {
			break;
		}
		if (GetKeyState('B') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("g100");
		}
		if (GetKeyState('N') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("g0");
		}
		if (GetKeyState('M') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCharArray("g-100");
		}
		if (GetKeyState('H') & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			sendCarteasian(COMMAND_ABSOLUTE_CARTESIAN, 0, 0, 110);
		}
		if (GetKeyState(VK_SPACE) & 0x8000) { //MSB is key state (1 if pressed). LSB is the toggle state
			jab(130, 200);
		}
		if (GetCursorPos(&p)) {
			if ((GetKeyState(VK_LBUTTON) & 0x100) != 0 && !click_flag) {
				o.x = p.x;
				o.y = p.y;
				click_flag = TRUE;
				if (p.x < 1080) {//Mouse on the left side of the screen
					z_flag = FALSE;
				}
				else if (p.x >= 1080) {//mouse on the right side of the screen
					z_flag = TRUE;
				}
			}
			if ((GetKeyState(VK_LBUTTON) & 0x100) == 0 && click_flag) {
				click_flag = FALSE;
			}

			if ((GetKeyState(VK_RBUTTON) & 0x100) != 0 && !g_click_flag) {//Toggles the gripper of the delta robot when right mouse button is clicked
				g_click_flag = TRUE;
				if ((GetKeyState(VK_RBUTTON) & 0x100) != 0 && g_flag) {
					sendGripper(15);
					g_flag = !g_flag;
				}
				else if ((GetKeyState(VK_RBUTTON) & 0x100) != 0 && !g_flag) {
					sendGripper(0);
					g_flag = !g_flag;
				}
			}

			if ((GetKeyState(VK_RBUTTON) & 0x100) == 0 && g_click_flag) {
				g_click_flag = FALSE;
			}

			if (click_flag) {
				if (p.x < 1080 && !z_flag) {
					x = (p.x - o.x) / 3.6;//scaled change in mouse position (3.6 is arbitrary)
					y = (p.y - o.y) / 3.6;
				}
				else if (p.x >= 1080 && z_flag) {//When the mouse is on the right side of the screen
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
