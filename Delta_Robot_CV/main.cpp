/*------------------------------------------------------------------------------------------------------------------------------------------------------/
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE
*
* Code written by isaac879
*
* IMPORTANT: This code was written quickly an botched together so I could test my delta robot. As such is may be buggy, poorly documented, follow bad practices,
* not work at all or explode... That being said: "It worked on my machine."
*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include "delta_robot_controller.hpp"
#include "computer_vision.hpp"

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

//calib better
//pick function
//place function

//only search for one target spec until all picked then next? (quicker processing )

//nice drawings to show what is going on //target crosshairs //label each target

//if waiting for too long request ready flag to check staus?
//different gripper?

//remove bg
//TODO: hue wrap only updates on click not moving the sliders...
//reduce resolution to process faster? //erode dilate and looping all target colours are making it v slow...

//Future work
//Auto camera calibration
//GUI? //pick with buttons and do stuff based on click locations
//save params to a file?
//movement detection
//avoid things/target things (hand or insects) //dropping net on insect
//position correction?
//sterio vision

int main(void){

	std::cout << "Delta robot init..." << std::endl;
	if(deltaRobotInit() == -1){ //If the delta robots init fails
		return 0;
	}

	std::cout << "Computer vision init..." << std::endl;
	if(computerVisionInit() == -1){ //If the camera stream isn't open
		return 0;
	}

	while(1){
		updateFPS();
		printIncomingData();
		if (GetKeyState(VK_F8) & 0x8000){
			char data[10];
			std::cin.getline(data, 10);
			std::cin.clear();
			serialWrite(data, sizeof(data));
		}
		
		processNewFrameToHSV(); //Crops, masks, blurs and converts the image to the HSV colour space

		std::vector<cv::Point2i> targetsVect = getTargetCoordinates();

		if(targetsVect.size() > 0){
			std::cout << "Target coordinates:\n" << targetsVect << "\n" << std::endl;
			if(GetKeyState(VK_SPACE) & 0x8000){			
				sendCarteasianLinearWait(targetsVect[0].x, targetsVect[0].y, 60, 3000);
				sendCarteasianLinearWait(targetsVect[0].x, targetsVect[0].y, 40, 3000);
				sendCharArray((char*)"g0");
				Sleep(1500);
				sendCommand(COMMAND_MOVE_HOME);
				Sleep(1500);
				sendCharArray((char*)"g255");
			}
		}

		if (GetKeyState('H') & 0x8000){
			sendCommand(COMMAND_MOVE_HOME);
		}

		if (GetKeyState('O') & 0x8000){
			std::cout << "Ping!\n" << std::endl;
			sendCommand(COMMAND_PING_PONG);
		}

		if (GetKeyState('F') & 0x8000){
			send_program_file("D:\\Documents\\Projects\\Delta_robot\\test_arrays.txt", 4);
		}

		if(GetKeyState('A') & 0x8000){
			saveProgramToFile((char *)PATH_PROGRAM_ARRAYS);
		}

		drawAditionalElements(); //adds drawn elements to the camera stream

		demoDrawings();
		showOutputImage(); //Displays the camera stream with drawn elements

		if(cv::waitKey(1) == 27) break; //27 == esc
	}
	closeSerial();
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
