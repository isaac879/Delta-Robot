#include "computer_vision.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include <chrono>
#include <string>

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

using namespace std;
using namespace cv;

//Input videos stream
VideoCapture stream(0);

const char windowName[] = "Threshold Stream";

const char camera_properties[] = "Camera Properties";

int x_coord = 0;
int y_coord = 0;

int target_size_min = 150;
int target_size_max = 800;

int biggest_contour = 0; //Variable to store the index of the larges contour

//Global HSV range variables
int H_min = 0;
int H_max = 179;
int S_min = 0;
int S_max = 255;
int V_min = 0;
int V_max = 255;

int H_wrap_min = 0;
int H_wrap_max = 0;

//HSV range variables
int H_range = 10;
int S_range = 30;
int V_range = 30;

//Camera property variables
int camera_prop_brightness = 176; //-64 - 128
int camera_prop_contrast = 30;//0 - 64
int camera_prop_sharpness = 6;//0 - 6
int camera_prop_gamma = 4;//0 - 8
int camera_prop_hue = 8;// -8 - 8
int camera_prop_saturation = 70;//1-128

int target_array_index = 0;
TargetSpec target_array[MAX_NUMBER_OF_TARGETS];

//Global Matricies
Mat hsv_image, masked_src_image;

//TODO: clean up the global stuff
chrono::high_resolution_clock::time_point t1;
chrono::duration<double, milli> time_avg_arr[10];
int fps_interator = 0;
double fps = 0;

String fps_string = "FPS: Error";

//TODO: points p1 and p2 should be calculated as the ratio of the capture size so the code doesn't break if the capture resolution changes.
Point2f p1_delta[] = { //First set of points to calculate the perspective transform 
	{302, 124}, {518, 125},
	{289, 361}, {551, 345},
};

Point2f p2_delta[] = { //Second set of points to calculate the perspective transform 
	{90, 90}, {390, 90},
	{90, 390}, {390, 390},
};

Mat pt_matrix_delta = getPerspectiveTransform(p1_delta, p2_delta);
Mat src_image, threshold_image, threshold_wraparound_image, gaussian_image, output_stream;
Mat circle_workspace((int)sqrt(pow(p2_delta[3].x - p2_delta[0].x, 2) * 2), (int)sqrt(pow(p2_delta[3].x - p2_delta[0].x, 2) * 2), CV_8UC1, Scalar(0, 0, 0));


/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

Scalar ScalarHSV2BGR(uchar H, uchar S, uchar V){
    Mat rgb;
    Mat hsv(1,1, CV_8UC3, Scalar(H,S,V));
    cvtColor(hsv, rgb, COLOR_HSV2BGR);
    return Scalar(rgb.data[0], rgb.data[1], rgb.data[2]);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setTargetArray(void){ //Updates the target specs at the current index
	target_array[target_array_index].hueMin = H_min;
	target_array[target_array_index].hueMax = H_max;
	target_array[target_array_index].saturationMin = S_min;
	target_array[target_array_index].saturationMax = S_max;
	target_array[target_array_index].valueMin = V_min;
	target_array[target_array_index].valueMax = V_max;
	target_array[target_array_index].hueWrapMin = H_wrap_min;
	target_array[target_array_index].hueWrapMax = H_wrap_max;
	target_array[target_array_index].areaMin = target_size_min;
	target_array[target_array_index].areaMax = target_size_max;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

static void on_index_trackbar(int, void*){
	setTrackbarPos("H min", windowName, target_array[target_array_index].hueMin);
	setTrackbarPos("H max", windowName, target_array[target_array_index].hueMax);

	setTrackbarPos("S min", windowName, target_array[target_array_index].saturationMin);
	setTrackbarPos("S max", windowName, target_array[target_array_index].saturationMax);

	setTrackbarPos("V min", windowName, target_array[target_array_index].valueMin);
	setTrackbarPos("V max", windowName, target_array[target_array_index].valueMax);

	setTrackbarPos("Area min", windowName, target_array[target_array_index].areaMin);
	setTrackbarPos("Area max", windowName, target_array[target_array_index].areaMax);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void createHSVTrackbars(void){
	namedWindow(windowName, 0);

	createTrackbar("H min", windowName, &H_min, 179);
	createTrackbar("H max", windowName, &H_max, 179);
	createTrackbar("S min", windowName, &S_min, 255);
	createTrackbar("S max", windowName, &S_max, 255);
	createTrackbar("V min", windowName, &V_min, 255);
	createTrackbar("V max", windowName, &V_max, 255);

	createTrackbar("Area min", windowName, &target_size_min, 2000);
	createTrackbar("Area max", windowName, &target_size_max, 2000);

	createTrackbar("Target Index", windowName, &target_array_index, MAX_NUMBER_OF_TARGETS -1, on_index_trackbar);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void updateThresholdTrackbars(void) {
	setTrackbarPos("H min", windowName, H_min);
	setTrackbarPos("H max", windowName, H_max);

	setTrackbarPos("S min", windowName, S_min);
	setTrackbarPos("S max", windowName, S_max);

	setTrackbarPos("V min", windowName, V_min);
	setTrackbarPos("V max", windowName, V_max);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

static void on_camera_prop(int, void*){
	stream.set(CAP_PROP_BRIGHTNESS, (int)camera_prop_brightness - 64);
	stream.set(CAP_PROP_CONTRAST, camera_prop_contrast);
	stream.set(CAP_PROP_SHARPNESS, camera_prop_sharpness);
	stream.set(CAP_PROP_GAMMA, camera_prop_gamma);
	stream.set(CAP_PROP_HUE, (int)camera_prop_hue - 8);
	stream.set(CAP_PROP_SATURATION, (int)camera_prop_saturation + 1);	
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void createCameraPropertiesTrackbars(void){ //Property values were obtained from another program and I have no idea why they have these ranges...
	namedWindow(camera_properties, 0);
	createTrackbar("Brightness", camera_properties, &camera_prop_brightness, 192, on_camera_prop); //-64 - 128
	createTrackbar("Contrast", camera_properties, &camera_prop_contrast, 64, on_camera_prop);//0 - 64
	createTrackbar("Sharpness", camera_properties, &camera_prop_sharpness, 6, on_camera_prop);//0 - 6
	createTrackbar("Gamma", camera_properties, &camera_prop_gamma, 8, on_camera_prop);//0 - 8
	createTrackbar("Hue", camera_properties, &camera_prop_hue, 16, on_camera_prop);// -8 - 8
	createTrackbar("Saturation", camera_properties, &camera_prop_saturation, 127, on_camera_prop);//1 - 128
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

Vec2i pixelToDeltaCoordinates(int px, int py, Size matSize){
	//mm relitive to delta 0
	float xmm = (px - matSize.width / 2) / X_PX_MM_RATIO; //image centre (width)
	float ymm = (py - matSize.height / 2) / Y_PX_MM_RATIO; //image centre (height)

	Vec2i deltaCoord;
	deltaCoord.val[0] = round(ymm); //Delta robot x axis
	deltaCoord.val[1] = round(-xmm); //Delta robot Y axis

	return deltaCoord;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

Vec3b averagePixelValue(Mat image, int x, int y, int squareSize){
	if (squareSize % 2 != 1)squareSize++;//If the squre size is not odd then increment it by one to make it odd
	if (x < (squareSize - 1) / 2 || y < (squareSize - 1) / 2 || x > image.size().width - (squareSize - 1) / 2 || y > image.size().height - (squareSize - 1) / 2) return -1;//checks that the pixels will be valid
	Vec3b pix;
	int H = 0, S = 0, V = 0;
	int i, j;
	for (i = -(squareSize - 1) / 2; i <= (squareSize - 1) / 2; i++) {//loop through a squre of pixels and sum them
		for (j = -(squareSize - 1) / 2; j <= (squareSize - 1) / 2; j++) {
			pix = image.at<Vec3b>(y + j, x + i);//Fills the vector with the pixel values at the clicked location on the image
			H += pix[0];
			S += pix[1];
			V += pix[2];
		}
	}
	pix[0] = round(H / pow(squareSize, 2)); //Calculates the averate value and sets the element of the vector
	pix[1] = round(S / pow(squareSize, 2)); //Calculates the averate value and sets the element of the vector
	pix[2] = round(V / pow(squareSize, 2)); //Calculates the averate value and sets the element of the vector
	return pix; //Returns the vector containing the averaged pixel values
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void on_mouseClick(int event, int x, int y, int f, void*){ //Gets the pixel values from a mouseclick on a specified window and sets the min/max threshold values.
	if (event == EVENT_LBUTTONDOWN){ //If the left mouse button was pressed	
		x_coord = x;
		y_coord = y;

		//disparityPixelValue = (croppedDisparity.at<short>(y, x)) / 16;

		Vec3b pix = averagePixelValue(hsv_image, x_coord, y_coord, 1); //Fills the vector with the pixel values at the clicked location on the image

		H_min = pix.val[0] - H_range;
		H_max = pix.val[0] + H_range;
		S_min = pix.val[1] - S_range;
		S_max = pix.val[1] + S_range;
		V_min = pix.val[2] - V_range;
		V_max = pix.val[2] + V_range;

		if(H_min < 0){ //Used for hue wraparound
			H_min = 0;
			H_wrap_min = 179 + H_min; //H_min will be negative so is added
			H_wrap_max = 179;
		}
		else if(H_max > 179){
			H_max = 179;
			H_wrap_min = 0;
			H_wrap_max = H_max - 179;
		}
		else{
			H_wrap_min = 0;
			H_wrap_max = 0;
		}

		//cout << "H_min: " << H_min << "\tH_max: " << H_max << "\tS_min: " << S_min << "\tS_max: " << S_max << "\tV_min: " << V_min << "\tV_max: " << V_max << "\tH_wrap_min: " << H_wrap_min << "\tH_wrap_max: " << H_wrap_max << endl;;
		updateThresholdTrackbars();

		setTargetArray();
	}
	else if(event == EVENT_RBUTTONDOWN){
		target_array[target_array_index].xPlacePos = x;
		target_array[target_array_index].yPlacePos = y;
		cout << "Xpx: " << x << "\tYpy: " << y << endl;

		Vec2i deltaxy = pixelToDeltaCoordinates(x, y, hsv_image.size());
		cout << "Xmm: " << deltaxy[0] << "\tYmm: " << deltaxy[1] << endl;
	}
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool validSize(double minSize, double maxSize, double targetSize){
	if(targetSize >= minSize && targetSize <= maxSize){
		return true;
	}
	return false;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void drawPlacePositions(InputOutputArray src, int crossSize){
	for(int i = 0; i < MAX_NUMBER_OF_TARGETS; i++){
		if(target_array[i].xPlacePos > 0 && target_array[i].yPlacePos > 0){
			line(src, Point(target_array[i].xPlacePos - crossSize, target_array[i].yPlacePos - crossSize), 
				Point(target_array[i].xPlacePos + crossSize, target_array[i].yPlacePos + crossSize), ScalarHSV2BGR((target_array[i].hueMin + target_array[i].hueMin) / 2, 255, 255), 2);
			line(src, Point(target_array[i].xPlacePos - crossSize, target_array[i].yPlacePos + crossSize), 
				Point(target_array[i].xPlacePos + crossSize, target_array[i].yPlacePos - crossSize), ScalarHSV2BGR((target_array[i].hueMin + target_array[i].hueMin) / 2, 255, 255), 2);
		}
	}
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

string typeToStr(int type){//Takes the type of a mat and returns the string corrosponding to the type
	string r;
	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}
	r += "C";
	r += (chans + '0');
	return r;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

vector<Point2i> getTargets(InputArray inputHSV, OutputArray dst){
	Mat output = dst.getMat();
	vector<Point2i> targetCoord;
	Mat threshold_image, threshold_wraparound_image;

	setTargetArray(); //sets the currently selected target index to the appropriate trackbar values

	for(int index = 0; index < MAX_NUMBER_OF_TARGETS; index++){	
		inRange(inputHSV, Scalar(target_array[index].hueMin, target_array[index].saturationMin, target_array[index].valueMin), 
							Scalar(target_array[index].hueMax, target_array[index].saturationMax, target_array[index].valueMax), threshold_image); //Creates a threshold image based on the HSV values

		if(target_array[index].hueWrapMin != 0 || target_array[index].hueWrapMax != 0){ //Implimens the hue wraparound
			inRange(inputHSV, Scalar(target_array[index].hueWrapMin, target_array[index].saturationMin, target_array[index].valueMin),
								Scalar(target_array[index].hueWrapMax, target_array[index].saturationMax, target_array[index].valueMax), threshold_wraparound_image); //Creates a threshold image based on the HSV values
			add(threshold_image, threshold_wraparound_image, threshold_image); //combines the two threshold images
		}

		erode(threshold_image, threshold_image, Mat(), Point(-1, -1), 1); //erode to remove noise
		dilate(threshold_image, threshold_image, Mat(), Point(-1, -1), 2); //dilate to fill holes
		erode(threshold_image, threshold_image, Mat(), Point(-1, -1), 1); //erode to reduce the threshold back to the original size
		
		if(index == target_array_index){
			imshow(windowName, threshold_image);
		}		

		vector<vector<Point> > pointVector;//Declaring the vector for findcontours function
		vector<Vec4i> hierarchy;//Declaring the hierarchy vector for find contours function
		int validContourIndexes[1000];
		findContours(threshold_image, pointVector, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));//Finds the outer contours of the threshold image

		int validIndex = 0;
		for (int i = 0; i < pointVector.size(); i++) {
			double area = contourArea(pointVector[i], false);//Calculates the area of the contour at index
			if(validSize(target_array[index].areaMin, target_array[index].areaMax, area)){//Checks if the area is withing the valid range
				validContourIndexes[validIndex] = i;//saves the valid contour indexes to an array
				validIndex++;
			}
		}

		double biggest_area = 0;//Declaring the variable and setting it to zero for each iteration of the while loop
		for (int index = 0; index < validIndex; index++) {//Finds the index of the largest contour
			double area = contourArea(pointVector[validContourIndexes[index]], false);//Calculates the area of the contour i
			if (area > biggest_area)//Checks to see if the area of the contour at the index is larger than the previous largest
			{
				biggest_area = area;//Assigns the new biggest contours area to the variable area
				biggest_contour = index;//Sets biggest_contour to the index
			}
		}

		vector<Point2f>centre(pointVector.size());
		vector<float>radius(pointVector.size());
		
		for (int i = 0; i < validIndex; i++) {
			minEnclosingCircle(Mat(pointVector[validContourIndexes[i]]), centre[i], radius[i]);
		}

		//get the moments
		vector<Moments> mu(pointVector.size());
		for (int i = 0; i < pointVector.size(); i++) {
			mu[i] = moments(pointVector[i], false);
		}

		//Get the centre of masses
		vector<Point2f> mc(pointVector.size());
		for (int i = 0; i < pointVector.size(); i++) {
			mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		if (!mc.empty() && validIndex != 0) {
			for (int i = 0; i < validIndex; i++) {
				circle(output, mc[validContourIndexes[i]], 3, Scalar(0, 0, 255), 5, 8); //Draws COM
				//targetCoord.push_back(mc[validContourIndexes[i]]);
				targetCoord.push_back(pixelToDeltaCoordinates(mc[validContourIndexes[i]].x, mc[validContourIndexes[i]].y, output.size())); //Fills the vector with the target coordinates converted to mm
			}
			circle(output, mc[validContourIndexes[biggest_contour]], 7, Scalar(255, 0, 0), 10, 8);//Draws a circle at the larges contours coordinates
		}

		for (int i = 0; i < validIndex; i++) {//Iterates through the contours that are valid
			drawContours(output, pointVector, validContourIndexes[i], Scalar(0, 255, 0), 2, LINE_AA);//Draws the contour i
		}

		for (int i = 0; i < pointVector.size(); i++) {//Draws a bounding circle			
			circle(output, centre[i], (int)radius[i], ScalarHSV2BGR((target_array[index].hueMin + target_array[index].hueMax) / 2, 255, 255), 2, 8, 0);
		}
	}
	return targetCoord;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int computerVisionInit(void){
	namedWindow(windowName, WINDOW_AUTOSIZE);
	namedWindow("Output Stream", WINDOW_AUTOSIZE);
	moveWindow("Output Stream", 0, 0);
	//moveWindow("input Stream", 0, 0);
	createHSVTrackbars();
	setMouseCallback("Output Stream", on_mouseClick, 0);//Sets the mouse handler for the disparity window
	createCameraPropertiesTrackbars();
	on_camera_prop(0, 0);
	circle(circle_workspace, Point(circle_workspace.size().width / 2, circle_workspace.size().height / 2), (sqrt(pow(p2_delta[3].x - p2_delta[0].x, 2) * 2) / 2)  + 20, Scalar(255, 255, 255), -1, 8, 0);
	//imshow("circle Stream", circle_workspace);
	if(!stream.isOpened()){
		cout << "Error: Camera stream is not open." << endl;
		return -1;
	}
	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

int processNewFrameToHSV(void){
	stream >> src_image; //gets a frame from the camera

	if(src_image.empty()){ //checks a frame was obtained
		cout << "Image empty..." << endl;
		return -1;
	}
	
	//circle(src_image, p1_delta[0], 5, Scalar(0, 0, 255), 2, 8, 0);
	//circle(src_image, p1_delta[1], 5, Scalar(0, 255, 0), 2, 8, 0);
	//circle(src_image, p1_delta[2], 5, Scalar(255, 0, 0), 2, 8, 0);
	//circle(src_image, p1_delta[3], 5, Scalar(255, 0, 255), 2, 8, 0);
	//imshow("input Stream", src_image);
	warpPerspective(src_image, src_image, pt_matrix_delta, src_image.size()); //Converts the image to a parallel perspective
		
	src_image = src_image(Rect(30, 30, sqrt(pow(p2_delta[3].x - p2_delta[0].x, 2) * 2), sqrt(pow(p2_delta[3].x - p2_delta[0].x, 2) * 2))); //Crops the image to the workspace size. Pixel values empirically determined after the perspective transform was implimented

	src_image.copyTo(masked_src_image, circle_workspace); //Maskes the image so it only contains the robots workspace

	GaussianBlur(masked_src_image, gaussian_image, Size(3, 3), 0, 0); //Gaussain Blur to smooth the image

	cvtColor(gaussian_image, hsv_image, COLOR_BGR2HSV); //Converts the colour spave from BGR to HSV

	return 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void drawAditionalElements(void){
	drawPlacePositions(src_image, 10); //draws an x at the place coordinates
	putText(src_image, fps_string, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, LINE_AA); //Draws the FPS counter on the output stream
	circle(src_image, Point(src_image.size().width / 2, src_image.size().height / 2), sqrt(pow(p2_delta[3].x - p2_delta[0].x, 2) * 2) / 2, Scalar(255, 0, 0), 1, 8, 0);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void demoDrawings(void){
	//imshow input, warpsed input, cropped input, show masked inpput, blured, hsv?, threshold, output
	//draw grid
	for(int i = 0; i <= src_image.size().height; i += src_image.size().height / 8){
		line(src_image, Point(0, i), Point(src_image.size().width, i), Scalar(0, 0, 0), 1, 8);
	}
	for(int i = 0; i <= src_image.size().width; i += src_image.size().width / 8){
		line(src_image, Point(i, 0), Point(i, src_image.size().height), Scalar(0, 0, 0), 1, 8);
	}

	line(src_image, Point(0, src_image.size().height / 2), Point(src_image.size().width, src_image.size().height / 2), Scalar(0, 0, 0), 2, 8);
	line(src_image, Point(src_image.size().width / 2, 0), Point(src_image.size().width / 2, src_image.size().height), Scalar(0, 0, 0), 2, 8);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void showOutputImage(void){
	imshow("Output Stream", src_image);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

vector<Point2i> getTargetCoordinates(void){
	//first to last
	//reverse order
	//random
	//vector<Point2i> targetsVect = 
	return getTargets(hsv_image, src_image);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void updateFPS(void){
	time_avg_arr[fps_interator] = chrono::high_resolution_clock::now() - t1;
	t1 = chrono::high_resolution_clock::now();

	double sum_time = 0; //Stores the sum of the time_avg_arr elements
	for(int i = 0; i < 10; i++){ //Sums the time_avg_arr elements
		sum_time += time_avg_arr[i].count();
	}

	fps = 1 / (sum_time /= 10000); //Sum_time is the sum of 10 elements and in ms not s so is devided by 10*1000

	fps_string = "FPS: " + to_string(fps).erase(4); //Gives a string to 3 significant figures

	if(++fps_interator >= 10){ //Impliments circular index for averaging the camera fps
		fps_interator = 0;
	}
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
