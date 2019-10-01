#ifndef COMPUTER_VISION_HPP_
#define COMPUTER_VISION_HPP_ 

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include <opencv2/opencv.hpp>

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define MAX_NUMBER_OF_TARGETS 6

#define X_PX_MM_RATIO 2.0
#define Y_PX_MM_RATIO 2.0

#define DELTA_ROBOT_WORKSPACE_RADIUS 105

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

struct TargetSpec{
	int hueMin = 0;
	int saturationMin = 0;
	int valueMin = 255; //initialised to 255 so black is not detected before a target colour is set
	int hueMax = 0;
	int saturationMax = 0;
	int valueMax = 0;
	int hueWrapMin = 0;
	int hueWrapMax = 0;

	double areaMin = 150;
	double areaMax = 800;

	int xPlacePos = -1;
	int yPlacePos = -1;
};

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

cv::Scalar ScalarHSV2BGR(uchar, uchar, uchar);
void setTargetArray(void);
static void on_index_trackbar(int, void*);
void createHSVTrackbars(void);
static void on_camera_prop(int, void*);
void createCameraPropertiesTrackbars(void);
cv::Vec2i pixelToDeltaCoordinates(int, int, cv::Size);
cv::Vec3b averagePixelValue(cv::Mat, int, int, int);
void on_mouseClick(int, int, int, int, void*);
bool validSize(double, double, double);
void drawPlacePositions(cv::InputOutputArray, int);
std::string typeToStr(int);
std::vector<cv::Point2i> getTargets(cv::InputArray, cv::OutputArray);
int computerVisionInit(void);
int processNewFrameToHSV(void);
void drawAditionalElements(void);
void showOutputImage(void);
std::vector<cv::Point2i> getTargetCoordinates(void);
void updateFPS(void);
void demoDrawings(void);

//int readSerialData(void);


/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#endif //COMPUTER_VISION_HPP_