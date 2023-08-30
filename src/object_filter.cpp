
/* Copyright by Huong Do Van - 10/11/2018
Any suggestion or advice, pls send via email: vanhuong.robotics@gmail.com
*/
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv_object_tracking/position_publish.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include<sstream>
// Add new topic
#include "geometry_msgs/Point.h"
#include <iostream>
#include <vector>
// Create new for circle drawing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Add global variable for pixel.
bool flag;
int posX = 0;
int posY = 0;
float X_111 = 0.0;
float Y_111 = 0.0;
float Z_111 = 0.0;
float x_value, y_value, z_value;
int x_position, y_position, z_position;
//End of global variable.
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::PointCloud2 my_pcl;
using namespace std;
using namespace cv;
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//Publishe new topic
ros::Publisher *pub;
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS = 50;
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

static const std::string OPENCV_WINDOW = "Image Window";
static const std::string windowName1 = "HSV image";
static const std::string windowName2 = "Thresholded Image";
static const std::string windowName3 = "After Morphological Operations";
static const std::string trackbarWindowName = "Track bars";

void on_trackbar(int, void*){}
string intToString(int number)
{
        std::stringstream ss;
        ss << number;
        return ss.str();
}
void createTrackbars()
{
        //Create window for trackbars
        namedWindow("Track_bars", 0);
        char TrackbarName[50];
        createTrackbar("H_MIN", "Track_bars", &H_MIN, H_MAX, on_trackbar);
        createTrackbar("H_MAX", "Track_bars", &H_MAX, H_MAX, on_trackbar);
        createTrackbar("S_MIN", "Track_bars", &S_MIN, S_MAX, on_trackbar);
        createTrackbar("S_MAX", "Track_bars", &S_MAX, S_MAX, on_trackbar);
        createTrackbar("V_MIN", "Track_bars", &V_MIN, V_MAX, on_trackbar);
        createTrackbar("V_MAX", "Track_bars", &V_MAX, V_MAX, on_trackbar);
}

void drawObject(int x, int y, Mat &frame)
{