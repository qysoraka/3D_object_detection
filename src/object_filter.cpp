
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
        circle(frame, Point(x, y), 40, Scalar(0, 255, 0), 2); //50
        if (y - 25 > 0)
                line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
        if (y + 25 < FRAME_HEIGHT)
                line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
        if (x - 25 > 0)
                line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
        if (x + 25 < FRAME_WIDTH)
                line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

        putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
        ::posX = x;
        ::posY = y;
        putText(frame, "X_Y_Z coordinate", Point(20, 200), 1, 2, Scalar(0, 255, 0), 2);
        putText(frame, "X = " + intToString(x_position) + "(mm)" , Point(20, 250), 1, 2, Scalar(0, 255, 0), 2);
        putText(frame, "Y = " + intToString(y_position) + "(mm)" , Point(20, 300), 1, 2, Scalar(0, 255, 0), 2);
        putText(frame, "Z = " + intToString(z_position) + "(mm)" , Point(20, 350), 1, 2, Scalar(0, 255, 0), 2);

}
void morphOps(Mat &thresh)
{
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);

        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{
        Mat temp;
        threshold.copyTo(temp);
        //These two vectors needed for output of findContours
        vector< vector<Point> > contours;
        vector<Vec4i> hierarchy;
        //Find contours of filtered image using openCV findContours function
        findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        //Use moments method to find our filtered object
        double refArea = 0;
        bool objectFound = false;
        if (hierarchy.size() > 0)
        {
                int numObjects = hierarchy.size();
                //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
                if (numObjects < MAX_NUM_OBJECTS)
                {
                        for (int index = 0; index >= 0; index = hierarchy[index][0])
                        {
                                Moments moment = moments((cv::Mat)contours[index]);
                                double area = moment.m00;
                                //if the area is less than 20 px by 20px then it is probably just noise
                                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                                //we only want the object with the largest area so we safe a reference area each
                                //iteration and compare it to the area in the next iteration.
                                if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                                {
                                        x = moment.m10 / area;
                                        y = moment.m01 / area;
                                        objectFound = true;
                                        ::flag = true;
                                        refArea = area;
                                }
                                else
                                {
                                 objectFound = false;
                                 ::flag = false;
                                }
                        }
                        //let user know you found an object
                        if (objectFound == true)
                        {
                                //::flag = true;
                                putText(cameraFeed, "Position Object tracking ", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
                                //draw object location on screen
                                drawObject(x, y, cameraFeed);
                        }

                }
                else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
        }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    bool trackObjects = true; //false
	bool useMorphOps = true;  //false

	Mat HSV;
	Mat threshold;
	int x = 0, y = 0;
	createTrackbars();
	std::cout << " The output of Object tracking by OpenCV!\n";

    cvtColor(cv_ptr->image, HSV, COLOR_BGR2HSV);

    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);

    if (useMorphOps)
        morphOps(threshold);

    if (trackObjects)
        trackFilteredObject(x, y, threshold, cv_ptr->image);
    //show frames
    imshow(windowName2, threshold);
    //imshow(OPENCV_WINDOW, cameraFeed);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //imshow(windowName1, HSV);
    cv::waitKey(3);

}

void getXYZ(int x, int y)
{
    int arrayPosition = y*my_pcl.row_step + x*my_pcl.point_step;
    int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    geometry_msgs::Point p;

    memcpy(&X, &my_pcl.data[arrayPosX], sizeof(float));
    memcpy(&Y, &my_pcl.data[arrayPosY], sizeof(float));
    memcpy(&Z, &my_pcl.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;