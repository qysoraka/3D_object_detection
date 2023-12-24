# 3D_object_detection
The source code provided is utilized for object detection from 2D and specifies the position in 3D coordinate of the object from RealSense D435 camera under ROS platform. An HSV algorithm is utilized to filter single color from camera stream for object detection. 

For further inquiries or suggestions, please contact through the following email: qysoraka@example.com

I'm eager to engage in discussions and further develop the project. Below is the process to use this code: 

 $ catkin_make 
 $ roslaunch realsense2_camera rs_rgbd.launch 
 $ roslaunch realsense2_camera rs_rgbd.launch 
 $ rosrun opencv_object_tracking object_filter