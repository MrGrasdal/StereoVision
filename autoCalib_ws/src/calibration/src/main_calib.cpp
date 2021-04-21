//
// Created by martin on 20.04.2021.
//

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv_bridge;


void imageCallback(const sensor_msgs::ImageConstPtr &imgL, const sensor_msgs::ImageConstPtr &imgR) {
    try {
        cv::Mat cv_imgR = cv_bridge::toCvCopy(imgR, image_encodings::BGR8)->image;
        cv::Mat cv_imgL = cv_bridge::toCvCopy(imgL, image_encodings::BGR8)->image;

        cv::Mat cv_img;
        cv::hconcat(cv_imgL, cv_imgR, cv_img);

        namedWindow("Left \t\t\t\t\t\t\t\t\t Right", cv::WINDOW_FULLSCREEN);
        cv::imshow("Left \t\t\t\t\t\t\t\t\t Right", cv_img);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgL->encoding.c_str());
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    cv::namedWindow("Left \t\t\t\t\t\t\t\t\t Right");

    message_filters::Subscriber <Image> imgL_sub(nh, "/camera_array/left/image_raw", 10);
    message_filters::Subscriber <Image> imgR_sub(nh, "/camera_array/right/image_raw", 10);
    TimeSynchronizer <Image, Image> sync(imgL_sub, imgR_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    ros::spin();
    cv::destroyWindow("Left \t\t\t\t\t\t\t\t\t Right");
}
