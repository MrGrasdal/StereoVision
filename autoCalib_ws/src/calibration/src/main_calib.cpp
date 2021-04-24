//
// Created by martin on 20.04.2021.
//

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>

#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <calibration//gnssGGA.h>
#include <iostream>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv_bridge;
using namespace std;


int i;
const string savePath = "/media/martin/Samsung_T5/imgs/calibRun/";
bool save = false;


void imageCallback(const sensor_msgs::ImageConstPtr &imgL,
                   const sensor_msgs::ImageConstPtr &imgR,
                   const calibration::gnssGGA::ConstPtr &gnss) {
    try {
        i++;
        if (i % 10 == 0)
        {
            cv::Mat cv_imgR = cv_bridge::toCvCopy(imgR, image_encodings::BGR8)->image;
            cv::Mat cv_imgL = cv_bridge::toCvCopy(imgL, image_encodings::BGR8)->image;

            cv::Mat cv_img;
            cv::hconcat(cv_imgL, cv_imgR, cv_img);

            std_msgs::Header h = imgL->header;
            double h_time = double(h.stamp.sec) + double(h.stamp.nsec)*1e-9;

            to_string(h_time);
            if (save) {
                //ROS_INFO("TS: '%f'", h_time);
                imwrite(savePath + "left/" + to_string(h_time) + ".bmp", cv_imgL);
                imwrite(savePath + "right/" + to_string(h_time) + ".bmp", cv_imgR);
            }

            namedWindow("Left \t\t\t\t\t\t\t\t\t Right", cv::WINDOW_FULLSCREEN);
            cv::imshow("Left \t\t\t\t\t\t\t\t\t Right", cv_img);
            //cv::waitKey(1);
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgL->encoding.c_str());
    }
}


int main(int argc, char **argv) {
    i = 0;

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    cv::namedWindow("Left \t\t\t\t\t\t\t\t\t Right");

    message_filters::Subscriber <Image> imgL_sub(nh, "/camera_array/left/image_raw", 1);
    message_filters::Subscriber <Image> imgR_sub(nh, "/camera_array/right/image_raw", 1);
    message_filters::Subscriber <calibration::gnssGGA> gnss_sub(nh, "custom_msgs/gnssGGA", 1);
    TimeSynchronizer <Image, Image, calibration::gnssGGA> sync(imgL_sub, imgR_sub, gnss_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    ros::spin();
    cv::destroyWindow("Left \t\t\t\t\t\t\t\t\t Right");
}
