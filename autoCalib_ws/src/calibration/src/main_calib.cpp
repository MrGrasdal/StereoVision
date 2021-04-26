//
// Created by martin on 20.04.2021.
//

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <calibration//gnssGGA.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string.h>
#include <iostream>

#include "../include/feat_utils.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv_bridge;
using namespace std;
using namespace cv;


FeatureUtils _feat;


void imageCallback(const sensor_msgs::ImageConstPtr &imgL_msg,
                   const sensor_msgs::ImageConstPtr &imgR_msg,
                   const calibration::gnssGGA::ConstPtr &gnss_msg) {

    double gnss_t = double(gnss_msg->header.stamp.sec) + double(gnss_msg->header.stamp.nsec)*1e-9;
    double imgL_t = double(imgL_msg->header.stamp.sec) + double(imgL_msg->header.stamp.nsec)*1e-9;

    cv::Mat imgL = cv_bridge::toCvCopy(imgL_msg, image_encodings::BGR8)->image;
    cv::Mat imgR = cv_bridge::toCvCopy(imgR_msg, image_encodings::BGR8)->image;

    //_feat.saveStereoImage(imgL, imgR, imgL_t);
    //_feat.printStereoImage(imgL, imgR);

    vector<Point2f> kptsL, kptsR;
    vector<DMatch> matches;

    _feat.extractKpts(imgL, imgR, kptsL, kptsR, matches, "SURF", true);

    int i = 0;



}


int main(int argc, char **argv) {

    ROS_INFO("Starting node");

    ros::init(argc, argv, "autoCalibration");
    ros::NodeHandle nh;

    message_filters::Subscriber<Image> imgL_sub(nh, "/camera_array/left/image_raw", 1);
    message_filters::Subscriber<Image> imgR_sub(nh, "/camera_array/right/image_raw", 1);
    message_filters::Subscriber<calibration::gnssGGA> gnss_sub(nh, "/vectorVS330/fix", 1);

    typedef sync_policies::ApproximateTime<Image, Image, calibration::gnssGGA> MySyncPolicy;
    //typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imgL_sub, imgR_sub, gnss_sub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    ros::spin();
}
