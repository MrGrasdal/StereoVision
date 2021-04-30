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
#include <calibration/gnssGGA.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
//#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <string.h>
#include <iostream>

#include "feat_utils.h"
#include "camera.h"
#include "camModel.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv_bridge;
using namespace std;
using namespace cv;

class Calibrator {

public:
    FeatureUtils _feat;

    Camera camLeft(false);
    Camera camRight(true);

    int noOfEpochs = 0;
    int totalMatches = 0;
    int averageMatches = 0;

    vector<KeyPoint> kptsL, kptsR;
    Mat descL, descR;
    vector<DMatch> matches;

    void imageCallback(const sensor_msgs::ImageConstPtr &imgL_msg,
                       const sensor_msgs::ImageConstPtr &imgR_msg,
                       const calibration::gnssGGA::ConstPtr &gnss_msg);
};


void Calibrator::imageCallback(const sensor_msgs::ImageConstPtr &imgL_msg,
                               const sensor_msgs::ImageConstPtr &imgR_msg,
                               const calibration::gnssGGA::ConstPtr &gnss_msg) {


    double gnss_t = double(gnss_msg->header.stamp.sec) + double(gnss_msg->header.stamp.nsec)*1e-9;
    double imgL_t = double(imgL_msg->header.stamp.sec) + double(imgL_msg->header.stamp.nsec)*1e-9;

    ROS_INFO("%f", imgL_t);

    vector<DMatch> newMatches;
    Camera newLeft( imgL_t);
    Camera newRight( imgL_t, right);

    newLeft.img = cv_bridge::toCvCopy(imgL_msg, image_encodings::BGR8)->image;
    newRight.img = cv_bridge::toCvCopy(imgR_msg, image_encodings::BGR8)->image;

    //_feat.saveStereoImage(imgL, imgR, imgL_t);
    //_feat.printStereoImage(imgL, imgR);


    int firstMatch = 0;
    int secondMatch = 0;

    _feat.extractFeatures(newLeft, newRight, "SURF");

    _feat.matchFeatures(newLeft, newRight, newMatches,
                       firstMatch, "BF");

    //_feat.showMatches(newLeft, newRight, newMatches);

    if (noOfEpochs == 0)
    {
        ROS_INFO("first");
    }

    else
    {

        if ( newLeft.time == camLeft.time ){
            ROS_INFO("WTF");
        }
    }
        bool enoughMatches = _feat.matchFeatures(camLeft, newLeft, newMatches,
                            secondMatch, "BF");

        if (enoughMatches) {
            _feat.showMatches(camLeft, newLeft, newMatches);
        }

    //Mat F = CameraModel::calcFundamental(cameraLeft, cameraRight);

    //Mat T = CameraModel::triTensor(cameraLeft.P, cameraLeft.P,
    //                               cameraLeftAhead.P, 1 ,2 , 1);


    noOfEpochs++;
    totalMatches += secondMatch;
    averageMatches = totalMatches / noOfEpochs;


    ROS_INFO("Timedifference %f", newLeft.time - camLeft.time);
    ROS_INFO("First matches: \t\t%i", firstMatch);
    ROS_INFO("Second matches: \t%i", secondMatch);
    ROS_INFO("Average matches: \t%i \n\n", averageMatches);

    camLeft = newLeft;
    camRight = newRight;

    matches = newMatches;


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
    sync.registerCallback(boost::bind(&Calibrator::imageCallback, _1, _2, _3));

    ros::spin();
}

