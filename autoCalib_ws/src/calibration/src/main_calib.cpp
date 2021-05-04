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
#include <calibration/orientation.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
//#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <string.h>
#include <iostream>
#include "Eigen/Dense"

#include "feat_utils.h"
#include "camera.h"
#include "camModel.h"
#include "EKF.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv_bridge;
using namespace std;
using namespace cv;

class CalibNode {

    public:
        CalibNode();

        void imageCallback(const sensor_msgs::ImageConstPtr &imgL_msg,
                           const sensor_msgs::ImageConstPtr &imgR_msg,
                           const calibration::gnssGGA::ConstPtr &gnss_msg,
                           const calibration::orientation::ConstPtr &att_msg);
    private:

        EKF filter;

        Camera camLeft;
        Camera camRight;
        vector<DMatch> currMatches;

        String descriptor = "SIFT";
        String matcher = "FLANN";

        int noOfEpochs;
        int totalMatches;
        int averageMatches;

        FeatureUtils _feat;

        ros::NodeHandle nh;

        message_filters::Subscriber<Image> imgL_sub;
        message_filters::Subscriber<Image> imgR_sub;
        message_filters::Subscriber<calibration::gnssGGA> gnss_sub;
        message_filters::Subscriber<calibration::orientation> att_sub;

        typedef sync_policies::ApproximateTime<Image, Image,calibration::gnssGGA, calibration::orientation> MySyncPolicy;
        typedef Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;

};


CalibNode::CalibNode() {

    camLeft.initCamera();
    camRight.initCamera(true);
    filter.Init(camRight);

    noOfEpochs = 0;
    totalMatches = 0;
    averageMatches = 0;

    imgL_sub.subscribe(nh, "/camera_array/left/image_raw", 1);
    imgR_sub.subscribe(nh, "/camera_array/right/image_raw", 1);
    gnss_sub.subscribe(nh, "/vectorVS330/fix", 1);
    att_sub.subscribe(nh, "/xsens/orientation", 1);
    sync.reset(new Sync(MySyncPolicy(10), imgL_sub, imgR_sub, gnss_sub, att_sub));
    sync->registerCallback(boost::bind(&CalibNode::imageCallback, this, _1, _2, _3, _4));
}


void CalibNode::imageCallback(const sensor_msgs::ImageConstPtr &imgL_msg,
                              const sensor_msgs::ImageConstPtr &imgR_msg,
                              const calibration::gnssGGA::ConstPtr &gnss_msg,
                              const calibration::orientation::ConstPtr &att_msg) {


    double gnss_t = double(gnss_msg->header.stamp.sec) + double(gnss_msg->header.stamp.nsec) * 1e-9;
    double imgL_t = double(imgL_msg->header.stamp.sec) + double(imgL_msg->header.stamp.nsec) * 1e-9;

    double dt = imgL_t - camLeft.time;

    if ( dt < 1) {
        return;
    }
    //ROS_INFO("%f", imgL_t);

    vector<DMatch> newMatches, epochMatches;
    Camera newLeft(imgL_t);
    Camera newRight(imgL_t, right);

    newLeft.model.unpackPosData(gnss_msg,att_msg);

    newLeft.img = cv_bridge::toCvCopy(imgL_msg, image_encodings::MONO8)->image;
    newRight.img = cv_bridge::toCvCopy(imgR_msg, image_encodings::MONO8)->image;

    //_feat.saveStereoImage(imgL, imgR, imgL_t);
    //_feat.printStereoImage(imgL, imgR);

    int firstMatch = 0;
    int secondMatch = 0;
    int noOf3pMatch = 0;

    _feat.extractFeatures(newLeft, newRight, descriptor);

    _feat.matchFeatures(newLeft, newRight, newMatches,
                        firstMatch, matcher);


    if (noOfEpochs == 0) {
        ROS_INFO("first");

        currMatches = newMatches;
        camLeft = newLeft;
        camRight = newRight;

        ROS_INFO("Timedifference %f", dt);
        ROS_INFO("First matches: \t\t%i", firstMatch);

        noOfEpochs++;


        return;
    }


    _feat.matchFeatures(newLeft, camLeft, epochMatches,
                        secondMatch, matcher, true);

    _feat.find3pmatch(newLeft, camLeft, camRight,
                      epochMatches, currMatches, noOf3pMatch);

    //_feat.draw3pMatches(newLeft, camLeft, camRight);

    if(noOf3pMatch < 10)
    {
        ROS_INFO("Timedifference %f", dt);
        ROS_INFO("First matches: \t\t%i", firstMatch);
        ROS_INFO("Second matches: \t%i", secondMatch);
        ROS_INFO("3 point matches: \t%i", noOf3pMatch);
        ROS_INFO("Not enough matches. SKIPPED");

        currMatches = newMatches;
        camLeft = newLeft;
        camRight = newRight;

        return;
    }

    Eigen::Vector3f dgps = newLeft.model.GNSSpos - camLeft.model.GNSSpos;

    ROS_INFO("Difference in position: [%f, %f, %f]", dgps[0], dgps[1], dgps[2]);

    totalMatches += noOf3pMatch;
    averageMatches = totalMatches / noOfEpochs;

    ROS_INFO("Timedifference %f", dt);
    ROS_INFO("First matches: \t\t%i", firstMatch);
    ROS_INFO("Second matches: \t%i", secondMatch);
    ROS_INFO("3 point matches: \t%i", noOf3pMatch);
    ROS_INFO("Average matches: \t%i \n", averageMatches);


    currMatches = newMatches;
    camLeft = newLeft;
    camRight = newRight;

    //filter.process () ;

    noOfEpochs++;
}


int main(int argc, char **argv) {

    FeatureUtils _feat;

    ROS_INFO("Starting node");

    ros::init(argc, argv, "autoCalibration");

    CalibNode calibNode;

    ros::spin();
}

