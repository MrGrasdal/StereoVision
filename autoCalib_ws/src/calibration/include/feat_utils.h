//
// Created by martin on 25.04.2021.
//
#ifndef SRC_FEAT_UTILS_H
#define SRC_FEAT_UTILS_H

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/calib3d.hpp>

#include <string>
#include <iostream>
#include <ros/ros.h>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

class FeatureUtils{

    private:
        typedef vector<KeyPoint> vKpts;
        typedef vector< Point2f > vPts;
        typedef vector< DMatch > vMatch;


public:

        void extractKpts(cv::Mat imgL, cv::Mat imgR, vPts& kptsL, vPts& kptsR, vMatch& matches, string descriptor="ORB", bool show=false);

        void saveStereoImage(cv::Mat imgL, cv::Mat imgR, double ts);

        void printStereoImage(cv::Mat imgL, cv::Mat imgR);



};




#endif //SRC_FEAT_UTILS_H
