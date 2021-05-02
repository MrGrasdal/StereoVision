//
// Created by martin on 25.04.2021.
//
#ifndef SRC_FEAT_UTILS_H
#define SRC_FEAT_UTILS_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>

#include <string>
#include <iostream>
#include <ros/ros.h>

#include "camera.h"

using namespace cv;
using namespace std;

class FeatureUtils {

    private:
        typedef vector<KeyPoint> vKpts;
        typedef vector<Point2f> vPts;
        typedef vector<DMatch> vMatch;

    public:

        void extractFeatures(Camera &query, Camera &train, string descriptor = "SIFT");

        bool matchFeatures(Camera &query, Camera &train, vMatch &matches,
                           int &noOfMatches, string matcher = "FLANN", bool epoch=false);

        //bool matchFeatures(Camera &master, Camera &sec, vMatch &matches,
        //                   int &noOfMatches, string matcher = "FLANN", bool epoch=false);

        void showMatches(Camera master, Camera sec, vMatch matches, bool epoch=false);

        void saveStereoImage(cv::Mat imgL, cv::Mat imgR, double ts);

        void printStereoImage(cv::Mat imgL, cv::Mat imgR);

        bool epochMatching(Camera &master, Camera &sec, vMatch &matches, int &noOfMatches, string matcher);

    void drawKeypoint(Mat &outImg, Point2f kpt, Scalar color);

    Mat draw3pMatches(Camera leftNxt, Camera left, Camera right);

    void find3pmatch(Camera &leftNxt, Camera &left, Camera &right, vMatch &epochMatches, vMatch &currMatches,
                int &noOfMatches);
};


#endif //SRC_FEAT_UTILS_H
