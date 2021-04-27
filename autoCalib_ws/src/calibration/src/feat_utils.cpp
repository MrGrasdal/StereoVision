//
// Created by martin on 25.04.2021.
//

#include "../include/feat_utils.h"

void FeatureUtils::extractKpts(cv::Mat imgL, cv::Mat imgR, vPts& goodKptsL, vPts& goodKptsR, vMatch& good_matches, string descriptor, bool show) {

    Mat descL, descR;
    vKpts kptsL, kptsR;
    vector< DMatch > matches;


    if (descriptor == "ORB") {
        ROS_INFO("ORB");
        Ptr<ORB> detector = ORB::create();

        detector->detectAndCompute( imgL, noArray(), kptsL, descL );
        detector->detectAndCompute( imgR, noArray(), kptsR, descR );
    }
    else if (descriptor == "SIFT") {
        ROS_INFO("SIFT");

        Ptr<SIFT> detector = SIFT::create();

        detector->detectAndCompute( imgL, noArray(), kptsL, descL );
        detector->detectAndCompute( imgR, noArray(), kptsR, descR );

    }


    descL.convertTo(descL, CV_32F);
    descR.convertTo(descR, CV_32F);

    FlannBasedMatcher matcher;
    matcher.match( descL, descR, matches );


    int min_dist = 150;
    int noOfGoodMatches = 0;

    for( int i = 0; i < descL.rows; i++ )
    {
        if( matches[i].distance < 3*min_dist )
        {
            good_matches.push_back( matches[i]);

            //-- Get the keypoints from the good matches
            goodKptsL.push_back( kptsL[ matches[i].queryIdx ].pt );
            goodKptsR.push_back( kptsR[ matches[i].trainIdx ].pt );
        }
    }

    cv::Mat mask;

    // Find the Homography Matrix
    Mat H = findHomography( goodKptsL, goodKptsR, RANSAC,
                            2,  mask);

    vector< DMatch> best_matches;

    for (int i = 0; i < mask.rows; i++)
    {
        if ( mask.at<int>(i,0) == 1)
        {
            best_matches.push_back(good_matches[i]);
            noOfGoodMatches++;
        }
    }


    if (show) {

        //-- Draw matches
        Mat img_matches;

//        drawMatches(imgL, kptsL, imgR, kptsR, good_matches, img_matches,
//                    Scalar::all(-1), Scalar::all(-1),
//                    mask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

       drawMatches(imgL, kptsL, imgR, kptsR, best_matches, img_matches,
                   Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        imshow("Matches", img_matches );
        waitKey(1);

        ROS_INFO("No of matches: %i", noOfGoodMatches);

    }
}

void FeatureUtils::saveStereoImage(cv::Mat imgL, cv::Mat imgR, double ts) {

    std::string savePath = "/media/martin/Samsung_T5/imgs/test/";

    imwrite(savePath + "left/" + std::to_string(ts) + ".bmp", imgL);
    imwrite(savePath + "right/" + std::to_string(ts) + ".bmp", imgR);
}

void FeatureUtils::printStereoImage(cv::Mat imgL, cv::Mat imgR) {

    cv::Mat cv_img;
    cv::hconcat(imgL, imgR, cv_img);

    namedWindow("Left \t\t\t\t\t\t\t\t\t Right", cv::WINDOW_FULLSCREEN);
    cv::imshow("Left \t\t\t\t\t\t\t\t\t Right", cv_img);
    cv::waitKey(1);

}