//
// Created by martin on 25.04.2021.
//

#include "feat_utils.h"


void FeatureUtils::extractFeatures(Camera& master, Camera& sec, string descriptor)
{
    if (descriptor == "SIFT") {

        Ptr<SIFT> detector = SIFT::create();
        detector->detectAndCompute(master.img, noArray(), master.kpts, master.desc);
        detector->detectAndCompute(sec.img, noArray(), sec.kpts, sec.desc);
    }

    else if (descriptor == "SURF") {

        Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create();
        detector->detectAndCompute(master.img, noArray(), master.kpts, master.desc );
        detector->detectAndCompute(sec.img, noArray(), sec.kpts, sec.desc );
    }

    else {

        Ptr<ORB> detector = ORB::create();
        detector->detectAndCompute(master.img, noArray(), master.kpts, master.desc );
        detector->detectAndCompute(sec.img, noArray(), sec.kpts, sec.desc );
    }

    master.desc.convertTo(master.desc, CV_32F);
    sec.desc.convertTo(sec.desc, CV_32F);

}

//(vKpts& kptsL, vKpts& kptsR, Mat& descL, Mat& descR,
//vMatch& matches, Mat& mask
//        string matcher) {


bool FeatureUtils::matchFeatures(Camera& master, Camera& sec, vMatch& matches,
                                 int& noOfMatches, string matcher) {

    if (matcher == "BF") {
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
        matcher->match(master.desc, sec.desc, matches);

    } else {
        FlannBasedMatcher matcher;
        matcher.match(master.desc, sec.desc, matches);
    }

    vMatch good_matches;
    vPts goodPtsM, goodPtsS;

    int min_dist = 150;

    for (int i = 0; i < master.desc.rows; i++) {
        if (matches[i].distance < 3 * min_dist) {
            good_matches.push_back(matches[i]);

            //-- Get the keypoints from the good matches
            goodPtsM.push_back(master.kpts[matches[i].queryIdx].pt);
            goodPtsS.push_back(sec.kpts[matches[i].trainIdx].pt);

            noOfMatches++;
        }
    }

    if (noOfMatches < 4)
    {
        return false;
    }

    noOfMatches = 0;

    Mat mask;
    // Find the Homography Matrix
    Mat H = findHomography(goodPtsM, goodPtsS, RANSAC,
                           2, mask);

    vMatch best_matches;
    vKpts bestKptsM, bestKptsS;
    Mat bestDescM, bestDescS;



    for (int i = 0; i < mask.rows; i++) {
        if (mask.at<int>(i, 0) == 1) {

            bestKptsM.push_back(master.kpts[good_matches[i].queryIdx]);
            bestKptsS.push_back(sec.kpts[good_matches[i].trainIdx]);

            bestDescM.push_back( master.desc.row(good_matches[i].queryIdx) );
            bestDescS.push_back( sec.desc.row(good_matches[i].trainIdx) );


            best_matches.push_back( DMatch( noOfMatches,noOfMatches,0,good_matches[i].distance));
            //best_matches[i].queryIdx = i;
            //best_matches[i].trainIdx = i;
            noOfMatches++;
        }
    }

    master.kpts = bestKptsM;
    master.desc = bestDescM;
    sec.kpts = bestKptsS;
    sec.desc = bestDescS;

    matches = best_matches;

    return true;
}

void FeatureUtils::showMatches(Camera master, Camera sec, vMatch matches)
{
    //-- Draw matches
    Mat img_matches;

    drawMatches(master.img, master.kpts,
                sec.img, sec.kpts, matches, img_matches);

    imshow("Matches", img_matches );
    waitKey(1);


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