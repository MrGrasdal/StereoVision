//
// Created by martin on 19.04.2021.
//

#include "autoCalibration.h"

using std::endl;
using std::cout;

using namespace cv;
using namespace cv::xfeatures2d;


void AutoCalibration::matchFeatures( ) {
    Mat img1 = imread("../Images/right.bmp", IMREAD_GRAYSCALE);
    Mat img2 = imread("../Images/left.bmp", IMREAD_GRAYSCALE);
    if (img1.empty() || img2.empty()) {
        cout << "Could not open or find the image!\n" << endl;
        //parser.printMessage();
        return;
    }
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    //int minHessian = 400;
    Ptr <FeatureDetector> detector = ORB::create(); // minHessian );
    std::vector <KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, noArray(), keypoints2, descriptors2);
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr <DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
    std::vector <std::vector<DMatch>> knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector <DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    //-- Draw matches
    Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
                Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //-- Show detected matches
    imshow("Good Matches", img_matches);
    waitKey();

    return;

}