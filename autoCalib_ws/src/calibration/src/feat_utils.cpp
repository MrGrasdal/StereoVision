//
// Created by martin on 25.04.2021.
//

#include "feat_utils.h"


void FeatureUtils::extractFeatures(Camera& query, Camera& train, string descriptor)
{
    if (descriptor == "SIFT") {
        ROS_INFO("SIFT");
        Ptr<SIFT> detector = SIFT::create();
        detector->detectAndCompute(query.img, noArray(), query.allKpts, query.allDesc);
        detector->detectAndCompute(train.img, noArray(), train.allKpts, train.allDesc);
    }

    else if (descriptor == "SURF") {

        Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create();
        detector->detectAndCompute(query.img, noArray(), query.allKpts, query.allDesc );
        detector->detectAndCompute(train.img, noArray(), train.allKpts, train.allDesc );
    }

    else {

        Ptr<ORB> detector = ORB::create();
        detector->detectAndCompute(query.img, noArray(), query.epochKpts, query.epochDesc );
        detector->detectAndCompute(train.img, noArray(), train.epochKpts, train.epochDesc );
    }

    query.epochDesc.convertTo(query.epochDesc, CV_32F);
    train.epochDesc.convertTo(train.epochDesc, CV_32F);
}

bool FeatureUtils::matchFeatures(Camera& query, Camera& train, vMatch& matches,
                                 int& noOfMatches, string matcher, bool epoch) {

    if (matcher == "BF") {
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
        matcher->match(query.allDesc, train.allDesc, matches);

    } else {
        FlannBasedMatcher matcher;
        matcher.match(query.allDesc, train.allDesc, matches);
    }

    vMatch good_matches;
    vPts goodPtsM, goodPtsS;

    int min_dist = 100;

    for (int i = 0; i < query.allDesc.rows; i++) {
        if (matches[i].distance < 3 * min_dist) {
            good_matches.push_back(matches[i]);

            //-- Get the keypoints from the good matches
            goodPtsM.push_back(query.allKpts[matches[i].queryIdx].pt);
            goodPtsS.push_back(train.allKpts[matches[i].trainIdx].pt);

            noOfMatches++;
        }
    }

    if (noOfMatches < 10)
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

            bestKptsM.push_back(query.allKpts[good_matches[i].queryIdx]);
            bestKptsS.push_back(train.allKpts[good_matches[i].trainIdx]);

            bestDescM.push_back(query.allDesc.row(good_matches[i].queryIdx));
            bestDescS.push_back(train.allDesc.row(good_matches[i].trainIdx));

            //best_matches.push_back(DMatch(noOfMatches, noOfMatches, 0, good_matches[i].distance));
            best_matches.push_back( good_matches[i] );


            noOfMatches++;
        }
    }

    if (epoch) {
        query.epochKpts = bestKptsM;
        query.epochDesc = bestDescM;
        train.epochKpts = bestKptsS;
        train.epochDesc = bestDescS;
    }

    else {
        query.currKpts = bestKptsM;
        query.currDesc = bestDescM;
        train.currKpts = bestKptsS;
        train.currDesc = bestDescS;
    }

    matches = best_matches;

    return true;
}

void FeatureUtils::find3pmatch(Camera& leftNxt, Camera& left, Camera& right, vMatch& epochMatches, vMatch& currMatches, int& noOfMatches) {

    noOfMatches = 0;

    vMatch  newEMatch, newCMatch;
    vKpts newKptsL, newKptsR, newKptsNxt;



    for (int i = 0; i < epochMatches.size(); ++i) {
        for (int j = 0; j < currMatches.size(); ++j) {
            if(epochMatches[i].trainIdx == currMatches[j].queryIdx) {
                newKptsNxt.push_back(leftNxt.allKpts[epochMatches[i].queryIdx]);
                newKptsL.push_back(left.allKpts[epochMatches[i].trainIdx]);
                newKptsR.push_back(right.allKpts[currMatches[j].trainIdx]);

                newEMatch.push_back(epochMatches[i]);
                newCMatch.push_back(currMatches[j]);

                noOfMatches++;
            }
        }
    }

    leftNxt.epochKpts = newKptsNxt;
    left.epochKpts = newKptsL;
    left.currKpts = newKptsL;
    right.currKpts = newKptsR;

    epochMatches = newEMatch;
    currMatches = newCMatch;
}

Mat FeatureUtils::draw3pMatches(Camera leftNxt, Camera left, Camera right) {

    Mat outImg,
        outImgGray,
        outImgInter,
        imgNxt = leftNxt.img,
        imgL = left.img,
        imgR = right.img;


    hconcat(imgNxt, imgL, outImgInter);
    hconcat(outImgInter, imgR, outImgGray);

    cvtColor(outImgGray, outImg,COLOR_BGRA2BGR);

    RNG& rng = theRNG();

    // draw matches
    for( int i = 0; i < left.currKpts.size(); i++ ) {
        rng = theRNG();
        bool isRandMatchColor = true;
        Scalar color = isRandMatchColor ? Scalar( rng(256), rng(256), rng(256), 255 ) : Scalar::all(-1);

        Point2f ptNxt = leftNxt.epochKpts[i].pt,
                ptL = left.currKpts[i].pt,
                ptR = right.currKpts[i].pt,
                dptL = Point2f(ptL.x + imgNxt.size().width, ptL.y),
                dptR = Point2f(ptR.x + outImgInter.size().width, ptR.y);

        drawKeypoint(outImg, ptNxt, color);
        drawKeypoint(outImg, dptL, color);
        drawKeypoint(outImg, dptR, color);


        line(outImg,
             Point(cvRound(ptNxt.x), cvRound(ptNxt.y)),
             Point(cvRound(dptL.x), cvRound(dptL.y)),
             color, 1, LINE_AA);

        line(outImg,
             Point(cvRound(dptL.x), cvRound(dptL.y)),
             Point(cvRound(dptR.x), cvRound(dptR.y)),
             color, 1, LINE_AA);
    }

    imshow("Matches", outImg );
    waitKey(1);

    return outImg;

}

void FeatureUtils::drawKeypoint( Mat& outImg, Point2f kpt, Scalar color)
{

    Point center( kpt.x, kpt.y);
    int radius = 3;
    circle(outImg, center, radius, color, 1, LINE_AA);

}


void FeatureUtils::showMatches(Camera master, Camera sec, vMatch matches, bool epoch)
{
    //-- Draw matches
    Mat img_matches;
    if (epoch) {
        drawMatches(master.img, master.allKpts,
                    sec.img, sec.allKpts, matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1),vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    }

    else {
        drawMatches(master.img, master.allKpts,
                    sec.img, sec.allKpts, matches, img_matches,Scalar::all(-1),
                    Scalar::all(-1),vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    }

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