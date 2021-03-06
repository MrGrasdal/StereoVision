#include <iostream>
#include "opencv2/core.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"

#include "autoCalibration.h"
#include "functions.h"

//#include <rosbag/bag.h>
//#include <rosbag/view.h>

using namespace cv;
using namespace std;
//using namespace cv::xfeatures2d;

const string path_bags = "/media/martin/Samsung_T5/rosbags/";


int main(int argc, char **argv) {

    AutoCalibration autoCalib;
    autoCalib.matchFeatures();

    cout << "OpenCV version : " << CV_VERSION << endl;
    cout << "Major version : " << CV_MAJOR_VERSION << endl;
    cout << "Minor version : " << CV_MINOR_VERSION << endl;
    cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;
/*
    rosbag::Bag bag;
    bag.open(path_bags+"d2autoClose.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
*/

}
