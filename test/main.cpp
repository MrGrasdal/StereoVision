#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;


int main() {
    cout << "OpenCV version : " <<      CV_VERSION << endl;
    cout << "Major version : " <<       CV_MAJOR_VERSION << endl;
    cout << "Minor version : " <<       CV_MINOR_VERSION << endl;
    cout << "Subminor version : " <<    CV_SUBMINOR_VERSION << endl;
    return 0;
}
