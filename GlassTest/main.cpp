#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main() {

    Mat imgAvs = imread("../ColorTest/MedAvstand.bmp");
    Mat imgUten = imread("../ColorTest/UtenGlass2.bmp");
    Mat imgMed = imread("../ColorTest/MedGlass.bmp");

    float blueA, blueU, blueM, greenA, greenU, greenM, redA, redU, redM;
    Vec3b intensity;

    float avgBU = 0.0, avgBA = 0.0, avgBM = 0.0, avgGU = 0.0, avgGA = 0.0, avgGM = 0.0, avgRU = 0.0, avgRA = 0.0, avgRM = 0.0;
    float avgBUA = 0.0, avgBUM = 0.0, avgGUA = 0, avgGUM = 0.0, avgRUA = 0.0, avgRUM = 0.0;

    for (int x = 0; x < imgAvs.cols; ++x){
        for (int y = 0; y < imgAvs.rows; ++y) {

            intensity = imgAvs.at<Vec3b>(y,x);
            blueA = intensity.val[0];
            greenA = intensity.val[1];
            redA = intensity.val[2];

            intensity = imgUten.at<Vec3b>(y,x);
            blueU = intensity.val[0];
            greenU = intensity.val[1];
            redU = intensity.val[2];

            intensity = imgMed.at<Vec3b>(y,x);
            blueM = intensity.val[0];
            greenM = intensity.val[1];
            redM = intensity.val[2];

            if (x % 500 == 0 && y % 500 == 0 ) {
                cout << "\n\n Pixel " << x << " " << y;
                cout << "\n Uten Blue: " << blueU << "  Green: " << greenU << "  Red: " << redU;
                cout << "\n Avs Blue: " << blueA << "  Green: " << greenA << "  Red: " << redA;
                cout << "\n MedBlue: " << blueM << "  Green: " << greenM << "  Red: " << redM;
            }

            avgBA += blueA;
            avgBU += blueU;
            avgBM += blueM;
            avgGA += greenA;
            avgGM += greenM;
            avgGU += greenU;
            avgRA += redA;
            avgRM += redM;
            avgRU += redU;

            avgBUA += blueU - blueA;
            avgBUM += blueU - blueM;
            avgGUA += greenU - greenA;
            avgGUM += greenU - greenM;
            avgRUA += redU - redA;
            avgRUM += redU - redM;

        }
    }

    avgBA = avgBA / (imgAvs.cols * imgAvs.rows);
    avgBU = avgBU / (imgAvs.cols * imgAvs.rows);
    avgBM = avgBM / (imgAvs.cols * imgAvs.rows);
    avgGA = avgGA / (imgAvs.cols * imgAvs.rows);
    avgGM = avgGM / (imgAvs.cols * imgAvs.rows);
    avgGU = avgGU / (imgAvs.cols * imgAvs.rows);
    avgRA = avgRA / (imgAvs.cols * imgAvs.rows);
    avgRM = avgRM / (imgAvs.cols * imgAvs.rows);
    avgRU = avgRU / (imgAvs.cols * imgAvs.rows);

    avgBUA = avgBUA / (imgAvs.cols * imgAvs.rows);
    avgBUM = avgBUM / (imgAvs.cols * imgAvs.rows);
    avgGUA = avgGUA / (imgAvs.cols * imgAvs.rows);
    avgGUM = avgGUM / (imgAvs.cols * imgAvs.rows);
    avgRUA = avgRUA / (imgAvs.cols * imgAvs.rows);
    avgRUM = avgRUM / (imgAvs.cols * imgAvs.rows);

    cout << "\n\nSnittfarge";
    cout << "\nUten Blue: " << avgBU << "  Green: " << avgGU << "  Red: " << avgRU;
    cout << "\nAvs Blue: " << avgBA << "  Green: " << avgGA << "  Red: " << avgRA;
    cout << "\nMedBlue: " << avgBM << "  Green: " << avgGM << "  Red: " << avgRM;

    cout << "\n\nSnitt differanse";
    cout << "\nBlå UA: " << avgBUA << " UM: " << avgBUM;
    cout << "\nGrønn UA: " << avgGUA << " UM: " << avgGUM;
    cout << "\nRød UA: " << avgRUA << " UM: " << avgRUM;


//    Vec3b intensity = img.at<Vec3b>(1000, 1000);
//    float blue = intensity.val[0];
//    float green = intensity.val[1];
//    float red = intensity.val[2];

/*
    cout << "Blue: " << blue << "  Green: " << green << "  Red: " << red;



    intensity = img.at<Vec3b>(1000, 1000);
    blue = intensity.val[0];
    green = intensity.val[1];
    red = intensity.val[2];


    cout << "\nBlue: " << blue << "  Green: " << green << "  Red: " << red;

*/
    namedWindow("show", WINDOW_NORMAL);
    resizeWindow("show", 612  , 512);



    imshow("show", imgUten);

    waitKey();

    return 0;
}