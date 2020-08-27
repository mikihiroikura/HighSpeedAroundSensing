#define _CRT_SECURE_NO_WARNINGS
#include <thread>
#include <opencv2/opencv.hpp>
#include <conio.h>
#include <vector>
#include <cstring>

#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

using namespace std;

int main() {
	string imgdir = "fisheyeimg2.png";
	cv::Mat fisheye,fishgrey, fisheyecpy;
	fisheye = cv::imread(imgdir);

    fisheyecpy = fisheye.clone();
	cv::cvtColor(fisheye, fishgrey, CV_BGR2GRAY);

	cv::GaussianBlur(fishgrey, fishgrey, cv::Size(3, 3), 2, 2);
	vector<cv::Vec3f> circles;
	cv::HoughCircles(fishgrey, circles, CV_HOUGH_GRADIENT, 2, fishgrey.rows/4, 100, 100);


    cv::Point imgcenter(fisheye.rows / 2, fisheye.cols / 2);
    cv::Vec3f optimal_circle;
    float dist_min = 1920;
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // ‰~‚Ì’†S‚ð•`‰æ‚µ‚Ü‚·D
        circle(fisheyecpy, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // ‰~‚ð•`‰æ‚µ‚Ü‚·D
        circle(fisheyecpy, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

        //‰æ‘œ’†S‚ÉÅ‚à‹ß‚¢‰~‚ðŒŸo‚·‚é
        float dist = cv::norm(imgcenter - center);
        if (dist<dist_min){
            optimal_circle = circles[i];
            dist_min = dist;
        }
    }
    cv::namedWindow("circles", 1);
    cv::imshow("circles", fisheyecpy);
    if (optimal_circle[2] > fisheyecpy.rows / 3) cout << "OK!" << endl;

    cv::Scalar lower(0, 0, 200);
    cv::Scalar upper(100, 100, 255);

    cv::Mat red_mask, output_mask;

    cv::inRange(fisheye, lower, upper, red_mask);
    fisheye.copyTo(output_mask, red_mask);

    cv::Mat erode_img;

    cv::erode(output_mask, erode_img, cv::Mat(), cv::Point(-1, -1), 1);

	return 0;
}