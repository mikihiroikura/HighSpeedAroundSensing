#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <iostream>
#include <vector>
#include <cstdio>

using namespace std;

int main() {
	string video_dir = "202009302029_video.mp4";
	cv::VideoCapture video;
	video.open(video_dir);
	if (!video.isOpened()) {
		cout << "video cannot ne opened..." << endl;
		return 0;
	}

	cv::Point2f bias(10.0, 1000.2);

	while (video.grab())
	{
		cv::Mat  image, imageCopy, mask ,nonzero;
		cv::Point2f out;
		
		vector<cv::Point> bps;
		video.retrieve(image);
		image.copyTo(imageCopy);

		cv::threshold(imageCopy, mask, 240, 255, cv::THRESH_BINARY);

		cv::cvtColor(mask, mask, CV_RGB2GRAY);
		cv::findNonZero(mask,bps);

		out.x = bps[0].x + bias.x;
		out.y = bps[0].y - bias.y;

		cv::imshow("out", image);
		cv::imshow("masked", mask);
		char key = (char)cv::waitKey(1);
		if (key == 27)
			break;
	}

	return 0;
}