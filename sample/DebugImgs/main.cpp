#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <iostream>
#include <vector>
#include <cstdio>
#include <filesystem>

#pragma warning(disable:4996)
using namespace std;

int main() {
	string video_dir = "202010131041_video.mp4";
	cv::VideoCapture video;
	video.open(video_dir);
	if (!video.isOpened()) {
		cout << "video cannot ne opened..." << endl;
		return 0;
	}

	//char dir[] = "E:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/100/100";
	//namespace fs = std::filesystem;
	//if (!fs::create_directories(dir)) { return 0; }
	//double ref = atan2(-0.36, -0.09) + 3.159265 / 2;
	//vector<vector<double>> a;

	//a.push_back(b);
	//cout << a.size() << endl;

	cv::Point2f bias(10.0, 1000.2);

	while (video.grab())
	{
		cv::Mat  image, imageCopy, mask ,nonzero, imageCopy2, mask_lsm, rei;
		cv::Point2f out;
		
		vector<cv::Point> bps, bps_roi;
		vector<cv::Vec3f> circles;
		video.retrieve(image);
		image.copyTo(imageCopy);
		image.copyTo(imageCopy2);
		cv::Rect roi(960 - 430, 540 - 430 , 430*2, 430*2);
		cv::Rect roi_ref(938 - 34, 492 - 34, 34 * 2, 34 * 2);

		mask_lsm = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar::all(0));
		cv::circle(mask_lsm, cv::Point(960, 540), 430, cv::Scalar::all(255), -1);
		cv::circle(mask_lsm, cv::Point(938, 492), 34+20, cv::Scalar::all(0), -1);

		imageCopy2.copyTo(rei, mask_lsm);

		cv::cvtColor(imageCopy2, imageCopy2, CV_BGR2GRAY);
		cv::HoughCircles(imageCopy2, circles, CV_HOUGH_GRADIENT, 2, 100, 200, 100, 400, 530);
		cv::threshold(imageCopy, mask, 240, 255, cv::THRESH_BINARY);
		cv::threshold(rei, rei, 240, 255, cv::THRESH_BINARY);

		cv::cvtColor(mask, mask, CV_RGB2GRAY);
		cv::findNonZero(mask(roi), bps_roi);
		cv::findNonZero(mask, bps);
		

		out.x = bps[0].x + bias.x;
		out.y = bps[0].y - bias.y;

		for (size_t i = 0; i < bps_roi.size(); i++)
		{
			bps_roi[i] = bps_roi[i] + cv::Point(roi.x, roi.y);
		}

		cv::Moments mu = cv::moments(mask(roi_ref));
		cv::Point refp;
		refp.x = mu.m10 / mu.m00 + 938-34;
		refp.y = mu.m01 / mu.m00 + 492-34;

		//for (size_t i = 0; i < circles.size(); i++)
		//{
		//	cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		//	int radius = cvRound(circles[i][2]);
		//	// 円の中心を描画します．
		//	circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		//	// 円を描画します．
		//	circle(image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
		//}
		circle(image, cv::Point(960, 540), 430, cv::Scalar(0, 0, 255), 3, 8, 0);
		circle(image, refp, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

		//cv::imshow("out", image);
		//cv::imshow("masked", mask);
		cv::imshow("lsmout", rei);
		char key = (char)cv::waitKey(1);
		if (key == 27)
			break;
	}

	//char dir[] = "G:/マイドライブ/博士/淺間山下研/研究/Conference/SI2020/figures/mov/data/Pictures/frame";
	//char todir[] = "G:/マイドライブ/博士/淺間山下研/研究/Conference/SI2020/figures/mov/data/Pictures/selected/frame";
	//int cnt = 0;
	//for (int i = 1; i < 9397; i++)
	//{
	//	if (i % 30 == 0)
	//	{
	//		char filename[256];
	//		char tofilename[256];
	//		char a[256];
	//		sprintf(filename, "%s%05d.png", dir, i);
	//		sprintf(tofilename, "%s%05d.png", todir, i);
	//		sprintf(a, "%s%05d.png", todir, cnt);
	//		rename(tofilename, a);
	//		cnt++;
	//	}

	//}

	return 0;
}