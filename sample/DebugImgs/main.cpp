#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <iostream>
#include <vector>
#include <cstdio>
#include <filesystem>
#include <Windows.h>
#include <omp.h>

#pragma warning(disable:4996)
using namespace std;

int main() {
	string video_dir = "202009302029_video.mp4";
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
		cv::Mat  image, imageCopy, mask ,nonzero, imageCopy2, mask_lsm, rei, mask_lsmcircle, rei2;
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
		cv::threshold(rei, rei2, 240, 255, cv::THRESH_BINARY);
		cv::bitwise_and(imageCopy, rei2, rei2);

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

		//20201111 同心円状に輝度重心を計算する
		//cout << (int)rei.data[424 * rei.step + 558 * rei.elemSize() + 1] << endl;
		int x, y;
		int step = rei.step;
		int elem = rei.elemSize();
		int rstart = 100;
		int forend;
		double deg = atan2(refp.y - 492, refp.x - 938);
		double dtheta = 30;
		double mass, moment_x, moment_y,rs;
		double cog[(430 - 60)*2][2] = { 0 };
		LARGE_INTEGER freq, start, end;
		double logtime = 0;
#ifdef _OPENMP
		cout << omp_get_max_threads() << endl;
#endif
		omp_set_num_threads(16);
		int a[10000];
		if (!QueryPerformanceFrequency(&freq)) { return 0; }
		if (!QueryPerformanceCounter(&start)) { return 0; }
		
		cv::Mat detectarea = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar::all(0));
		cv::Mat cogs = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar::all(0));
		if (!QueryPerformanceFrequency(&freq)) { return 0; }
		if (!QueryPerformanceCounter(&start)) { return 0; }
//#pragma omp parallel for
		for (int r = rstart*2; r < 430*2; r++)
		{
			mass = 0;
			moment_x = 0;
			moment_y = 0;
			rs = (double)r / 2;
			forend = (int)(3.15 * 2 * rs * dtheta / 360);
			for (size_t i = 0; i < forend; i++)
			{
				x = (int)(960 + (double)rs * cos((double)i / rs + deg - dtheta / 2 / 360 * 3.1415));
				y = (int)(540 + (double)rs * sin((double)i / rs + deg - dtheta / 2 / 360 * 3.1415));
				//detectarea.data[y * detectarea.cols + x] = 255;
				if ((int)rei2.data[y * step + x * elem + 1] > 0)
				{
					//cout << x << "," << y << " bps " << (int)rei2.data[y * rei2.step + x * rei2.elemSize() + 1] << endl;
					mass += rei2.data[y * step + x * elem + 1];
					moment_x += rei2.data[y * step + x * elem + 1] * x;
					moment_y += rei2.data[y * step + x * elem + 1] * y;
				}
			}
			if (mass > 0)
			{
				cog[(int)((r - rstart*2))][0] = moment_x / mass;
				cog[(int)((r - rstart * 2))][1] = moment_y / mass;
				//cogs.data[(int)(moment_y / mass) * rei.cols + (int)(moment_x / mass)] = 255;
			}
		}
//#pragma omp parallel for
		//for (int r = rstart; r < 430; r++)
		//{
		//	cout << omp_get_thread_num() << endl;
		//	mass = 0;
		//	moment_x = 0;
		//	moment_y = 0;
		//	forend = (int)(3.15 * 2 * (double)r * dtheta / 360);
		//	for (size_t i = 0; i < forend; i++)
		//	{
		//		x = (int)(960 + (double)r * cos((double)i / r + deg - dtheta / 2 / 360 * 3.1415));
		//		y = (int)(540 + (double)r * sin((double)i / r + deg - dtheta / 2 / 360 * 3.1415));
		//		detectarea.data[y * rei.cols + x] = 255;
		//		if ((int)rei2.data[y * step + x * elem + 1] > 0)
		//		{
		//			//cout << x << "," << y << " bps " << (int)rei.data[y * rei.step + x * rei.elemSize() + 1] << endl;
		//			mass += rei2.data[y * step + x * elem + 1];
		//			moment_x += rei2.data[y * step + x * elem + 1] * x;
		//			moment_y += rei2.data[y * step + x * elem + 1] * y;
		//		}
		//	}
		//	if (mass > 0)
		//	{
		//		cog[r - rstart][0] = moment_x / mass;
		//		cog[r - rstart][1] = moment_y / mass;
		//	}
		//}
		if (!QueryPerformanceCounter(&end)) { return 0; }
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		cout << logtime << endl;
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