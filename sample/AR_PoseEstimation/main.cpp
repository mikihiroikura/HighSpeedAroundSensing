#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, const char* argv[])
{
	//内部パラメータcalibration用動画の読み取り
	string video_dir = "202010071825_video.mp4";
	cv::VideoCapture video;
	video.open(video_dir);
	if (!video.isOpened()) {
		cout << "video cannot ne opened..." << endl;
		return 0;
	}

	//ARマーカの種類の設定
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	//カメラパラメータの呼び出し
	string calib_dir = "PerspectiveCamCalibParams.xml";
	cv::FileStorage calibxml(calib_dir, cv::FileStorage::READ);
	if (!calibxml.isOpened())
	{
		cout << "calib xml cannot be opened..." << endl;
		return 0;
	}
	cv::Mat K, D;
	calibxml["K"] >> K;
	calibxml["D"] >> D;


	//動画からARマーカ検出&位置姿勢推定
	while (video.grab())
	{
		cv::Mat  image, imageCopy;
		video.retrieve(image);
		image.copyTo(imageCopy);

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		// if at least one marker detected
		if (ids.size() > 0) {
			cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
			vector< cv::Vec3d > Rvec, Tvec;
			cv::aruco::estimatePoseSingleMarkers(corners, 0.2, K, D, Rvec, Tvec);
			for (size_t i = 0; i < ids.size(); i++)
			{
				cv::aruco::drawAxis(imageCopy, K, D, Rvec[i], Tvec[i], 0.1);
				cv::Mat R;
				cv::Rodrigues(Rvec[i], R);
				cout << "Rot: " << R << endl;
				cout << "Tra: " << Tvec[i] << endl;
			}
		}
		cv::imshow("out", imageCopy);
		char key = (char)cv::waitKey(1);
		if (key == 27)
			break;
	}


    return 0;
}