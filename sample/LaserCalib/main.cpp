#include <opencv2/opencv.hpp>
#include <HSC/KAYACoaXpressClass.hpp>
#include <vector>
#include <time.h>
#include <thread>
#include <opencv2/ccalib/omnidir.hpp>


#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

int main() {
	//事前にCalibrationした内部パラメータの呼び出し
	string calib_dir = "OmnidirCamCalibParams.xml";
	cv::FileStorage calibxml(calib_dir, cv::FileStorage::READ);
	if (!calibxml.isOpened())
	{
		cout << "calib xml cannot be opened..." << endl;
		return 0;
	}
	cv::Mat K, Xi, D;
	calibxml["K"] >> K;
	calibxml["Xi"] >> Xi;
	calibxml["D"] >> D;


	//レーザCalibration用動画の読み取り
	string video_dir = "";
	cv::VideoCapture video;
	video.open(video_dir);
	if (!video.isOpened())
	{
		cout << "video cannot be opened..." << endl;
		return 0;
	}

	//画像保存用バッファ
	vector<cv::Mat> laser_imgs;
	cv::Mat laser_img;
	while (true)
	{
		video >> laser_img;
		if (laser_img.empty()) break;
		laser_imgs.push_back(laser_img);
	}


	return 0;
}