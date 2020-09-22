#include <opencv2/opencv.hpp>
#include <HSC/KAYACoaXpressClass.hpp>
#include <vector>
#include <time.h>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <vector>
#include <Windows.h>
#include "RS232c.h"
#include <opencv2/core.hpp>


#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//グローバル変数
/// 画像に関する変数
cv::Mat in_img_now;
vector<cv::Mat> in_imgs;
int in_imgs_saveid = 0;
/// 時間に関する変数
int timeout = 10;
LARGE_INTEGER freq, start;
double logtime = 0;
/// ARマーカに関する変数
unsigned int marker_num = 1;
vector<cv::Mat> Rs;
vector< cv::Vec3d > Rvecs, Tvecs;
/// 排他制御用のMutex
cv::Mutex mutex;


//プロトタイプ宣言
void TakePicture(kayacoaxpress* cam, bool* flg);
void ShowLogs(bool* flg);
void DetectAR(bool* flg);

int main() {
	//パラメータ
	bool flg = true;
	LARGE_INTEGER end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// 単位習得

	//カメラパラメータ
	int width = 1920;
	int height = 1080;
	float fps = 1000.0;
	float exposuretime = 912.0;
	int offsetx = 0;
	int offsety = 0;

	//カメラクラスのインスタンスの生成
	kayacoaxpress cam;
	cam.connect(0);

	//パラメータの設定
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x1);
	cam.parameter_all_print();

	//レーザCalibrationの結果の呼び出し


	//取得画像を格納するVectorの作成
	cout << "Set Mat Vector..." << endl;
	for (size_t i = 0; i < (int)(timeout)*fps+10; i++)
	{
		in_imgs.push_back(cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	}

	//画像出力用Mat
	in_img_now = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(255));



	//カメラ起動
	cout << "Aroud 3D Sensing Start!" << endl;
	cam.start();

	//Threadの作成
	/// 1000fpsで画像を格納し続けるスレッド
	thread thr1(TakePicture, &cam, &flg);
	/// 現在の画像をPCに出力して見えるようするスレッド
	thread thr2(ShowLogs, &flg);
	/// ARマーカを検出＆位置姿勢を計算するスレッド
	//thread thr3(DetectAR, &flg);
	

	//計測開始
	if (!QueryPerformanceCounter(&start)) { return 0; }


	//メインループ
	/// 取得された画像から光切断法で三次元位置を計算する
	while (flg)
	{
		//光切断の高度の更新

		//時刻の更新
		if (!QueryPerformanceCounter(&end)) { return 0; }
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		if (logtime > timeout)
		{
			flg = false;
		}
	}

	//スレッドの停止
	if (thr1.joinable())thr1.join();
	if (thr2.joinable())thr2.join();
	//if (thr3.joinable())thr3.join();

	//計算した座標，取得画像の保存

	return 0;
}

void TakePicture(kayacoaxpress* cam, bool* flg) {
	cv::Mat temp = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar::all(255));
	while (*flg)
	{
		cam->captureFrame(temp.data);
		{
			//cv::AutoLock lock(mutex);
			in_imgs[in_imgs_saveid] = temp.clone();
			in_imgs_saveid++;
			in_img_now = temp.clone();
		}
		
	}
}

//現在の画像を30fps程度で出力する
void ShowLogs(bool* flg) {
	while (*flg)
	{
		if (in_imgs_saveid>3)
		{
			//cv::AutoLock lock(mutex);
			cv::imshow("img", in_imgs[in_imgs_saveid-3]);
		}		
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;
		printf("Time: %lf [s]", logtime);
	}
}

//ARマーカの検出と位置姿勢推定
void DetectAR(bool* flg) {
	//パラメータの設定
	cv::Mat R;
	for (size_t i = 0; i < marker_num; i++) Rs.push_back(R);
	//辞書の指定
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	//Perspectiveカメラの内部パラメータを取得
	string calib_dir = "PerspectiveCamCalibParams.xml";
	cv::FileStorage calibxml(calib_dir, cv::FileStorage::READ);
	if (!calibxml.isOpened()) cout << "calib xml cannot be opened..." << endl;
	cv::Mat K, D;
	calibxml["K"] >> K;
	calibxml["D"] >> D;
	while (*flg)
	{
		//マーカ検出
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		cv::aruco::detectMarkers(in_img_now, dictionary, corners, ids);
		//マーカ検出時，位置姿勢を計算する
		if (ids.size() > 0) {
			cv::aruco::estimatePoseSingleMarkers(corners, 0.2, K, D, Rvecs, Tvecs);
			for (size_t i = 0; i < ids.size(); i++)
			{
				cv::Rodrigues(Rvecs[i], Rs[i]);
			}
		}
	}
}