#define _CRT_SECURE_NO_WARNINGS
#include "RS232c.h"
#include <thread>
#include <opencv2/opencv.hpp>
#include <conio.h>
#include <vector>
#include <cstring>
#include <Windows.h>
#include <cstdint>
#include <iostream>


//#define USE_EOSENS
//#define USE_XIMEA
//#define USE_IDPEXPRESS
#define USE_BASLER



#ifdef _DEBUG
#define CAMERA_EXT "d.lib"
#else
#define CAMERA_EXT ".lib"
#endif

#define STR_EXP(__A) #__A
#define STR(__A) STR_EXP(__A)
#define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" CV_VER CAMERA_EXT)

#include <HSC/baslerClass.hpp>
#pragma comment(lib, "BaslerLib" CAMERA_EXT) //BaslerLib.libを導入

#pragma warning(disable:4996)

using namespace std;

struct Capture {
	bool flg;
	basler cam;
	cv::Mat in_img;
};

//プロトタイプ宣言
void TakePicture(Capture* cap);

//画像保存先のディレクトリ
string save_dir = "D:\save_img";

int main(int argc, char* argv[]) {
	//カメラのセットアップ
	Capture cap;
	int width = 640;
	int height = 480;
	float fps = 750.0f;
	float gain = 1.0f;
	vector<cv::Mat> save_imgs;

	cap.cam.connect(0);
	//カメラ共通パラメータ設定
	cap.cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cap.cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);

	//Baslerのカメラパラメータ設定
	cap.cam.setParam(paramTypeBasler::Param::ExposureTime, 1280.0f);
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	cap.cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	cap.cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame); //常にバッファを更新
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);

	cap.cam.parameter_all_print();

	//変数定義
	cap.in_img = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255));
	bool flg = true;

	//DDmotorセットアップ
	RS232c ddmotor;
	char ddmotor_buff[256];

	//カメラ起動
	cap.cam.start();

	//DDmotor起動
	ddmotor.Connect("COM3", 38400, 8, NOPARITY, 0, 0, 0, 20000, 20000);

	//画像を更新し続けるスレッド
	thread thr(TakePicture, &cap);

	//DDmotorのサーボ起動
	//Task：ここにDDMotorのサーボがONになっているか確認するコード
	ddmotor.Send("$O\r");

	while (1)
	{
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
		else if (key == 'p') { save_imgs.push_back(cap.in_img.clone()); } //撮像し，データを格納
		else if (key == 'r') { ddmotor.Send("$I4500,10\r"); }//DDMotorを回転
	}
	flg = false;
	if (thr.joinable())thr.join();

	//カメラ切断
	cap.cam.stop();
	cap.cam.disconnect();

	//DDmotorのサーボOFF
	ddmotor.Send("$F\r");

	//取得した画像の一括保存
	for (size_t i = 0; i < save_imgs.size(); i++)
	{
		cv::imwrite(save_dir + "no" + to_string(i) + ".png", save_imgs[i]);
	}

	return 0;
}

//スレッド関数
void TakePicture(Capture* cap) {
	while (cap->flg)
	{
		cap->cam.captureFrame(cap->in_img.data);
	}
}