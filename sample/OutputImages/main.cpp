#define _CRT_SECURE_NO_WARNINGS
#include <thread>
#include <opencv2/opencv.hpp>
#include <conio.h>


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
void TakePicture(Capture* cap, bool* flg);

int main() {
	//カメラのセットアップ
	Capture cap;
	int width = 640;
	int height = 480;
	float fps = 750.0f;
	float gain = 1.0f;


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

	//カメラ起動
	cap.cam.start();

	//画像を更新し続けるスレッド
	thread thr(TakePicture, &cap, &flg);

	while (1)
	{
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
	}
	flg = false;
	if (thr.joinable())thr.join();

	cap.cam.stop();
	cap.cam.disconnect();

	return 0;
}

//スレッド関数
void TakePicture(Capture *cap, bool* flg) {
	while (*flg)
	{
		cap->cam.captureFrame(cap->in_img.data);
	}
}