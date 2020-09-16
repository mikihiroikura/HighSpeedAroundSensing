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


#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//グローバル変数
cv::Mat in_img;

int main() {
	//パラメータ
	bool flg = true;

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
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x1);
	cam.parameter_all_print();

	//レーザCalibrationの結果の呼び出し


	//カメラ起動
	cam.start();

	//Threadの作成
	/// 1000fpsで画像を格納し続けるスレッド
	/// 現在の画像をPCに出力して見えるようするスレッド
	/// ARマーカを検出＆位置姿勢を計算するスレッド
	
	//メインループ
	/// 取得された画像から光切断法で三次元位置を計算する
	while (flg)
	{

	}

	//スレッドの停止

	//計算した座標，取得画像の保存

	return 0;
}

void TakePicture(kayacoaxpress* cam, bool* flg) {
	while (*flg)
	{
		cam->captureFrame(in_img.data);
	}
}

void ShowLogs() {

}