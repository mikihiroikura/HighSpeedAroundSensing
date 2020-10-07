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
#include "params.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <filesystem>


#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;
namespace fs = std::filesystem;

//DEFINE群
#define SAVE_LOGS_
//#define SAVE_IMGS_

//グローバル変数
/// 画像に関する変数
cv::Mat in_img_now;
vector<cv::Mat> in_imgs;
long long in_imgs_saveid = 0;
/// 時間に関する変数
int timeout = 10;
LARGE_INTEGER freq, start;
double logtime = 0;
/// ARマーカに関する変数
unsigned int marker_num = 1;
vector<cv::Mat> Rs;
vector< cv::Vec3d > Rvecs, Tvecs;
cv::Vec3d Tvec_id0;
double dir_arid0_rad;
vector<bool> detect_arid0_flgs;
long long detectarcnt = 0;
/// 排他制御用のMutex
cv::Mutex mutex;
/// DDMotor制御に関する変数
long long processarcnt = 1;
RS232c mbed;
char command[256] = "";
int rpm = 100;
char mode = 'R';
int initpulse = 100;
int movepulse = 50;
const int gearratio = 1000;
const int rotpulse = 432000 / gearratio;
#define READBUFFERSIZE 256
/// 光切断法計算用変数
double phi, lambda, u, v, w;
cv::Mat campt;
cv::Rect roi_lsm(960 - 430, 540 - 430, 430 * 2, 430 * 2);
cv::Rect roi_ref;
//デバッグ用変数
LARGE_INTEGER lsmstart, lsmend, takestart, takeend, arstart, arend;
double taketime = 0;
double lsmtime = 0;
double artime = 0;

//プロトタイプ宣言
void TakePicture(kayacoaxpress* cam, bool* flg, LSM* lsm);
void ShowLogs(bool* flg);
void DetectAR(bool* flg);
void SendDDMotorCommand(bool* flg);
int CalcLSM(LSM* lsm, Logs* logs);

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

	//光切断法に関する構造体のインスタンス
	LSM lsm;
	lsm.processcnt = 0;

	//ログ保存に関する構造体のインスタンス
	Logs logs;
	
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
	FILE* fcam, * flaser;
	fcam = fopen("202009301655_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++){ fscanf(fcam, "%lf,", &lsm.map_coefficient[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &lsm.stretch_mat[i]); }
	swap(lsm.stretch_mat[1], lsm.stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &lsm.distortion[i]); }
	fclose(fcam);
	flaser = fopen("202010051818_laserinterpolparam.csv", "r");
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pa[i]); }
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pb[i]); }
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pc[i]); }
	for (size_t i = 0; i < 2; i++) { fscanf(flaser, "%lf,", &lsm.ref_center[i]); }
	fscanf(flaser, "%lf,", &lsm.ref_radius);
	fscanf(flaser, "%lf,", &lsm.ref_arcwidth);
	fclose(flaser);
	lsm.det = 1 / (lsm.stretch_mat[0] - lsm.stretch_mat[1] * lsm.stretch_mat[2]);

	//MBEDのRS232接続
	mbed.Connect("COM4", 115200, 8, NOPARITY, 0, 0, 0, 5000, 20000);

	//画像出力用Mat
	in_img_now = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(255));

	//マスク画像の生成
	lsm.mask_refarc = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(0));
	cv::circle(lsm.mask_refarc, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius - lsm.ref_arcwidth / 2), cv::Scalar::all(255), (int)lsm.ref_arcwidth);
	roi_ref = cv::Rect((int)(lsm.ref_center[0] - lsm.ref_radius), (int)(lsm.ref_center[1] - lsm.ref_radius), (int)(2 * lsm.ref_radius), (int)(2 * lsm.ref_radius));
	lsm.mask_lsm = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(0));
	cv::circle(lsm.mask_lsm, cv::Point(960, 540), 430, cv::Scalar::all(255), -1);
	cv::circle(lsm.mask_lsm, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius), cv::Scalar::all(0), -1);

	//取得画像を格納するVectorの作成
	cout << "Set Mat Vector..." << endl;
	for (size_t i = 0; i < (int)(timeout)*fps+100; i++)
	{
		in_imgs.push_back(cv::Mat(height, width, CV_8UC1, cv::Scalar::all(0)));
		lsm.processflgs.push_back(false);
	}

	//カメラ起動
	cout << "Aroud 3D Sensing Start!" << endl;
	cam.start();

	//計測開始
	if (!QueryPerformanceCounter(&start)) { return 0; }

	//Threadの作成
	/// 1000fpsで画像を格納し続けるスレッド
	thread thr1(TakePicture, &cam, &flg, &lsm);
	/// 現在の画像をPCに出力して見えるようするスレッド
	thread thr2(ShowLogs, &flg);
	/// ARマーカを検出＆位置姿勢を計算するスレッド
	thread thr3(DetectAR, &flg);
	/// DDMotorにコマンドを送信するスレッド
	thread thr4(SendDDMotorCommand, & flg);
	

	//メインループ
	/// 取得された画像から光切断法で三次元位置を計算する
	while (flg)
	{
		//光切断の高度の更新
		if (in_imgs_saveid > 3)
		{
			if (lsm.processflgs[in_imgs_saveid - 1])
			{
				CalcLSM(&lsm, &logs);
				if (!QueryPerformanceCounter(&end)) { return 0; }
				logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
				logs.LSM_times.push_back(logtime);
			}
		}
		
		
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
	if (thr3.joinable())thr3.join();
	if (thr4.joinable())thr4.join();

	//カメラの停止，RS232Cの切断
	cam.stop();
	cam.disconnect();

	//計算した座標，取得画像の保存
	/// Logファイルの作成
#ifdef SAVE_LOGS_
	FILE* fr;
	time_t timer;
	struct tm now;
	timer = time(NULL);
	localtime_s(&now, &timer);
	char dir[256];
	strftime(dir, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S", &now);
	if (!fs::create_directories(dir)) { return 0; }
	char logfile[256];
	strftime(logfile, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S/LSM_result.csv", &now);
	fr = fopen(logfile, "w");

	/// Logの保存
	cout << "Saving logs..." << endl;
	for (size_t i = 0; i < logs.LSM_times.size(); i++)
	{
		if (logs.LSM_pts[i].size() > 10000) { continue; }//バグ取り
		fprintf(fr, "%lf,", logs.LSM_times[i]);
		fprintf(fr, "\n");

		for (size_t j = 0; j < logs.LSM_pts[i].size(); j++) { fprintf(fr, "%lf,", logs.LSM_pts[i][j].at<double>(0, 0)); }
		if (logs.LSM_pts.size() == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < logs.LSM_pts[i].size(); j++) { fprintf(fr, "%lf,", logs.LSM_pts[i][j].at<double>(0, 1)); }
		if (logs.LSM_pts.size() == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < logs.LSM_pts[i].size(); j++) { fprintf(fr, "%lf,", logs.LSM_pts[i][j].at<double>(0, 2)); }
		if (logs.LSM_pts.size() == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");
	}
	cout << "Logs finish!" << endl;
	fclose(fr);

	/// 画像を保存
#ifdef SAVE_IMGS_
	cout << "Saving imgs..." << endl;
	char picturename[256];
	char picsubname[256];
	strftime(picsubname, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S//Pictures/frame", &now);
	for (int i = 0; i < in_imgs_saveid; i++)
	{
		sprintf(picturename, "%s%05d.png", picsubname, i);//png可逆圧縮
		cv::imwrite(picturename, in_imgs[i]);
	}
	cout << "Imgs finished!" << endl;
#endif // SAVE_IMGS_

#endif // SAVE_LOGS_


	return 0;
}

//画像を格納する
void TakePicture(kayacoaxpress* cam, bool* flg, LSM *lsm) {
	cv::Mat temp = cv::Mat(1080, 1920, CV_8UC1, cv::Scalar::all(0));
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
		cam->captureFrame(temp.data);
		{
			//cv::AutoLock lock(mutex);
			in_imgs[in_imgs_saveid] = temp.clone();
			//in_img_now = temp.clone();
		}
		lsm->processflgs[in_imgs_saveid] = true;
		in_imgs_saveid++;
		QueryPerformanceCounter(&takeend);
		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		while (taketime<0.001)
		{
			QueryPerformanceCounter(&takeend);
			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		}
		
		//cout << taketime << endl;
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
		//printf("Time: %lf [s]", logtime);
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
		//QueryPerformanceCounter(&arstart);
		//マーカ検出
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		if (in_imgs_saveid>2){ cv::aruco::detectMarkers(in_imgs[in_imgs_saveid - 2], dictionary, corners, ids); }
		//マーカ検出時，位置姿勢を計算する
		if (ids.size() > 0) {
			cv::aruco::estimatePoseSingleMarkers(corners, 0.2, K, D, Rvecs, Tvecs);
			for (size_t i = 0; i < ids.size(); i++)
			{
				//ID=0のマーカ検出されると方向ベクトル保存
				if (ids[i] == 0) {
					Tvec_id0 = Tvecs[i];
					detect_arid0_flgs.push_back(true);
				}
			}
		}
		//マーカ検出されなかったとき
		else
		{
			detect_arid0_flgs.push_back(false);
			cout << "FALSE" << endl;
		}
		//QueryPerformanceCounter(&arend);
		//artime = (double)(arend.QuadPart - arstart.QuadPart) / freq.QuadPart;
		//cout << "AR time:" << artime << endl;
	}
}

//DDMotorへのコマンド送信
void SendDDMotorCommand(bool* flg) {
	//動作開始のコマンド
	snprintf(command, READBUFFERSIZE, "%c,%d,%d,%d,\r", mode, rpm, initpulse, movepulse);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);
	while (*flg)
	{
		if (detect_arid0_flgs.size()>processarcnt)
		{
			//ARマーカ検出したとき，局所計測
			if (detect_arid0_flgs[processarcnt-1])
			{
				//ARマーカの方向計算
				dir_arid0_rad = atan2(Tvec_id0[1], Tvec_id0[0]) + M_PI / 2;
				initpulse = ((int)(dir_arid0_rad * 180 / M_PI / 360 * rotpulse) + rotpulse - movepulse / 2) % (rotpulse);
				if (initpulse < 0) initpulse += rotpulse;
				//コマンド送信
				mode = 'L';
				/// rpm, movepulseの変更
			}
			else// if(!detect_arid0_flgs[processarcnt]&& !detect_arid0_flgs[processarcnt-1])
			{//ARマーカが2回連続ないとき，全周計測
				mode = 'R';
			}
			snprintf(command, READBUFFERSIZE, "%c,%d,%d,%d,\r", mode, rpm, initpulse, movepulse);
			mbed.Send(command);
			memset(command, '\0', READBUFFERSIZE);
			processarcnt++;
		}
	}
	//終了コマンド送信
	mode = 'F';
	snprintf(command, READBUFFERSIZE, "%c,%d,%d,%d,\r", mode, rpm, initpulse, movepulse);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);
}

//Mainループでの光切断法による形状計測
int CalcLSM(LSM *lsm, Logs *logs) {
	//QueryPerformanceCounter(&lsmstart);
	//変数のリセット
	lsm->bps.clear();
	lsm->idpixs.clear();
	lsm->campts.clear();

	//画像の格納
	{
		//cv::AutoLock lock(mutex);
		lsm->in_img = in_imgs[in_imgs_saveid-1].clone();
	}
	lsm->processcnt++;
	if (lsm->in_img.data!=NULL)
	{
		//参照面の輝度重心の検出
		lsm->in_img.copyTo(lsm->ref_arc, lsm->mask_refarc);
		cv::threshold(lsm->ref_arc, lsm->ref_arc, 240.0, 255.0, cv::THRESH_BINARY);
		cv::Moments mu = cv::moments(lsm->ref_arc(roi_ref));
		lsm->rp[0] = mu.m10 / mu.m00 + roi_ref.x;
		lsm->rp[1] = mu.m01 / mu.m00 + roi_ref.y;
		//ラインレーザの輝点座標を検出
		lsm->in_img.copyTo(lsm->lsm_laser, lsm->mask_lsm);
		cv::threshold(lsm->lsm_laser, lsm->lsm_laser, 240, 255, cv::THRESH_BINARY);
		cv::findNonZero(lsm->lsm_laser(roi_lsm), lsm->bps);
		for (size_t i = 0; i < lsm->bps.size(); i++) { lsm->bps[i] += cv::Point(roi_lsm.x, roi_lsm.y); }
		
		//レーザ平面の法線ベクトルの計算
		lsm->plane_nml[0] = lsm->pa[0] + lsm->pa[1] * lsm->rp[0] + lsm->pa[2] * lsm->rp[1] + lsm->pa[3] * pow(lsm->rp[0], 2)
			+ lsm->pa[4] * lsm->rp[0] * lsm->rp[1] + lsm->pa[5] * pow(lsm->rp[1], 2) + lsm->pa[6] * pow(lsm->rp[0], 3)
			+ lsm->pa[7] * pow(lsm->rp[0], 2) * lsm->rp[1] + lsm->pa[8] * lsm->rp[0] * pow(lsm->rp[1], 2)
			+ lsm->pa[9] * pow(lsm->rp[1], 3);
		lsm->plane_nml[1] = lsm->pb[0] + lsm->pb[1] * lsm->rp[0] + lsm->pb[2] * lsm->rp[1] + lsm->pb[3] * pow(lsm->rp[0], 2)
			+ lsm->pb[4] * lsm->rp[0] * lsm->rp[1] + lsm->pb[5] * pow(lsm->rp[1], 2) + lsm->pb[6] * pow(lsm->rp[0], 3)
			+ lsm->pb[7] * pow(lsm->rp[0], 2) * lsm->rp[1] + lsm->pb[8] * lsm->rp[0] * pow(lsm->rp[1], 2)
			+ lsm->pb[9] * pow(lsm->rp[1], 3);
		lsm->plane_nml[2] = lsm->pc[0] + lsm->pc[1] * lsm->rp[0] + lsm->pc[2] * lsm->rp[1] + lsm->pc[3] * pow(lsm->rp[0], 2)
			+ lsm->pc[4] * lsm->rp[0] * lsm->rp[1] + lsm->pc[5] * pow(lsm->rp[1], 2) + lsm->pc[6] * pow(lsm->rp[0], 3)
			+ lsm->pc[7] * pow(lsm->rp[0], 2) * lsm->rp[1] + lsm->pc[8] * lsm->rp[0] * pow(lsm->rp[1], 2)
			+ lsm->pc[9] * pow(lsm->rp[1], 3);

		//理想ピクセル座標系に変換
		for (size_t i = 0; i < lsm->bps.size(); i++)
		{
			cv::Point2f idpix;
			idpix.x = lsm->det * ((lsm->bps[i].x - lsm->distortion[0]) - lsm->stretch_mat[1] * (lsm->bps[i].y- lsm->distortion[1]));
			idpix.y = lsm->det * (-lsm->stretch_mat[2] * (lsm->bps[i].x - lsm->distortion[0]) + lsm->stretch_mat[0] * (lsm->bps[i].y - lsm->distortion[1]));
			lsm->idpixs.push_back(idpix);
		}

		//理想ピクセル座標->直線の式とレーザ平面から輝点三次元座標の計算
		for (size_t i = 0; i < lsm->idpixs.size(); i++)
		{
			u = lsm->idpixs[i].x;
			v = lsm->idpixs[i].y;
			phi = hypot(u, v);
			w = lsm->map_coefficient[0] + lsm->map_coefficient[1] * pow(phi, 2) +
				lsm->map_coefficient[2] * pow(phi, 3) + lsm->map_coefficient[3] * pow(phi, 4);
			lambda = 1 / (lsm->plane_nml[0] * u + lsm->plane_nml[1] * v + lsm->plane_nml[2] * w);
			campt = (cv::Mat_<double>(1, 3) << lambda * u, lambda * v, lambda * w);
			lsm->campts.push_back(campt);
		}
	}
	//QueryPerformanceCounter(&lsmend);
	//lsmtime = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
	//cout << lsmtime << endl;
	logs->LSM_pts.push_back(lsm->campts);
	return 0;
}