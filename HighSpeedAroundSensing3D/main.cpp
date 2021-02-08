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
#include "graphics.h"


#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif

//グローバル変数
/// カメラパラメータ
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 480;
const int offsety = 92;
/// 画像に関する変数
vector<cv::Mat> in_imgs;
int in_imgs_saveid = 0;
cv::Mat full, zero;
int takepicid = 0;
/// 時間に関する変数
LARGE_INTEGER freq, start;
double logtime = 0;
/// ARマーカに関する変数
unsigned int marker_num = 1;
vector<cv::Mat> Rs;
vector< cv::Vec3d > Rvecs, Tvecs;
cv::Vec3d Tvec_id0;
double dir_arid0_rad;
vector<bool> detect_arid0_flgs;
/// 排他制御用のMutex
cv::Mutex mutex;
/// DDMotor制御に関する変数
long long processarcnt = 1;
RS232c mbed;
char command[256] = "";
char rotdir = 'R';
const int gearratio = 1000;
const int rotpulse = 432000 / gearratio;
#define READBUFFERSIZE 256
long long detectfailcnt = 0;
const int rotaterpm = 500, reciprorpm = 200;
int rpm = rotaterpm;
const double Dc = danger_area * 1000, Ac = safe_area * 1000; //局所領域計測範囲切り替えの距離
const int Nc = 20; //一つのラインレーザからAc以下の距離の点群の最小個数
const int Cc = 5; //Dc以下の距離の点群の個数Nc以上の最小連続回数
const int dangerthr = 10;//危険領域の判定点数の閾値
int alertcnt = 0, dangercnt = 0, objcnt = 0, nonobjcnt = 0, totaldanger = 0;
int reciprocntdown = 3;
bool objdetected = false;//物体検出判定
bool detectenableflg = true;//物体検出可能フラグ
int detectenablecnt = 0;
int detectunablecnt = 30;
int contnonobjcnt = 100;//この回数分だけ連続で未検出判定ならば，全周囲計測に戻す
bool rotmode = false;//回転モード True:往復運動　False：全周囲
/// 光切断法計算用変数
int lsm_success = 1;
double phi, lambda, u, v, w;
cv::Rect roi_lsm(960 - 430, 540 - 430, 430 * 2, 430 * 2);
cv::Rect roi_ref;
cv::Rect roi_laser;
vector<double> rps;
const float mono_thr = 240.0;
const cv::Scalar color_thr_min(0, 0, 150);
const cv::Scalar color_thr_max(256, 256, 256);
vector<cv::Point> refpts;
const int colorstep = width * 3, colorelem = 3;
const int monostep = width, monoelem = 1;
double refmass, refmomx, refmomy;
int refx, refy;
unsigned int forend, cogx, cogy;
double lsmmass, lsmmomx, lsmmomy;
double dtheta = 30;
double deg;
vector<double> calcpt(3, 0);
cv::Point2f idpix;
unsigned int r_calc;
double lsmmass_r[rends - rstart] = { 0 }, lsmmomx_r[rends - rstart] = { 0 }, lsmmomy_r[rends - rstart] = { 0 };
int roi_laser_minx = width, roi_laser_maxx = 0, roi_laser_miny = height, roi_laser_maxy = 0;
const int roi_laser_margin = 30;
cv::Point laser_pts(0, 0);
int roi_laser_outcnt = 0;
long long lsmcalcid = 0;
int showglid = 0;
/// 単軸ロボットに関する変数
RS232c axisrobot;
#define READBUFFERSIZE 256
char replybuf[READBUFFERSIZE];
char axisrobmodes[][10] = { "@SRVO", "@START", "@ORG" };
char axisrobcommand[READBUFFERSIZE] = "";
const int initaxisstart = 0, initaxisend = 600;
const int posunits = 100, speedunits = 10;

/// ログに関する変数
const int cyclebuffersize = 10;
const int timeout = 30;
const int log_img_fps = 40;
const int log_img_finish_cnt = log_img_fps * timeout + 100;
const int log_LSM_finish_cnt = fps * timeout + 100;
long long log_img_cnt = 0, log_lsm_cnt = 0;
//デバッグ用変数
LARGE_INTEGER lsmstart, lsmend, takestart, takeend, arstart, arend, showstart, showend, mbedstart, mbedstop;
double taketime = 0, lsmtime = 0, artime = 0, showtime = 0, mbedtime = 0;
double lsmtime_a, lsmtime_b, lsmtime_c, lsmtime_d;

#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;
namespace fs = std::filesystem;

//DEFINE群
#define SAVE_LOGS_
//#define SAVE_IMGS_
#define OUT_COLOR_
//#define OUT_MONO_
#define AUTONOMOUS_SENSING_
//#define SHOW_PROCESSING_TIME_
#define SHOW_IMGS_OPENGL_
#define MOVE_AXISROBOT_

//プロトタイプ宣言
void TakePicture(kayacoaxpress* cam, bool* flg, LSM* lsm);
void ShowLogs(bool* flg);
void ShowAllLogs(bool* flg, double* pts, int* lsmshowid, cv::Mat* imglog);
void DetectAR(bool* flg);
void SendDDMotorCommand(bool* flg);
int CalcLSM(LSM* lsm, Logs* logs, long long* logid);
void Read_Reply_toEND(RS232c* robot);
void wait_QueryPerformance(double finishtime, LARGE_INTEGER freq);
void ControlAxisRobot(RS232c* robot, bool* flg);

int main() {
	//パラメータ
	bool flg = true;
	LARGE_INTEGER end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// 単位習得

	//光切断法に関する構造体のインスタンス
	LSM lsm;
	lsm.processcnt = 0;
	lsm.buffersize = cyclebuffersize;

	//ログ保存に関する構造体のインスタンス
	Logs logs;
	logs.LSM_pts_cycle = (double*)malloc(sizeof(double) * cyclebuffersize * (rends - rstart) * 3);
	
	//カメラクラスのインスタンスの生成
	kayacoaxpress cam;
	cam.connect(0);

	//パラメータの設定
	std::cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetX, offsetx);
	cam.setParam(paramTypeKAYACoaXpress::paramInt::OffsetY, offsety);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x2);
	cam.setParam(paramTypeKAYACoaXpress::CaptureType::BayerGRGrab);
	cam.parameter_all_print();

	//レーザCalibrationの結果の呼び出し
	FILE* fcam, * flaser;
	fcam = fopen("202101070034_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++){ fscanf(fcam, "%lf,", &lsm.map_coefficient[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &lsm.stretch_mat[i]); }
	swap(lsm.stretch_mat[1], lsm.stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &lsm.distortion[i]); }
	fclose(fcam);
	flaser = fopen("202101070034_laserinterpolparam.csv", "r");
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pa[i]); }
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pb[i]); }
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pc[i]); }
	for (size_t i = 0; i < 2; i++) { fscanf(flaser, "%lf,", &lsm.ref_center[i]); }
	fscanf(flaser, "%lf,", &lsm.ref_radius);
	fscanf(flaser, "%lf,", &lsm.ref_arcwidth);
	fclose(flaser);
	lsm.det = 1 / (lsm.stretch_mat[0] - lsm.stretch_mat[1] * lsm.stretch_mat[2]);

	//カラーORモノクロ
#ifdef OUT_MONO_
	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(0));
#endif // OUT_MONO_
#ifdef OUT_COLOR_
	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
#endif // OUT_COLOR_
	lsm.in_img = zero.clone();

	//マスク画像の生成
	lsm.mask_refarc = zero.clone();
	cv::circle(lsm.mask_refarc, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius - lsm.ref_arcwidth / 2), cv::Scalar::all(255), (int)lsm.ref_arcwidth);
	roi_ref = cv::Rect((int)(lsm.ref_center[0] - lsm.ref_radius), (int)(lsm.ref_center[1] - lsm.ref_radius), (int)(2 * lsm.ref_radius), (int)(2 * lsm.ref_radius));
	lsm.mask_lsm = zero.clone();
	cv::circle(lsm.mask_lsm, cv::Point((int)width/2, (int)height/2), 430, cv::Scalar::all(255), -1);
	cv::circle(lsm.mask_lsm, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius)+20, cv::Scalar::all(0), -1);
	roi_laser = cv::Rect(0, 0, width, height);

#ifdef SAVE_IMGS_
	//取得画像を格納するVectorの作成
	std::cout << "Set Img Vector for logs....................";
	for (size_t i = 0; i < log_img_finish_cnt; i++){logs.in_imgs_log.push_back(zero.clone());}
	logs.in_imgs_log_ptr = logs.in_imgs_log.data();
	cout << "OK!" << endl;
#endif // SAVE_IMGS_
#ifdef SAVE_LOGS_
	std::cout << "Set mode Vector and time Vector for logs...";
	logs.LSM_times = (double*)malloc(sizeof(double) * log_LSM_finish_cnt);
	logs.LSM_rotdir = (char*)malloc(sizeof(char) * log_LSM_finish_cnt);
	logs.LSM_pts_logs = (double*)malloc(sizeof(double) * log_LSM_finish_cnt * (rends-rstart) * 3);
	memset(logs.LSM_pts_logs, 0, sizeof(double) * log_LSM_finish_cnt * (rends - rstart) * 3);
	logs.LSM_detectedenableflg = (bool*)malloc(sizeof(bool) * log_LSM_finish_cnt);
	logs.LSM_objdetectedflg = (bool*)malloc(sizeof(bool) * log_LSM_finish_cnt);
	logs.LSM_reciprocntdown = (int*)malloc(sizeof(int) * log_LSM_finish_cnt);
	logs.LSM_alertcnt = (int*)malloc(sizeof(int) * log_LSM_finish_cnt);
	logs.LSM_dangercnt = (int*)malloc(sizeof(int) * log_LSM_finish_cnt);
	logs.LSM_rpm = (int*)malloc(sizeof(int) * log_LSM_finish_cnt);
	logs.LSM_laserplane_nml = (double*)malloc(sizeof(double) * log_LSM_finish_cnt * 3);
	logs.LSM_rps = (float*)malloc(sizeof(float) * log_LSM_finish_cnt * 2);
	logs.LSM_rotmodes = (bool*)malloc(sizeof(bool) * log_LSM_finish_cnt);
	cout << "OK!" << endl;
#endif // SAVE_LOGS_
	std::cout << "Set Mat Cycle Buffer.......................";
	for (size_t i = 0; i < cyclebuffersize; i++)
	{
		in_imgs.push_back(zero.clone());
		lsm.processflgs.push_back(false);
	}
	cout << "OK!" << endl;

	//単軸ロボットとの通信確保
#ifdef MOVE_AXISROBOT_
	std::cout << "Set commection to AXIS ROBOT...............";
	if (!axisrobot.Connect("COM6", 38400, 8, ODDPARITY, 0, 0, 0, 20000, 20000)) {
		cout << "No connect" << endl;
		return 1;
	}
	std::cout << "OK!" << endl;
	
	snprintf(axisrobcommand, READBUFFERSIZE, "%s%d.1\r\n", axisrobmodes[0], 1);
	axisrobot.Send(axisrobcommand);
	memset(axisrobcommand, '\0', READBUFFERSIZE);
	Read_Reply_toEND(&axisrobot);
	std::cout << "SERVO ON" << endl;
	wait_QueryPerformance(0.1, freq);
	snprintf(axisrobcommand, READBUFFERSIZE, "%s.1\r\n", axisrobmodes[2]);
	axisrobot.Send(axisrobcommand);
	cout << "ORG START" << endl;
	memset(axisrobcommand, '\0', READBUFFERSIZE);
	Read_Reply_toEND(&axisrobot);
	std::cout << "ORG STOP" << endl;
#endif // MOVE_AXISROBOT_

	//MBEDのRS232接続
	mbed.Connect("COM4", 115200, 8, NOPARITY, 0, 0, 0, 5000, 20000);
	//動作開始のコマンド
	snprintf(command, READBUFFERSIZE, "%c,%d,\r", rotdir, rpm);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);

	//カメラ起動
	std::cout << "Aroud 3D Sensing Start!" << endl;
	cam.start();

	//Threadの作成
	/// 1000fpsで画像を格納し続けるスレッド
	thread thr1(TakePicture, &cam, &flg, &lsm);
	/// 現在の画像をPCに出力して見えるようするスレッド
#ifdef SHOW_IMGS_OPENGL_
	thread thr2(ShowAllLogs, &flg, logs.LSM_pts_cycle, &showglid, logs.in_imgs_log_ptr);
#endif // SHOW_IMGS_OPENGL_
#ifdef MOVE_AXISROBOT_
	thread thr3(ControlAxisRobot, &axisrobot, &flg);
#endif // MOVE_AXISROBOT_


	//メインループ
	/// 取得された画像から光切断法で三次元位置を計算する
	//計測開始
	if (!QueryPerformanceCounter(&start)) { return 0; }
	while (flg)
	{
		//画像取得
		//takepicid = in_imgs_saveid % cyclebuffersize;
		//QueryPerformanceCounter(&takestart);
		//cam.captureFrame(in_imgs[takepicid].data);
		////memcpy(in_imgs[takepicid].data, temp.data, height * width * 3);
		//lsm.processflgs[takepicid] = true;
		//in_imgs_saveid = (in_imgs_saveid + 1) % cyclebuffersize;
		//QueryPerformanceCounter(&takeend);
		//taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		//std::cout << "TakePicture() time: " << taketime << endl;

		//光切断の高度の更新
		QueryPerformanceCounter(&lsmstart);
		lsm_success = CalcLSM(&lsm, &logs, &log_lsm_cnt);
		QueryPerformanceCounter(&lsmend);
		lsmtime = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
		while (lsmtime < 0.001)
		{
			QueryPerformanceCounter(&lsmend);
			lsmtime = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
		}
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "CalcLSM() time: " << lsmtime << endl;
#endif // SHOW_PROCESSING_TIME_
		

		//OpenGLでの点群表示
		/*QueryPerformanceCounter(&start);
		showglid = (in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize;
		drawGL_one(logs.LSM_pts_cycle, &showglid);
		QueryPerformanceCounter(&end);
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		std::cout << "drawGL() time: " << logtime << endl;*/

		//画像表示
		/*QueryPerformanceCounter(&start);
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
		int key = cv::waitKey(1);
		if (key == 'q') flg = false;
		QueryPerformanceCounter(&end);
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		std::cout << "Showimg() time: " << logtime << endl;*/

#ifdef SAVE_LOGS_
		QueryPerformanceCounter(&end);
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		if (lsm_success == 0)
		{
			*(logs.LSM_times + log_lsm_cnt) = logtime;
			*(logs.LSM_rotdir + log_lsm_cnt) = rotdir;
			*(logs.LSM_detectedenableflg + log_lsm_cnt) = detectenableflg;
			*(logs.LSM_objdetectedflg + log_lsm_cnt) = objdetected;
			*(logs.LSM_reciprocntdown + log_lsm_cnt) = reciprocntdown;
			*(logs.LSM_alertcnt + log_lsm_cnt) = alertcnt;
			*(logs.LSM_dangercnt + log_lsm_cnt) = dangercnt;
			*(logs.LSM_rpm + log_lsm_cnt) = rpm;
			memcpy(logs.LSM_laserplane_nml + log_lsm_cnt * 3, &lsm.plane_nml, sizeof(double) * 3);
			memcpy(logs.LSM_rps + log_lsm_cnt * 2, &lsm.rp, sizeof(float) * 2);
			*(logs.LSM_rotmodes + log_lsm_cnt) = rotmode;
			log_lsm_cnt++;
		}
		if (log_lsm_cnt > log_LSM_finish_cnt) flg = false;
		if (logtime > timeout) { flg = false; }
#endif // SAVE_LOGS_
	}

	//スレッドの停止
	if (thr1.joinable())thr1.join();
#ifdef SHOW_IMGS_OPENGL_
	if (thr2.joinable())thr2.join();
#endif // SHOW_IMGS_OPENGL_
#ifdef MOVE_AXISROBOT_
	if (thr3.joinable())thr3.join();
#endif // MOVE_AXISROBOT_

	//カメラの停止，RS232Cの切断
	cam.stop();
	cam.disconnect();

	//単軸ロボットの停止
#ifdef MOVE_AXISROBOT_
	snprintf(axisrobcommand, READBUFFERSIZE, "%s%d.1\r\n", axisrobmodes[0], 0);
	axisrobot.Send(axisrobcommand);
	Read_Reply_toEND(&axisrobot);
	memset(axisrobcommand, '\0', READBUFFERSIZE);
	cout << "SERVO OFF" << endl;
#endif // MOVE_AXISROBOT_


	//終了コマンド送信
	rotdir = 'F';
	snprintf(command, READBUFFERSIZE, "%c,%d,\r", rotdir, rpm);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);

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
	strftime(logfile, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S/%y%m%d%H%M%S_LSM_result.csv", &now);
	fr = fopen(logfile, "w");

	/// Logの保存
	std::cout << "Saving logs..." << endl;
	for (size_t i = 0; i < log_lsm_cnt; i++)
	{
		fprintf(fr, "%lf,", logs.LSM_times[i]);
		if (logs.LSM_rotdir[i]=='L') { fprintf(fr, "%lf,", 1.0); }
		else if (logs.LSM_rotdir[i] == 'R') { fprintf(fr, "%lf,", 0.0); }
		if (logs.LSM_detectedenableflg[i]) { fprintf(fr, "%lf,", 1.0); }
		else { fprintf(fr, "%lf,", 0.0); }
		if (logs.LSM_objdetectedflg[i]) { fprintf(fr, "%lf,", 1.0); }
		else { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "%d,", logs.LSM_reciprocntdown[i]);
		fprintf(fr, "%d,", logs.LSM_alertcnt[i]);
		fprintf(fr, "%d,", logs.LSM_dangercnt[i]);
		fprintf(fr, "%d,", logs.LSM_rpm[i]);
		for (size_t j = 0; j < 3; j++)
		{
			fprintf(fr, "%lf,", *(logs.LSM_laserplane_nml + i * 3 + j));
		}
		for (size_t j = 0; j < 2; j++)
		{
			fprintf(fr, "%f,", *(logs.LSM_rps + i * 2 + j));
		}
		if (logs.LSM_rotmodes[i]) { fprintf(fr, "%lf,", 1.0); }
		else { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < rends - rstart; j++) { 
			if (*(logs.LSM_pts_logs + i * (rends - rstart) * 3 + j * 3 + 0) != 0) { fprintf(fr, "%lf,", *(logs.LSM_pts_logs + i * (rends - rstart) * 3 + j * 3 + 0)); }
		}
		if (*(logs.LSM_pts_logs + i * (rends - rstart) * 3 + 0) == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < rends - rstart; j++) {
			if (*(logs.LSM_pts_logs + i * (rends - rstart) * 3 + j * 3 + 1) != 0) { fprintf(fr, "%lf,", *(logs.LSM_pts_logs + i * (rends - rstart) * 3 + j * 3 + 1)); }
		}
		if (*(logs.LSM_pts_logs + i * (rends - rstart) * 3 + 1) == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < rends - rstart; j++) {
			if (*(logs.LSM_pts_logs + i * (rends - rstart) * 3 + j * 3 + 2) != 0) { fprintf(fr, "%lf,", *(logs.LSM_pts_logs + i * (rends - rstart) * 3 + j * 3 + 2)); }
		}
		if (*(logs.LSM_pts_logs + i * (rends - rstart) * 3 + 2) == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");
	}
	std::cout << "Logs finish!" << endl;
	fclose(fr);
#endif // SAVE_LOGS_

	/// 画像を保存
#ifdef SAVE_IMGS_
	std::cout << "Saving imgs..." << endl;
	char picdir[256];
	strftime(picdir, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S/Pictures", &now);
	if (!fs::create_directories(picdir)) { return 0; }
	char picturename[256];
	char picsubname[256];
	strftime(picsubname, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S/Pictures/frame", &now);
	for (int i = 0; i < log_img_cnt; i++)
	{
		sprintf(picturename, "%s%05d.png", picsubname, i);//png可逆圧縮
		cv::imwrite(picturename, logs.in_imgs_log[i]);
	}
	std::cout << "Imgs finished!" << endl;
#endif // SAVE_IMGS_

	//動的メモリの開放
	free(logs.LSM_pts_cycle);
#ifdef SAVE_LOGS_
	free(logs.LSM_rotdir);
	free(logs.LSM_times);
	free(logs.LSM_pts_logs);
	free(logs.LSM_detectedenableflg);
	free(logs.LSM_objdetectedflg);
	free(logs.LSM_reciprocntdown);
	free(logs.LSM_alertcnt);
	free(logs.LSM_dangercnt);
	free(logs.LSM_rpm);
	free(logs.LSM_laserplane_nml);
	free(logs.LSM_rps);
	free(logs.LSM_rotmodes);
#endif // SAVE_LOGS_

	return 0;
}

//画像を格納する
void TakePicture(kayacoaxpress* cam, bool* flg, LSM *lsm) {
	cv::Mat temp = zero.clone();
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
		takepicid = in_imgs_saveid % cyclebuffersize;
		cam->captureFrame(in_imgs[takepicid].data);
		lsm->processflgs[takepicid] = true;
		in_imgs_saveid = (in_imgs_saveid+1)%cyclebuffersize;
		QueryPerformanceCounter(&takeend);
		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		while (taketime < 0.001)
		{
			QueryPerformanceCounter(&takeend);
			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		}
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "TakePicture() time: " << taketime << endl;
#endif // SHOW_PROCESSING_TIME_
	}
}

//現在の画像を30fps程度で出力する
void ShowLogs(bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;
		QueryPerformanceCounter(&showend);
		showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
		std::cout << "ShowLogs() time: " << showtime << endl;
	}
}

//画像とOpenGLの点群全てを表示
void ShowAllLogs(bool* flg, double* pts, int* lsmshowid, cv::Mat* imglog) {
	initGL();
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);

		//OpenCVで画像表示
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;
		
		//OpenGL描画
		drawGL_one(pts, lsmshowid);

#ifdef SAVE_IMGS_
		memcpy((imglog+log_img_cnt)->data, in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize].data, height * width * 3);
		log_img_cnt++;
		if (log_img_cnt > log_img_finish_cnt) *flg = false;
#endif // SAVE_IMGS_

		QueryPerformanceCounter(&showend);
		showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
		while (showtime < 0.033)
		{
			QueryPerformanceCounter(&showend);
			showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
		}
#ifdef SHOW_PROCESSING_TIME_
		std::cout << "ShowAllLogs() time: " << showtime << endl;
#endif // SHOW_PROCESSING_TIME_
	}
	finishGL();
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
	if (!calibxml.isOpened()) std::cout << "calib xml cannot be opened..." << endl;
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
			std::cout << "FALSE" << endl;
		}
		/*QueryPerformanceCounter(&arend);
		artime = (double)(arend.QuadPart - arstart.QuadPart) / freq.QuadPart;*/
		//std::cout << "AR time:" << artime << endl;
	}
}

//DDMotorへのコマンド送信
void SendDDMotorCommand(bool* flg) {
	//動作開始のコマンド
	snprintf(command, READBUFFERSIZE, "%c,%d,\r", rotdir, rpm);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);
	
	while (*flg)
	{
		mbedtime = 0;
		QueryPerformanceCounter(&mbedstart);
		while (mbedtime < 2.0)
		{
			QueryPerformanceCounter(&mbedstop);
			mbedtime = (double)(mbedstop.QuadPart - mbedstart.QuadPart) / freq.QuadPart;
		}
		if (rotdir == 'R') rotdir = 'L';
		else rotdir = 'R';
		snprintf(command, READBUFFERSIZE, "%c,%d,\r", rotdir, rpm);
		mbed.Send(command);
		memset(command, '\0', READBUFFERSIZE);
	}
	//終了コマンド送信
	rotdir = 'F';
	snprintf(command, READBUFFERSIZE, "%c,%d,\r", rotdir, rpm);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);
}

//Mainループでの光切断法による形状計測
int CalcLSM(LSM* lsm, Logs* logs, long long* logid) {
	//画像の格納
	lsmcalcid = (in_imgs_saveid - 1 + cyclebuffersize) % cyclebuffersize;
	if (lsm->processflgs[lsmcalcid]){
		memcpy(lsm->in_img.data, in_imgs[lsmcalcid].data, height * width * 3);
	}
	/*QueryPerformanceCounter(&lsmend);
	lsmtime_a = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
	std::cout << "CalcLSM() getimg time: " << lsmtime_a << endl;*/
	if (lsm->in_img.data!=NULL && (int)lsm->in_img.data[0]!=255)
	{
		//参照面の輝度重心の検出
		lsm->in_img(roi_ref).copyTo(lsm->ref_arc, lsm->mask_refarc(roi_ref));
#ifdef OUT_COLOR_
		cv::inRange(lsm->ref_arc, color_thr_min, color_thr_max, lsm->ref_arc_ranged);
		cv::findNonZero(lsm->ref_arc_ranged, refpts);
		refmass = 0, refmomx = 0, refmomy = 0;
		if (refpts.size() != 0)
		{
			lsm->ref_arc_src = lsm->ref_arc.ptr<uint8_t>(0);
			for (size_t i = 0; i < refpts.size(); i++)
			{
				refmass += lsm->ref_arc_src[refpts[i].y * 66 * 3 + refpts[i].x * 3 + 2];
				refmomx += (double)lsm->ref_arc_src[refpts[i].y * 66 * 3 + refpts[i].x * 3 + 2] * refpts[i].x;
				refmomy += (double)lsm->ref_arc_src[refpts[i].y * 66 * 3 + refpts[i].x * 3 + 2] * refpts[i].y;
			}
			lsm->rp[0] = refmomx / refmass + roi_ref.x;
			lsm->rp[1] = refmomy / refmass + roi_ref.y;
#endif // OUT_COLOR
#ifdef OUT_MONO_
			cv::threshold(lsm->ref_arc, lsm->ref_arc, mono_thr, 255.0, cv::THRESH_BINARY);
			cv::Moments mu = cv::moments(lsm->ref_arc(roi_ref));
			lsm->rp[0] = mu.m10 / mu.m00 + roi_ref.x;
			lsm->rp[1] = mu.m01 / mu.m00 + roi_ref.y;
#endif // OUT_MONO_
			/*QueryPerformanceCounter(&lsmend);
			lsmtime_b = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
			std::cout << "CalcLSM() calcrefpt time: " << lsmtime_b - lsmtime_a << endl;*/
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

			//ラインレーザの輝点座標を検出
			lsm->in_img.copyTo(lsm->lsm_laser, lsm->mask_lsm);
			/*QueryPerformanceCounter(&lsmend);
			lsmtime_c = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
			std::cout << "CalcLSM() calclaserpts time: " << lsmtime_c - lsmtime_b << endl;*/
#ifdef OUT_COLOR_
			cv::inRange(lsm->lsm_laser(roi_laser), color_thr_min, color_thr_max, lsm->lsm_laser_ranged);
			/*QueryPerformanceCounter(&lsmend);
			lsmtime_c = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
			std::cout << "CalcLSM() calclaserpts time: " << lsmtime_c - lsmtime_b << endl;*/
			cv::findNonZero(lsm->lsm_laser_ranged, lsm->allbps);
			/*QueryPerformanceCounter(&lsmend);
			lsmtime_c = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
			std::cout << "CalcLSM() calclaserpts time: " << lsmtime_c - lsmtime_b << endl;*/
#ifdef AUTONOMOUS_SENSING_
			alertcnt = 0, dangercnt = 0;
#endif // AUTONOMOUS_SENSING_
			//ここで同心円状に輝度重心を取得
			if (lsm->allbps.size() != 0)
			{
				roi_laser_outcnt = 0;
				roi_laser_minx = width, roi_laser_maxx = 0, roi_laser_miny = height, roi_laser_maxy = 0;
				lsm->lsm_laser_src = lsm->lsm_laser.ptr<uint8_t>(0);
				for (const auto& pts : lsm->allbps)
				{
					laser_pts.x = pts.x + roi_laser.x;
					laser_pts.y = pts.y + roi_laser.y;
					r_calc = (unsigned int)hypot(laser_pts.x - lsm->ref_center[0], laser_pts.y - lsm->ref_center[1]);
					if (roi_laser_maxx < laser_pts.x) roi_laser_maxx = laser_pts.x;
					if (roi_laser_minx > laser_pts.x) roi_laser_minx = laser_pts.x;
					if (roi_laser_maxy < laser_pts.y) roi_laser_maxy = laser_pts.y;
					if (roi_laser_miny > laser_pts.y) roi_laser_miny = laser_pts.y;
					if (r_calc < rends - rstart)
					{
						lsmmass_r[r_calc] += lsm->lsm_laser_src[laser_pts.y * colorstep + laser_pts.x * colorelem + 2];
						lsmmomx_r[r_calc] += (double)lsm->lsm_laser_src[laser_pts.y * colorstep + laser_pts.x * colorelem + 2] * laser_pts.x;
						lsmmomy_r[r_calc] += (double)lsm->lsm_laser_src[laser_pts.y * colorstep + laser_pts.x * colorelem + 2] * laser_pts.y;
					}
				}
				if (roi_laser_maxx > width - roi_laser_margin) roi_laser_maxx = width;
				else roi_laser_maxx += roi_laser_margin;
				if (roi_laser_minx < roi_laser_margin) roi_laser_minx = 0;
				else roi_laser_minx -= roi_laser_margin;
				if (roi_laser_maxy > height - roi_laser_margin) roi_laser_maxy = height;
				else roi_laser_maxy += roi_laser_margin;
				if (roi_laser_miny < roi_laser_margin) roi_laser_miny = 0;
				else roi_laser_miny -= roi_laser_margin;
				roi_laser.x = roi_laser_minx;
				roi_laser.width = roi_laser_maxx - roi_laser_minx;
				roi_laser.y = roi_laser_miny;
				roi_laser.height = roi_laser_maxy - roi_laser_miny;

				/*QueryPerformanceCounter(&lsmend);
				lsmtime_c = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
				std::cout << "CalcLSM() calclaserpts time: " << lsmtime_c - lsmtime_b << endl;*/

				for (long long rs = 0; rs < rends - rstart; rs++)
				{
					if (lsmmass_r[rs] > 0)
					{
						lsm->bp.x = lsmmomx_r[rs] / lsmmass_r[rs];
						lsm->bp.y = lsmmomy_r[rs] / lsmmass_r[rs];
						lsmmass_r[rs] = 0, lsmmomx_r[rs] = 0, lsmmomy_r[rs] = 0;

						//理想ピクセル座標系に変換
						idpix.x = lsm->det * ((lsm->bp.x - lsm->distortion[0]) - lsm->stretch_mat[1] * (lsm->bp.y - lsm->distortion[1]));
						idpix.y = lsm->det * (-lsm->stretch_mat[2] * (lsm->bp.x - lsm->distortion[0]) + lsm->stretch_mat[0] * (lsm->bp.y - lsm->distortion[1]));

						//理想ピクセル座標->直線の式とレーザ平面から輝点三次元座標の計算
						u = idpix.x;
						v = idpix.y;
						phi = hypot(u, v);
						w = lsm->map_coefficient[0] + lsm->map_coefficient[1] * pow(phi, 2) +
							lsm->map_coefficient[2] * pow(phi, 3) + lsm->map_coefficient[3] * pow(phi, 4);
						lambda = 1 / (lsm->plane_nml[0] * u + lsm->plane_nml[1] * v + lsm->plane_nml[2] * w);
						//*(pts + (long long)lsmcalcid * rs * 3 + (long long)rs * 3 + 1) = lambda * v + 100 * sin(lsm->processcnt * 0.0099);
						*(logs->LSM_pts_cycle + lsmcalcid * (rends - rstart) * 3 + rs * 3 + 0) = lambda * u;
						*(logs->LSM_pts_cycle + lsmcalcid * (rends - rstart) * 3 + rs * 3 + 1) = lambda * v;
						*(logs->LSM_pts_cycle + lsmcalcid * (rends - rstart) * 3 + rs * 3 + 2) = lambda * w;
#ifdef SAVE_LOGS_
						*(logs->LSM_pts_logs + *logid * (rends - rstart) * 3 + rs * 3 + 0) = lambda * u;
						*(logs->LSM_pts_logs + *logid * (rends - rstart) * 3 + rs * 3 + 1) = lambda * v;
						*(logs->LSM_pts_logs + *logid * (rends - rstart) * 3 + rs * 3 + 2) = lambda * w;
#endif // SAVE_LOGS_


						//ここで取得点群の距離を計算し，その結果をもとにDDMotorに対するフラグを立てる
#ifdef AUTONOMOUS_SENSING_
						if (hypot(lambda * u, lambda * v) < Ac) alertcnt++;
						if (hypot(lambda * u, lambda * v) < Dc) dangercnt++;
#endif // AUTONOMOUS_SENSING_
					}
					else
					{
						*(logs->LSM_pts_cycle + lsmcalcid * (rends - rstart) * 3 + rs * 3 + 0) = 0;
						*(logs->LSM_pts_cycle + lsmcalcid * (rends - rstart) * 3 + rs * 3 + 1) = 0;
						*(logs->LSM_pts_cycle + lsmcalcid * (rends - rstart) * 3 + rs * 3 + 2) = 0;
					}
				}

#endif // OUT_COLOR_
#ifdef OUT_MONO_
				cv::threshold(lsm->lsm_laser, lsm->lsm_laser, mono_thr, 255, cv::THRESH_BINARY);
				cv::findNonZero(lsm->lsm_laser(roi_lsm), lsm->bps);
				for (size_t i = 0; i < lsm->bps.size(); i++) { lsm->bps[i] += cv::Point(roi_lsm.x, roi_lsm.y); }
#endif // OUT_MONO_
				/*QueryPerformanceCounter(&lsmend);
				lsmtime_c = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
				std::cout << "CalcLSM() calclaserpts time: " << lsmtime_c - lsmtime_b << endl;*/

				/*std::cout << "CalcLSM() calc3dpts time: " << lsmtime-lsmtime_c << endl;*/
				//std::cout << "CalcLSM() total time: " << lsmtime << endl;
				//logs->LSM_pts.emplace_back(lsm->campts);
				//rps = { lsm->rp[0],lsm->rp[1] };
				//logs->LSM_rps.emplace_back(rps);
				lsm->processcnt++;
			}
			else
			{
				roi_laser_outcnt++;
				if (roi_laser_outcnt>10)
				{
					roi_laser.x = 0;
					roi_laser.width = width;
					roi_laser.y = 0;
					roi_laser.height = height;
				}
			}
			//DDMotorの制御
#ifdef AUTONOMOUS_SENSING_
			if (detectenableflg)
			{
				if (alertcnt >= Nc) {
					//物体検出判定
					objcnt++;
					nonobjcnt = 0;
					totaldanger += dangercnt;
					objdetected = true;
				}
				else//物体検出できなかった
				{
					if (objdetected)
					{//前フレームで物体検出がされていてかつ今フレームで物体未検出になった時
						if (objcnt > Cc) {//連続でCc回物体検出判定されていれば
							//この時点で物体検出が連続Cc回の時
							if (totaldanger < dangerthr) reciprocntdown--;
							else rotmode = true;
							if (reciprocntdown > 0)
							{//往復回数があるとき
								if (rotdir == 'R') rotdir = 'L';
								else rotdir = 'R';
								rpm = reciprorpm;
							}
							else {//往復終了の時
								rpm = rotaterpm;
								rotmode = false;
								reciprocntdown = 3;
								detectenableflg = false;
								detectenablecnt = 0;
							}
							snprintf(command, READBUFFERSIZE, "%c,%d,\r", rotdir, rpm);
							mbed.Send(command);
							memset(command, '\0', READBUFFERSIZE);
						}
					}
					else
					{//連続で物体未検出になった時
						nonobjcnt++;
						if (nonobjcnt == contnonobjcnt) {//連続Cont回目で全周囲計測に変更する
							rpm = rotaterpm;
							rotmode = false;
							snprintf(command, READBUFFERSIZE, "%c,%d,\r", rotdir, rpm);
							mbed.Send(command);
							memset(command, '\0', READBUFFERSIZE);
						}
					}
					objdetected = false;
					objcnt = 0, totaldanger = 0;
				}
			}
			else
			{//検出->未検出に変化した後連続100回は検出判定を行わない
				detectenablecnt++;
				if (detectenablecnt > detectunablecnt) detectenableflg = true;
			}

#endif // AUTONOMOUS_SENSING_
		}
	}
	else return 1;
	lsm->processflgs[lsmcalcid] = false;
	//QueryPerformanceCounter(&lsmend);
	//lsmtime = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
	return 0;
}

void Read_Reply_toEND(RS232c* robot) {
	char bufreply[256];
	while (true)
	{
		robot->Read_CRLF(bufreply, 256);
		if (bufreply[0] == 'E' || bufreply[0] == 'O') {
			break;
		}
	}
}

void ControlAxisRobot(RS232c* robot, bool* flg){
	srand(time(NULL));
	int axisspeed = 100;
	int axisposition = 200000;
	int initaxispos = 600;
	char controlcommand[READBUFFERSIZE];
	while (*flg)
	{
		//位置と速度のランダム設定
		if (initaxispos == initaxisend) initaxispos = initaxisstart;
		else if (initaxispos == initaxisstart) initaxispos = initaxisend;
		else initaxispos = initaxisstart;
		axisposition = (initaxispos + rand() % posunits+ 1) * 100; //0~100 or 600~700
		axisspeed = (rand() % speedunits + 1) * 10; //10~100で10刻み

		//コマンド送信
		snprintf(controlcommand, READBUFFERSIZE, "@S_17.1=%d\r\n", axisspeed);
		robot->Send(controlcommand);
		memset(controlcommand, '\0', READBUFFERSIZE);
		Read_Reply_toEND(robot);
		snprintf(controlcommand, READBUFFERSIZE, "@START17#P%d.1\r\n", axisposition);
		robot->Send(controlcommand);
		memset(controlcommand, '\0', READBUFFERSIZE);
		Read_Reply_toEND(robot);
	}
}

void wait_QueryPerformance(double finishtime, LARGE_INTEGER freq) {
	LARGE_INTEGER waitstart, waitstop;
	double waittime = 0;
	QueryPerformanceCounter(&waitstart);
	while (waittime < finishtime)
	{
		QueryPerformanceCounter(&waitstop);
		waittime = (double)(waitstop.QuadPart - waitstart.QuadPart) / freq.QuadPart;
	}
}