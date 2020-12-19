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

//�O���[�o���ϐ�
/// �J�����p�����[�^
const int width = 896;
const int height = 896;
const float fps = 1000.0;
const float exposuretime = 912.0;
const int offsetx = 480;
const int offsety = 92;
/// �摜�Ɋւ���ϐ�
vector<cv::Mat> in_imgs;
int in_imgs_saveid = 0;
cv::Mat full, zero;
int takepicid = 0;
/// ���ԂɊւ���ϐ�
int timeout = 10;
LARGE_INTEGER freq, start;
double logtime = 0;
/// AR�}�[�J�Ɋւ���ϐ�
unsigned int marker_num = 1;
vector<cv::Mat> Rs;
vector< cv::Vec3d > Rvecs, Tvecs;
cv::Vec3d Tvec_id0;
double dir_arid0_rad;
vector<bool> detect_arid0_flgs;
/// �r������p��Mutex
cv::Mutex mutex;
/// DDMotor����Ɋւ���ϐ�
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
long long detectfailcnt = 0;
/// ���ؒf�@�v�Z�p�ϐ�
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
const int rstart = 104, rends = 432;
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
int lsmcalcid = 0;
int showglid = 0;

/// ���O�Ɋւ���ϐ�
const int cyclebuffersize = 10;
//�f�o�b�O�p�ϐ�
LARGE_INTEGER lsmstart, lsmend, takestart, takeend, arstart, arend, showstart, showend;
double taketime = 0, lsmtime = 0, artime = 0, showtime = 0;
double lsmtime_a, lsmtime_b, lsmtime_c, lsmtime_d;

#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;
namespace fs = std::filesystem;

//DEFINE�Q
//#define SAVE_LOGS_
//#define SAVE_IMGS_
#define OUT_COLOR_
//#define OUT_MONO_

//�v���g�^�C�v�錾
void TakePicture(kayacoaxpress* cam, bool* flg, LSM* lsm);
void ShowLogs(bool* flg);
void ShowAllLogs(bool* flg, double* pts, int* lsmshowid);
void DetectAR(bool* flg);
void SendDDMotorCommand(bool* flg);
int CalcLSM(LSM* lsm, Logs* logs, double* pts);

int main() {
	//�p�����[�^
	bool flg = true;
	LARGE_INTEGER end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// �P�ʏK��

	//���ؒf�@�Ɋւ���\���̂̃C���X�^���X
	LSM lsm;
	lsm.processcnt = 0;
	lsm.buffersize = cyclebuffersize;

	//���O�ۑ��Ɋւ���\���̂̃C���X�^���X
	Logs logs;
	logs.LSM_pts_cycle = (double*)malloc(sizeof(double) * cyclebuffersize * (rends - rstart) * 3);
	
	//�J�����N���X�̃C���X�^���X�̐���
	kayacoaxpress cam;
	cam.connect(0);

	//�p�����[�^�̐ݒ�
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

	//���[�UCalibration�̌��ʂ̌Ăяo��
	FILE* fcam, * flaser;
	fcam = fopen("202011251943_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++){ fscanf(fcam, "%lf,", &lsm.map_coefficient[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &lsm.stretch_mat[i]); }
	swap(lsm.stretch_mat[1], lsm.stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &lsm.distortion[i]); }
	fclose(fcam);
	flaser = fopen("202011251943_laserinterpolparam.csv", "r");
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pa[i]); }
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pb[i]); }
	for (size_t i = 0; i < 10; i++) { fscanf(flaser, "%lf,", &lsm.pc[i]); }
	for (size_t i = 0; i < 2; i++) { fscanf(flaser, "%lf,", &lsm.ref_center[i]); }
	fscanf(flaser, "%lf,", &lsm.ref_radius);
	fscanf(flaser, "%lf,", &lsm.ref_arcwidth);
	fclose(flaser);
	lsm.det = 1 / (lsm.stretch_mat[0] - lsm.stretch_mat[1] * lsm.stretch_mat[2]);

	//MBED��RS232�ڑ�
	mbed.Connect("COM4", 115200, 8, NOPARITY, 0, 0, 0, 5000, 20000);

	//�J���[OR���m�N��
#ifdef OUT_MONO_
	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(0));
#endif // OUT_MONO_
#ifdef OUT_COLOR_
	full = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(255));
	zero = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC3, cv::Scalar::all(0));
#endif // OUT_COLOR_
	lsm.in_img = zero.clone();

	//�}�X�N�摜�̐���
	lsm.mask_refarc = zero.clone();
	cv::circle(lsm.mask_refarc, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius - lsm.ref_arcwidth / 2), cv::Scalar::all(255), (int)lsm.ref_arcwidth);
	roi_ref = cv::Rect((int)(lsm.ref_center[0] - lsm.ref_radius), (int)(lsm.ref_center[1] - lsm.ref_radius), (int)(2 * lsm.ref_radius), (int)(2 * lsm.ref_radius));
	lsm.mask_lsm = zero.clone();
	cv::circle(lsm.mask_lsm, cv::Point((int)width/2, (int)height/2), 430, cv::Scalar::all(255), -1);
	cv::circle(lsm.mask_lsm, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius)+20, cv::Scalar::all(0), -1);
	roi_laser = cv::Rect(0, 0, width, height);

#ifdef SAVE_IMGS_
	//�擾�摜���i�[����Vector�̍쐬
	std::cout << "Set Mat Vector..." << endl;
	for (size_t i = 0; i < (int)(timeout)*fps + 100; i++)
	{
		in_imgs.push_back(zero.clone());
		lsm.processflgs.push_back(false);
	}
#endif // SAVE_IMGS_
#ifndef SAVE_IMGS_
	std::cout << "Set Mat Cycle Buffer..." << endl;
	for (size_t i = 0; i < cyclebuffersize; i++)
	{
		in_imgs.push_back(zero.clone());
		lsm.processflgs.push_back(false);
	}
#endif // !SAVE_IMGS_

	//OpenGL������
	//initGL();


	//�J�����N��
	std::cout << "Aroud 3D Sensing Start!" << endl;
	cam.start();

	//�v���J�n
	if (!QueryPerformanceCounter(&start)) { return 0; }

	//Thread�̍쐬
	/// 1000fps�ŉ摜���i�[��������X���b�h
	thread thr1(TakePicture, &cam, &flg, &lsm);
	/// ���݂̉摜��PC�ɏo�͂��Č�����悤����X���b�h
	thread thr2(ShowAllLogs, &flg, logs.LSM_pts_cycle, &showglid);
	/// AR�}�[�J�����o���ʒu�p�����v�Z����X���b�h
	//thread thr3(DetectAR, &flg);
	/// DDMotor�ɃR�}���h�𑗐M����X���b�h
	//thread thr4(SendDDMotorCommand, & flg);
	/// OpenGL�œ_�Q��\������X���b�h
	//thread thr5(drawGL2, &flg, logs.LSM_pts_cycle, &showglid);
	
	cv::Mat temp = zero.clone();
	//���C�����[�v
	/// �擾���ꂽ�摜������ؒf�@�ŎO�����ʒu���v�Z����
	while (flg)
	{
		//�摜�擾
		//takepicid = in_imgs_saveid % cyclebuffersize;
		//QueryPerformanceCounter(&takestart);
		//cam.captureFrame(in_imgs[takepicid].data);
		////memcpy(in_imgs[takepicid].data, temp.data, height * width * 3);
		//lsm.processflgs[takepicid] = true;
		//in_imgs_saveid = (in_imgs_saveid + 1) % cyclebuffersize;
		//QueryPerformanceCounter(&takeend);
		//taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		//std::cout << "TakePicture() time: " << taketime << endl;

		//���ؒf�̍��x�̍X�V
		QueryPerformanceCounter(&start);
		CalcLSM(&lsm, &logs, logs.LSM_pts_cycle);
		QueryPerformanceCounter(&end);
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		std::cout << "CalcLSM() time: " << logtime << endl;

		//OpenGL�ł̓_�Q�\��
		/*QueryPerformanceCounter(&start);
		showglid = (in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize;
		drawGL_one(logs.LSM_pts_cycle, &showglid);
		QueryPerformanceCounter(&end);
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		std::cout << "drawGL() time: " << logtime << endl;*/
		/*if (!QueryPerformanceCounter(&end)) { return 0; }
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		logs.LSM_times.push_back(logtime);*/
		/*logs.LSM_modes.push_back(mode);*/

		//�摜�\��
		/*QueryPerformanceCounter(&start);
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
		int key = cv::waitKey(1);
		if (key == 'q') flg = false;
		QueryPerformanceCounter(&end);
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		std::cout << "Showimg() time: " << logtime << endl;*/
		
		
		//�����̍X�V
		/*if (!QueryPerformanceCounter(&end)) { return 0; }
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;*/
#ifdef SAVE_IMGS_
		if (logtime > timeout) { flg = false; }
#endif // SAVE_IMGS_

		
	}

	//�X���b�h�̒�~
	if (thr1.joinable())thr1.join();
	if (thr2.joinable())thr2.join();
	//if (thr3.joinable())thr3.join();
	//if (thr4.joinable())thr4.join();
	//if (thr5.joinable())thr5.join();

	//OpenGL�̒�~
	//finishGL();

	//�J�����̒�~�CRS232C�̐ؒf
	cam.stop();
	cam.disconnect();

	//�v�Z�������W�C�擾�摜�̕ۑ�
	/// Log�t�@�C���̍쐬
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

	/// Log�̕ۑ�
	std::cout << "Saving logs..." << endl;
	for (size_t i = 0; i < logs.LSM_times.size(); i++)
	{
		if (logs.LSM_pts[i].size() > 10000) { continue; }//�o�O���
		fprintf(fr, "%lf,", logs.LSM_times[i]);
		fprintf(fr, "%lf,%lf,", logs.LSM_rps[i][0], logs.LSM_rps[i][1]);
		if (logs.LSM_modes[i]=='L') { fprintf(fr, "%lf,", 1.0); }
		else if (logs.LSM_modes[i] == 'R') { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < logs.LSM_pts[i].size(); j++) { fprintf(fr, "%lf,", logs.LSM_pts[i][j].at<double>(0, 0)); }
		if (logs.LSM_pts[i].size() == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < logs.LSM_pts[i].size(); j++) { fprintf(fr, "%lf,", logs.LSM_pts[i][j].at<double>(0, 1)); }
		if (logs.LSM_pts[i].size() == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");

		for (size_t j = 0; j < logs.LSM_pts[i].size(); j++) { fprintf(fr, "%lf,", logs.LSM_pts[i][j].at<double>(0, 2)); }
		if (logs.LSM_pts[i].size() == 0) { fprintf(fr, "%lf,", 0.0); }
		fprintf(fr, "\n");
	}
	std::cout << "Logs finish!" << endl;
	fclose(fr);

	/// �摜��ۑ�
#ifdef SAVE_IMGS_
	std::cout << "Saving imgs..." << endl;
	char picdir[256];
	strftime(picdir, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S/Pictures", &now);
	if (!fs::create_directories(picdir)) { return 0; }
	char picturename[256];
	char picsubname[256];
	strftime(picsubname, 256, "D:/Github_output/HighSpeedAroundSensing/HighSpeedAroundSensing3D/results/%y%m%d/%H%M%S/Pictures/frame", &now);
	for (int i = 0; i < in_imgs_saveid; i++)
	{
		sprintf(picturename, "%s%05d.png", picsubname, i);//png�t���k
		cv::imwrite(picturename, in_imgs[i]);
	}
	std::cout << "Imgs finished!" << endl;
#endif // SAVE_IMGS_

#endif // SAVE_LOGS_

	free(logs.LSM_pts_cycle);

	return 0;
}

//�摜���i�[����
void TakePicture(kayacoaxpress* cam, bool* flg, LSM *lsm) {
	cv::Mat temp = zero.clone();
	while (*flg)
	{
		QueryPerformanceCounter(&takestart);
#ifdef SAVE_IMGS_
		takepicid = in_imgs_saveid;
#endif // SAVE_IMG
#ifndef SAVE_IMGS_
		takepicid = in_imgs_saveid % cyclebuffersize;
#endif // !SAVE_IMGS
		cam->captureFrame(in_imgs[takepicid].data);
		//memcpy(in_imgs[takepicid].data, temp.data, height * width * 3);
		lsm->processflgs[takepicid] = true;
		in_imgs_saveid = (in_imgs_saveid+1)%cyclebuffersize;
		QueryPerformanceCounter(&takeend);
		taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		while (taketime < 0.001)
		{
			QueryPerformanceCounter(&takeend);
			taketime = (double)(takeend.QuadPart - takestart.QuadPart) / freq.QuadPart;
		}
		
		std::cout << "TakePicture() time: " << taketime << endl;
	}
}

//���݂̉摜��30fps���x�ŏo�͂���
void ShowLogs(bool* flg) {
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);
#ifdef SAVE_IMGS
		cv::imshow("img", in_imgs[in_imgs_saveid - 2]);
#endif // SAVE_IMGS
#ifndef SAVE_IMGS
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
#endif // !SAVE_IMGS
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;
		QueryPerformanceCounter(&showend);
		showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
		std::cout << "ShowLogs() time: " << showtime << endl;
	}
}

//�摜��OpenGL�̓_�Q�S�Ă�\��
void ShowAllLogs(bool* flg, double* pts, int* lsmshowid) {
	initGL();
	while (*flg)
	{
		QueryPerformanceCounter(&showstart);
#ifdef SAVE_IMGS
		cv::imshow("img", in_imgs[in_imgs_saveid - 2]);
#endif // SAVE_IMGS
#ifndef SAVE_IMGS
		cv::imshow("img", in_imgs[(in_imgs_saveid - 2 + cyclebuffersize) % cyclebuffersize]);
#endif // !SAVE_IMGS
		int key = cv::waitKey(1);
		if (key == 'q') *flg = false;
		
		//OpenGL�`��
		drawGL_one(pts, lsmshowid);

		QueryPerformanceCounter(&showend);
		showtime = (double)(showend.QuadPart - showstart.QuadPart) / freq.QuadPart;
		std::cout << "ShowAllLogs() time: " << showtime << endl;
	}
	finishGL();
}

//AR�}�[�J�̌��o�ƈʒu�p������
void DetectAR(bool* flg) {
	//�p�����[�^�̐ݒ�
	cv::Mat R;
	for (size_t i = 0; i < marker_num; i++) Rs.push_back(R);
	//�����̎w��
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	//Perspective�J�����̓����p�����[�^���擾
	string calib_dir = "PerspectiveCamCalibParams.xml";
	cv::FileStorage calibxml(calib_dir, cv::FileStorage::READ);
	if (!calibxml.isOpened()) std::cout << "calib xml cannot be opened..." << endl;
	cv::Mat K, D;
	calibxml["K"] >> K;
	calibxml["D"] >> D;
	while (*flg)
	{
		//QueryPerformanceCounter(&arstart);
		//�}�[�J���o
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		if (in_imgs_saveid>2){ cv::aruco::detectMarkers(in_imgs[in_imgs_saveid - 2], dictionary, corners, ids); }
		//�}�[�J���o���C�ʒu�p�����v�Z����
		if (ids.size() > 0) {
			cv::aruco::estimatePoseSingleMarkers(corners, 0.2, K, D, Rvecs, Tvecs);
			for (size_t i = 0; i < ids.size(); i++)
			{
				//ID=0�̃}�[�J���o�����ƕ����x�N�g���ۑ�
				if (ids[i] == 0) {
					Tvec_id0 = Tvecs[i];
					detect_arid0_flgs.push_back(true);
				}
			}
		}
		//�}�[�J���o����Ȃ������Ƃ�
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

//DDMotor�ւ̃R�}���h���M
void SendDDMotorCommand(bool* flg) {
	//����J�n�̃R�}���h
	snprintf(command, READBUFFERSIZE, "%c,%d,%d,%d,\r", mode, rpm, initpulse, movepulse);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);
	while (*flg)
	{
		if (detect_arid0_flgs.size()>processarcnt)
		{
			//AR�}�[�J���o�����Ƃ��C�Ǐ��v��
			if (detect_arid0_flgs[processarcnt])
			{
				detectfailcnt = 0;
				//AR�}�[�J�̕����v�Z
				dir_arid0_rad = atan2(Tvec_id0[1], Tvec_id0[0]) + M_PI / 2;
				initpulse = ((int)(dir_arid0_rad * 180 / M_PI / 360 * rotpulse) + rotpulse - movepulse / 2) % (rotpulse);
				if (initpulse < 0) initpulse += rotpulse;
				//�R�}���h���M
				mode = 'L';
				rpm = 200;
			}
			else// if(!detect_arid0_flgs[processarcnt]&& !detect_arid0_flgs[processarcnt-1])
			{//AR�}�[�J��2��A���Ȃ��Ƃ��C�S���v��
				detectfailcnt++;
				if (detectfailcnt>10){ 
					mode = 'R';
					rpm = 500;
				}
			}
			snprintf(command, READBUFFERSIZE, "%c,%d,%d,%d,\r", mode, rpm, initpulse, movepulse);
			mbed.Send(command);
			memset(command, '\0', READBUFFERSIZE);
			processarcnt++;
		}
	}
	//�I���R�}���h���M
	mode = 'F';
	snprintf(command, READBUFFERSIZE, "%c,%d,%d,%d,\r", mode, rpm, initpulse, movepulse);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);
}

//Main���[�v�ł̌��ؒf�@�ɂ��`��v��
int CalcLSM(LSM* lsm, Logs* logs, double* pts) {
	QueryPerformanceCounter(&lsmstart);

	//�摜�̊i�[
	lsmcalcid = (in_imgs_saveid - 1 + cyclebuffersize) % cyclebuffersize;
	if (lsm->processflgs[lsmcalcid]){
#ifdef SAVE_IMGS_
		lsm->in_img = in_imgs[in_imgs_saveid - 1].clone();
#endif // SAVE_IMGS_
#ifndef SAVE_IMGS_
		memcpy(lsm->in_img.data, in_imgs[lsmcalcid].data, height * width * 3);
		//lsm->in_img = in_imgs[lsmcalcid].clone();
#endif // !SAVE_IMGS_
	}
	/*QueryPerformanceCounter(&lsmend);
	lsmtime_a = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
	std::cout << "CalcLSM() getimg time: " << lsmtime_a << endl;*/
	if (lsm->in_img.data!=NULL && (int)lsm->in_img.data[0]!=255)
	{
		//�Q�Ɩʂ̋P�x�d�S�̌��o
		lsm->in_img(roi_ref).copyTo(lsm->ref_arc, lsm->mask_refarc(roi_ref));
#ifdef OUT_COLOR_
		cv::inRange(lsm->ref_arc, color_thr_min, color_thr_max, lsm->ref_arc_ranged);
		cv::findNonZero(lsm->ref_arc_ranged, refpts);
		refmass = 0, refmomx = 0, refmomy = 0;
		if (refpts.size() != 0)
		{
			for (size_t i = 0; i < refpts.size(); i++)
			{
				refmass += (double)lsm->ref_arc.data[refpts[i].y * 66 * 3 + refpts[i].x * 3 + 2];
				refmomx += (double)lsm->ref_arc.data[refpts[i].y * 66 * 3 + refpts[i].x * 3 + 2] * refpts[i].x;
				refmomy += (double)lsm->ref_arc.data[refpts[i].y * 66 * 3 + refpts[i].x * 3 + 2] * refpts[i].y;
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
			//���[�U���ʂ̖@���x�N�g���̌v�Z
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

			//���C�����[�U�̋P�_���W�����o
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
			//�����œ��S�~��ɋP�x�d�S���擾
			if (lsm->allbps.size() != 0)
			{
				roi_laser_outcnt = 0;
				roi_laser_minx = width, roi_laser_maxx = 0, roi_laser_miny = height, roi_laser_maxy = 0;
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
						lsmmass_r[r_calc] += lsm->lsm_laser.data[laser_pts.y * colorstep + laser_pts.x * colorelem + 2];
						lsmmomx_r[r_calc] += (double)lsm->lsm_laser.data[laser_pts.y * colorstep + laser_pts.x * colorelem + 2] * laser_pts.x;
						lsmmomy_r[r_calc] += (double)lsm->lsm_laser.data[laser_pts.y * colorstep + laser_pts.x * colorelem + 2] * laser_pts.y;
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
				for (int rs = 0; rs < rends - rstart; rs++)
				{
					if (lsmmass_r[rs] > 0)
					{
						lsm->bp.x = lsmmomx_r[rs] / lsmmass_r[rs];
						lsm->bp.y = lsmmomy_r[rs] / lsmmass_r[rs];
						lsmmass_r[rs] = 0, lsmmomx_r[rs] = 0, lsmmomy_r[rs] = 0;

						//���z�s�N�Z�����W�n�ɕϊ�
						idpix.x = lsm->det * ((lsm->bp.x - lsm->distortion[0]) - lsm->stretch_mat[1] * (lsm->bp.y - lsm->distortion[1]));
						idpix.y = lsm->det * (-lsm->stretch_mat[2] * (lsm->bp.x - lsm->distortion[0]) + lsm->stretch_mat[0] * (lsm->bp.y - lsm->distortion[1]));

						//���z�s�N�Z�����W->�����̎��ƃ��[�U���ʂ���P�_�O�������W�̌v�Z
						u = idpix.x;
						v = idpix.y;
						phi = hypot(u, v);
						w = lsm->map_coefficient[0] + lsm->map_coefficient[1] * pow(phi, 2) +
							lsm->map_coefficient[2] * pow(phi, 3) + lsm->map_coefficient[3] * pow(phi, 4);
						lambda = 1 / (lsm->plane_nml[0] * u + lsm->plane_nml[1] * v + lsm->plane_nml[2] * w);
						//calcpt[0] = lambda * u;
						////calcpt[1] = lambda * v + 100*sin(lsm->processcnt*0.0099); //�f�o�b�O�p
						//calcpt[1] = lambda * v;
						//calcpt[2] = lambda * w;
						/*lsm->campts.emplace_back(calcpt);*/
						*(pts + (long long)lsmcalcid * rs * 3 + (long long)rs * 3 + 0) = lambda * u;
						*(pts + (long long)lsmcalcid * rs * 3 + (long long)rs * 3 + 1) = lambda * v + 100 * sin(lsm->processcnt * 0.0099);
						*(pts + (long long)lsmcalcid * rs * 3 + (long long)rs * 3 + 2) = lambda * w;
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
		}
	}
	lsm->processflgs[lsmcalcid] = false;
	QueryPerformanceCounter(&lsmend);
	lsmtime = (double)(lsmend.QuadPart - lsmstart.QuadPart) / freq.QuadPart;
	return 0;
}