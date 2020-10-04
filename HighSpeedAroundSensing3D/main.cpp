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


#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//�O���[�o���ϐ�
/// �摜�Ɋւ���ϐ�
cv::Mat in_img_now;
vector<cv::Mat> in_imgs;
int in_imgs_saveid = 0;
/// ���ԂɊւ���ϐ�
int timeout = 10;
LARGE_INTEGER freq, start;
double logtime = 0;
/// AR�}�[�J�Ɋւ���ϐ�
unsigned int marker_num = 1;
vector<cv::Mat> Rs;
vector< cv::Vec3d > Rvecs, Tvecs;
/// �r������p��Mutex
cv::Mutex mutex;
/// DDMotor����Ɋւ���ϐ�
RS232c mbed;
char command[256] = "";
int rpm = 10;
char mode = 'R';
int initpulse = 100;
int movepulse = 100;
/// ���ؒf�@�v�Z�p�ϐ�
double phi, lambda, u, v, w;
cv::Mat campt;


//�v���g�^�C�v�錾
void TakePicture(kayacoaxpress* cam, bool* flg);
void ShowLogs(bool* flg);
void DetectAR(bool* flg);
void SendDDMotorCommand(bool* flg);
int CalcLSM(LSM* lsm);

int main() {
	//�p�����[�^
	bool flg = true;
	LARGE_INTEGER end;
	if (!QueryPerformanceFrequency(&freq)) { return 0; }// �P�ʏK��

	//�J�����p�����[�^
	int width = 1920;
	int height = 1080;
	float fps = 1000.0;
	float exposuretime = 912.0;
	int offsetx = 0;
	int offsety = 0;

	//���ؒf�@�Ɋւ���\���̂̃C���X�^���X
	LSM lsm;
	lsm.processcnt = 0;

	//�J�����N���X�̃C���X�^���X�̐���
	kayacoaxpress cam;
	cam.connect(0);

	//�p�����[�^�̐ݒ�
	cout << "Set Camera Params..." << endl;
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x1);
	cam.parameter_all_print();

	//���[�UCalibration�̌��ʂ̌Ăяo��
	FILE* fcam, * flaser;
	fcam = fopen("202009301655_fisheyeparam.csv", "r");
	for (size_t i = 0; i < 4; i++){ fscanf(fcam, "%lf,", &lsm.map_coefficient[i]); }
	for (size_t i = 0; i < 4; i++) { fscanf(fcam, "%lf,", &lsm.stretch_mat[i]); }
	swap(lsm.stretch_mat[1], lsm.stretch_mat[2]);
	for (size_t i = 0; i < 2; i++) { fscanf(fcam, "%lf,", &lsm.distortion[i]); }
	fclose(fcam);
	flaser = fopen("202010010508_laserinterpolparam.csv", "r");
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

	//�摜�o�͗pMat
	in_img_now = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(255));

	//�}�X�N�摜�̐���
	lsm.mask_refarc = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(0));
	cv::circle(lsm.mask_refarc, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius - lsm.ref_arcwidth / 2), cv::Scalar::all(255), (int)lsm.ref_arcwidth);
	lsm.mask_lsm = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(255));
	cv::circle(lsm.mask_lsm, cv::Point((int)lsm.ref_center[0], (int)lsm.ref_center[1]), (int)(lsm.ref_radius), cv::Scalar::all(0), -1);

	//�擾�摜���i�[����Vector�̍쐬
	cout << "Set Mat Vector..." << endl;
	for (size_t i = 0; i < (int)(timeout)*fps+10; i++)
	{
		in_imgs.push_back(cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	}

	//�J�����N��
	cout << "Aroud 3D Sensing Start!" << endl;
	cam.start();

	//Thread�̍쐬
	/// 1000fps�ŉ摜���i�[��������X���b�h
	thread thr1(TakePicture, &cam, &flg);
	/// ���݂̉摜��PC�ɏo�͂��Č�����悤����X���b�h
	//thread thr2(ShowLogs, &flg);
	/// AR�}�[�J�����o���ʒu�p�����v�Z����X���b�h
	//thread thr3(DetectAR, &flg);
	

	//�v���J�n
	if (!QueryPerformanceCounter(&start)) { return 0; }


	//���C�����[�v
	/// �擾���ꂽ�摜������ؒf�@�ŎO�����ʒu���v�Z����
	while (flg)
	{
		//���ؒf�̍��x�̍X�V
		CalcLSM(&lsm);

		//�����̍X�V
		if (!QueryPerformanceCounter(&end)) { return 0; }
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		if (logtime > timeout)
		{
			flg = false;
		}
	}

	//�X���b�h�̒�~
	if (thr1.joinable())thr1.join();
	//if (thr2.joinable())thr2.join();
	//if (thr3.joinable())thr3.join();

	//�v�Z�������W�C�擾�摜�̕ۑ�

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

//���݂̉摜��30fps���x�ŏo�͂���
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
	if (!calibxml.isOpened()) cout << "calib xml cannot be opened..." << endl;
	cv::Mat K, D;
	calibxml["K"] >> K;
	calibxml["D"] >> D;
	while (*flg)
	{
		//�}�[�J���o
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		cv::aruco::detectMarkers(in_img_now, dictionary, corners, ids);
		//�}�[�J���o���C�ʒu�p�����v�Z����
		if (ids.size() > 0) {
			cv::aruco::estimatePoseSingleMarkers(corners, 0.2, K, D, Rvecs, Tvecs);
			for (size_t i = 0; i < ids.size(); i++)
			{
				cv::Rodrigues(Rvecs[i], Rs[i]);
			}
		}
	}
}

//DDMotor�ւ̃R�}���h���M
void SendDDMotorCommand(bool* flg) {
	while (*flg)
	{

	}
}

//Main���[�v�ł̌��ؒf�@�ɂ��`��v��
int CalcLSM(LSM *lsm) {
	//�ϐ��̃��Z�b�g
	lsm->bps.clear();
	lsm->idpixs.clear();
	lsm->campts.clear();

	lsm->in_img = in_imgs[lsm->processcnt].clone();
	lsm->processcnt++;
	if (lsm->in_img.data!=NULL)
	{
		//�Q�Ɩʂ̋P�x�d�S�̌��o
		lsm->in_img.copyTo(lsm->ref_arc, lsm->mask_refarc);
		cv::threshold(lsm->ref_arc, lsm->ref_arc, 240.0, 255.0, cv::THRESH_BINARY);
		cv::Moments mu = cv::moments(lsm->ref_arc);
		lsm->rp[0] = mu.m10 / mu.m00;
		lsm->rp[1] = mu.m01 / mu.m00;
		//���C�����[�U�̋P�_���W�����o
		lsm->in_img.copyTo(lsm->lsm_laser, lsm->mask_lsm);
		cv::threshold(lsm->lsm_laser, lsm->lsm_laser, 240, 255, cv::THRESH_BINARY);
		/// �����ŋP�_�Q�̍��W���v�Z����
		
		//���[�U���ʂ̖@���x�N�g���̌v�Z
		lsm->plane_nml[0] = lsm->pa[0] + lsm->pa[1] * lsm->rp[0] + lsm->pa[2] * lsm->rp[1] + lsm->pa[3] * pow(lsm->rp[0], 2)
			+ lsm->pa[4] * lsm->rp[0] * lsm->rp[1] + lsm->pa[5] * pow(lsm->rp[1], 2) + lsm->pa[6] * pow(lsm->rp[0], 3)
			+ lsm->pa[7] * pow(lsm->rp[0], 2) * lsm->rp[1] + lsm->pa[8] * lsm->rp[0] * pow(lsm->rp[0], 2)
			+ lsm->pa[9] * pow(lsm->rp[1], 3);
		lsm->plane_nml[1] = lsm->pb[0] + lsm->pb[1] * lsm->rp[0] + lsm->pb[2] * lsm->rp[1] + lsm->pb[3] * pow(lsm->rp[0], 2)
			+ lsm->pb[4] * lsm->rp[0] * lsm->rp[1] + lsm->pb[5] * pow(lsm->rp[1], 2) + lsm->pb[6] * pow(lsm->rp[0], 3)
			+ lsm->pb[7] * pow(lsm->rp[0], 2) * lsm->rp[1] + lsm->pb[8] * lsm->rp[0] * pow(lsm->rp[0], 2)
			+ lsm->pb[9] * pow(lsm->rp[1], 3);
		lsm->plane_nml[2] = lsm->pc[0] + lsm->pc[1] * lsm->rp[0] + lsm->pc[2] * lsm->rp[1] + lsm->pc[3] * pow(lsm->rp[0], 2)
			+ lsm->pc[4] * lsm->rp[0] * lsm->rp[1] + lsm->pc[5] * pow(lsm->rp[1], 2) + lsm->pc[6] * pow(lsm->rp[0], 3)
			+ lsm->pc[7] * pow(lsm->rp[0], 2) * lsm->rp[1] + lsm->pc[8] * lsm->rp[0] * pow(lsm->rp[0], 2)
			+ lsm->pc[9] * pow(lsm->rp[1], 3);

		//���z�s�N�Z�����W�n�ɕϊ�
		for (size_t i = 0; i < lsm->bps.size(); i++)
		{
			cv::Point2f idpix;
			idpix.x = lsm->det * ((lsm->bps[i].x - lsm->distortion[0]) - lsm->stretch_mat[1] * (lsm->bps[i].y- lsm->distortion[1]));
			idpix.y = lsm->det * (-lsm->stretch_mat[2] * (lsm->bps[i].x - lsm->distortion[0]) + lsm->stretch_mat[0] * (lsm->bps[i].y - lsm->distortion[1]));
			lsm->idpixs.push_back(idpix);
		}

		//���z�s�N�Z�����W->�����̎��ƃ��[�U���ʂ���P�_�O�������W�̌v�Z
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

	return 0;
}