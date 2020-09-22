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


//�v���g�^�C�v�錾
void TakePicture(kayacoaxpress* cam, bool* flg);
void ShowLogs(bool* flg);
void DetectAR(bool* flg);

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


	//�擾�摜���i�[����Vector�̍쐬
	cout << "Set Mat Vector..." << endl;
	for (size_t i = 0; i < (int)(timeout)*fps+10; i++)
	{
		in_imgs.push_back(cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255)));
	}

	//�摜�o�͗pMat
	in_img_now = cv::Mat(cam.getParam(paramTypeCamera::paramInt::HEIGHT), cam.getParam(paramTypeCamera::paramInt::WIDTH), CV_8UC1, cv::Scalar::all(255));



	//�J�����N��
	cout << "Aroud 3D Sensing Start!" << endl;
	cam.start();

	//Thread�̍쐬
	/// 1000fps�ŉ摜���i�[��������X���b�h
	thread thr1(TakePicture, &cam, &flg);
	/// ���݂̉摜��PC�ɏo�͂��Č�����悤����X���b�h
	thread thr2(ShowLogs, &flg);
	/// AR�}�[�J�����o���ʒu�p�����v�Z����X���b�h
	//thread thr3(DetectAR, &flg);
	

	//�v���J�n
	if (!QueryPerformanceCounter(&start)) { return 0; }


	//���C�����[�v
	/// �擾���ꂽ�摜������ؒf�@�ŎO�����ʒu���v�Z����
	while (flg)
	{
		//���ؒf�̍��x�̍X�V

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
	if (thr2.joinable())thr2.join();
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