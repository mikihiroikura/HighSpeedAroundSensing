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
#pragma comment(lib, "BaslerLib" CAMERA_EXT) //BaslerLib.lib�𓱓�

#pragma warning(disable:4996)

using namespace std;

struct Capture {
	bool flg;
	basler cam;
	cv::Mat in_img;
};

//�v���g�^�C�v�錾
void TakePicture(Capture* cap);

//�摜�ۑ���̃f�B���N�g��
string save_dir = "D:\save_img";

int main(int argc, char* argv[]) {
	//�J�����̃Z�b�g�A�b�v
	Capture cap;
	int width = 640;
	int height = 480;
	float fps = 750.0f;
	float gain = 1.0f;
	vector<cv::Mat> save_imgs;

	cap.cam.connect(0);
	//�J�������ʃp�����[�^�ݒ�
	cap.cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cap.cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);

	//Basler�̃J�����p�����[�^�ݒ�
	cap.cam.setParam(paramTypeBasler::Param::ExposureTime, 1280.0f);
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	cap.cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cap.cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cap.cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cap.cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	cap.cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame); //��Ƀo�b�t�@���X�V
	cap.cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);

	cap.cam.parameter_all_print();

	//�ϐ���`
	cap.in_img = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255));
	bool flg = true;

	//DDmotor�Z�b�g�A�b�v
	RS232c ddmotor;
	char ddmotor_buff[256];

	//�J�����N��
	cap.cam.start();

	//DDmotor�N��
	ddmotor.Connect("COM3", 38400, 8, NOPARITY, 0, 0, 0, 20000, 20000);

	//�摜���X�V��������X���b�h
	thread thr(TakePicture, &cap);

	//DDmotor�̃T�[�{�N��
	//Task�F������DDMotor�̃T�[�{��ON�ɂȂ��Ă��邩�m�F����R�[�h
	ddmotor.Send("$O\r");

	while (1)
	{
		cv::imshow("img", cap.in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
		else if (key == 'p') { save_imgs.push_back(cap.in_img.clone()); } //�B�����C�f�[�^���i�[
		else if (key == 'r') { ddmotor.Send("$I4500,10\r"); }//DDMotor����]
	}
	flg = false;
	if (thr.joinable())thr.join();

	//�J�����ؒf
	cap.cam.stop();
	cap.cam.disconnect();

	//DDmotor�̃T�[�{OFF
	ddmotor.Send("$F\r");

	//�擾�����摜�̈ꊇ�ۑ�
	for (size_t i = 0; i < save_imgs.size(); i++)
	{
		cv::imwrite(save_dir + "no" + to_string(i) + ".png", save_imgs[i]);
	}

	return 0;
}

//�X���b�h�֐�
void TakePicture(Capture* cap) {
	while (cap->flg)
	{
		cap->cam.captureFrame(cap->in_img.data);
	}
}