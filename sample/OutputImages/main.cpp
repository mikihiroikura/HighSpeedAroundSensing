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
#pragma comment(lib, "BaslerLib" CAMERA_EXT) //BaslerLib.lib�𓱓�

#pragma warning(disable:4996)

using namespace std;

//�v���g�^�C�v�錾
void TakePicture(basler *cam, cv::Mat *img, bool *flg);

int main() {
	//�J�����̃Z�b�g�A�b�v
	basler cam;
	int width = 640;
	int height = 480;
	float fps = 750.0f;
	float gain = 12.0f;


	cam.connect(0);
	//�J�������ʃp�����[�^�ݒ�
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);

	//Basler�̃J�����p�����[�^�ݒ�
	cam.setParam(paramTypeBasler::Param::ExposureTime, 2000.0f);
	cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	cam.setParam(paramTypeBasler::GrabStrategy::LatestOnlyFrame); //��Ƀo�b�t�@���X�V
	cam.setParam(paramTypeBasler::CaptureType::ColorGrab);

	cam.parameter_all_print();

	//�ϐ���`
	cv::Mat in_img = cv::Mat(height, width, CV_8UC1, cv::Scalar::all(255));
	bool flg = true;

	//�J�����N��
	cam.start();

	//�摜���X�V��������X���b�h
	thread thr(TakePicture, &cam, &in_img, &flg);

	while (1)
	{
		cv::imshow("img", in_img);
		int key = cv::waitKey(1);
		if (key == 'q')break;
	}
	flg = false;
	if (thr.joinable())thr.join();

	cam.stop();
	cam.disconnect();

	return 0;
}

//�X���b�h�֐�
void TakePicture(basler *cam, cv::Mat *img, bool *flg) {
	while (flg)
	{
		cam->captureFrame(img->data);
	}
}