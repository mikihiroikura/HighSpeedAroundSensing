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

//�O���[�o���ϐ�
cv::Mat in_img;

int main() {
	//�p�����[�^
	bool flg = true;

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
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeKAYACoaXpress::paramFloat::ExposureTime, exposuretime);
	cam.setParam(paramTypeKAYACoaXpress::Gain::x1);
	cam.parameter_all_print();

	//���[�UCalibration�̌��ʂ̌Ăяo��


	//�J�����N��
	cam.start();

	//Thread�̍쐬
	/// 1000fps�ŉ摜���i�[��������X���b�h
	/// ���݂̉摜��PC�ɏo�͂��Č�����悤����X���b�h
	/// AR�}�[�J�����o���ʒu�p�����v�Z����X���b�h
	
	//���C�����[�v
	/// �擾���ꂽ�摜������ؒf�@�ŎO�����ʒu���v�Z����
	while (flg)
	{

	}

	//�X���b�h�̒�~

	//�v�Z�������W�C�擾�摜�̕ۑ�

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