#pragma once
#ifndef PARAMETER_H_
#define PARAMETER_H_

#include <opencv2/opencv.hpp>

struct LSM
{
	//魚眼レンズ内部パラメータ
	double map_coefficient[4];
	double stretch_mat[4];
	double distortion[2];

	//レーザ平面に関するパラメータ
	double pa[10], pb[10], pc[10];
	double ref_center[2];
	double ref_radius, ref_arcwidth;
	double plane_nml[3];

	//画像類
	cv::Mat in_img;
	cv::Mat mask_refarc;
	cv::Mat mask_lsm;
	cv::Mat ref_arc;
	cv::Mat lsm_laser;

	//パラメータ
	int processcnt;
	double rp[2];
};

#endif // !PARAMETER_H_
