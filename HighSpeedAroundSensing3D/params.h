#pragma once
#ifndef PARAMETER_H_
#define PARAMETER_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include "RS232c.h"

using namespace std;

struct LSM
{
	//魚眼レンズ内部パラメータ
	double map_coefficient[4];
	double stretch_mat[4];
	double det;
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
	cv::Mat ref_arc_ranged;
	cv::Mat lsm_laser;
	cv::Mat lsm_laser_ranged;

	//画像ポインタ
	uint8_t* ref_arc_src;
	uint8_t* lsm_laser_src;


	//パラメータ
	long long processcnt;
	int buffersize;
	float rp[2];
	vector<cv::Point> bps;
	vector<cv::Point> allbps;
	vector<cv::Point2d> idpixs;
	vector<vector<double>> campts;
	vector<bool> processflgs;
	cv::Point2d bp;
};

struct Logs
{
	vector<vector<vector<double>>> LSM_pts;
	//double LSM_pts_cycle[cyclebuffersize][rends - rstart][3] = {0};
	double* LSM_pts_cycle;
	double* LSM_pts_logs;
	char* LSM_rotdir;
	double* LSM_times;
	bool* LSM_detectedenableflg;
	bool* LSM_objdetectedflg;
	int* LSM_reciprocntdown;
	int* LSM_alertcnt;
	int* LSM_dangercnt;
	int* LSM_rpm;
	double* LSM_laserplane_nml;
	float* LSM_rps;
	bool* LSM_rotmodes;
	vector<cv::Mat> in_imgs_log;
	cv::Mat* in_imgs_log_ptr;
};

#endif // !PARAMETER_H_
