#include <opencv2/opencv.hpp>
#include <HSC/KAYACoaXpressClass.hpp>
#include <vector>
#include <time.h>
#include <thread>
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/calib3d.hpp>

#ifdef _DEBUG
#define LIB_EXT "d.lib"
#else
#define LIB_EXT ".lib"
#endif


#pragma comment(lib,"KAYACoaXpressLib" LIB_EXT)
#pragma warning(disable:4996)
using namespace std;

//#define _OMNIDIR_CAM
//#define _FISHEYE_CAM

int main() {
	//内部パラメータcalibration用動画の読み取り
	string video_dir = "202009012005_video.mp4";
	cv::VideoCapture video;
	video.open(video_dir);
	if (!video.isOpened()) {
		cout << "video cannot ne opened..." << endl;
		return 0;
	}

	//画像保存用バッファ
	vector<cv::Mat> calib_imgs;
	cv::Mat calib_img;
	while (true)
	{
		video >> calib_img;
		if (calib_img.empty()) break;
		calib_imgs.push_back(calib_img.clone());
	}

	//チェスボード検出
	cout << "Checkerboard Detects" << endl;
	vector<vector<cv::Point2f>> imgPoints;
	vector<cv::Point2f> corners;
	cv::Size PatternSize(6, 8);
	cv::Size imgsize;
	int step = 10;
	for (int i = 0; i<calib_imgs.size() ; i=i+step)
	{
		cv::Mat cal_img = calib_imgs[i];
		bool find = cv::findChessboardCorners(cal_img, PatternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (!find) cout << "No. " << to_string(i) << " img: " << "No Chekerboard" << endl;
		else
		{
			imgPoints.push_back(corners);
			imgsize = cal_img.size();
			cout << "No. " << to_string(i) << " img: " << "Found" << endl;
		}
	}

	//チェスボードの三次元座標
	cout << "Calc Chessboard 3D points" << endl;
	vector<vector<cv::Point3f>> objPoints;
	float checksize = 32.0;
	for (int i = 0; i < imgPoints.size(); i++) {
		vector<cv::Point3f> obj;
		for (int c = 0; c < PatternSize.height; c++) {
			for (int r = 0; r < PatternSize.width; r++) {
				float x = r * checksize; //mm
				float y = c * checksize; //mm
				float z = 0.0;
				obj.push_back(cv::Point3f(x, y, z));
			}
		}
		objPoints.push_back(obj);
	}

#ifdef _OMNIDIR_CAM
	//Omnidirキャリブレーション
	cout << "Omnidir Calibration" << endl;
	cv::Mat K, Xi, D, R, T, idx;
	string omnicalibfile = "./OmnidirCamCalibParams.xml";
	cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
	double omniRMS = cv::omnidir::calibrate(objPoints, imgPoints, imgsize, K, Xi, D, R, T, 0, critia, idx);
	cv::FileStorage omnixml(omnicalibfile, cv::FileStorage::WRITE);
	cv::write(omnixml, "RMS", omniRMS);
	cv::write(omnixml, "K", K);
	cv::write(omnixml, "Xi", Xi);
	cv::write(omnixml, "D", D);
	omnixml.release();

	//確認のUndistort
	cout << "Undistort" << endl;
	cv::Mat perspective, cylindrical, stereographic, longlati;
	cv::omnidir::undistortImage(calib_imgs[300], perspective, K, D, Xi, cv::omnidir::RECTIFY_PERSPECTIVE);
	/*cv::omnidir::undistortImage(calib_imgs[300], cylindrical, K, D, Xi, cv::omnidir::RECTIFY_CYLINDRICAL);
	cv::omnidir::undistortImage(calib_imgs[300], stereographic, K, D, Xi, cv::omnidir::RECTIFY_STEREOGRAPHIC);
	cv::omnidir::undistortImage(calib_imgs[300], longlati, K, D, Xi, cv::omnidir::RECTIFY_LONGLATI);*/

#elif defined(_FISHEYE_CAM)
	cout << "Fisheye Calibration" << endl;
	cv::Mat	K, D, R, T;
	string fisheyecalibfile = "./FisheyeCamCalibParams.xml";
	cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
	double fishRMS = cv::fisheye::calibrate(objPoints, imgPoints, imgsize, K, D, R, T, 0, critia);
	cv::FileStorage fishxml(fisheyecalibfile, cv::FileStorage::WRITE);
	cv::write(fishxml, "RMS", fishRMS);
	cv::write(fishxml, "K", K);
	cv::write(fishxml, "D", D);
	fishxml.release();

	//確認のUndistort
	cout << "Undistort" << endl;
	cv::Mat perspective, cylindrical, stereographic, longlati;
	cv::fisheye::undistortImage(calib_imgs[300], perspective, K, D);
	/*cv::omnidir::undistortImage(calib_imgs[300], cylindrical, K, D, Xi, cv::omnidir::RECTIFY_CYLINDRICAL);
	cv::omnidir::undistortImage(calib_imgs[300], stereographic, K, D, Xi, cv::omnidir::RECTIFY_STEREOGRAPHIC);
	cv::omnidir::undistortImage(calib_imgs[300], longlati, K, D, Xi, cv::omnidir::RECTIFY_LONGLATI);*/
#else
	cout << "Perspective Calibration" << endl;
	cv::Mat K, D, R, T;
	string perscalibfile = "./PerspectiveCamCalibParams.xml";
	double persRMS = cv::calibrateCamera(objPoints, imgPoints, imgsize, K, D, R, T);
	cv::FileStorage persxml(perscalibfile, cv::FileStorage::WRITE);
	cv::write(persxml, "RMS", persRMS);
	cv::write(persxml, "K", K);
	cv::write(persxml, "D", D);
	persxml.release();

#endif // _OMNIDIR_CAM
	return 0;
}