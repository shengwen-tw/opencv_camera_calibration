#include "calibration.hpp"
#include <stdio.h>

#define IMAGE_WIDTH  1920
#define IMAGE_HEIGHT 1080

#define CHECKERBOARD_WIDTH  6
#define CHECKERBOARD_HEIGHT 8

#define CALIB_IMG_SAMPLE_SIZE 15

using namespace std;

int image_count = 1;
bool checkerboard_detected = false;

cv::Mat raw_image;

void on_click_callback(int event, int x, int y, int flags, void* param)
{
	char img_save_path[100] = "";

	if((event == CV_EVENT_LBUTTONDOWN) && (image_count <= CALIB_IMG_SAMPLE_SIZE)) {
		if(checkerboard_detected == true) {
			sprintf(img_save_path, "/tmp/intrinsic_%dx%d_%d.jpg", IMAGE_WIDTH,
			        IMAGE_HEIGHT, image_count);
			imwrite(img_save_path, raw_image);
			cout << "saving image to " << img_save_path << "\n";
			image_count++;
		} else {
			cout << "could not detect checkerboard on image.\n";
		}
	}
}

int main()
{ 
	cv::VideoCapture camera(0);
	Mat undistorted_image;

	if(camera.isOpened() == false) {
		return 0;
	}

	namedWindow("intrinsic calibration");
	setMouseCallback("intrinsic calibration", on_click_callback, NULL);

	cout << "please click the window to save 15 images for calibration.\n";

	vector<Point2f> corners;

	while(image_count <= 15) {
		camera >> raw_image;

		Mat chessboard_visialize_image;
		raw_image.copyTo(chessboard_visialize_image);

		Mat gray_image;
		cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);

		bool ret = findChessboardCorners(gray_image, 
			Size(CHECKERBOARD_WIDTH,CHECKERBOARD_HEIGHT), corners);

		if(ret == true) {
			TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
			cornerSubPix(gray_image, corners, Size(5,5), Size(-1,-1), param);
			cv::drawChessboardCorners(chessboard_visialize_image, Size(CHECKERBOARD_WIDTH,CHECKERBOARD_HEIGHT), corners, ret);
			checkerboard_detected = true;
			cv::imshow("intrinsic calibration", chessboard_visialize_image);
		} else {
			checkerboard_detected = false;
			cv::imshow("intrinsic calibration", raw_image);
		}
		waitKey(30);
	}

	cout << "start estimating extrinsicing parameters...\n";

	CameraCalibrator myCameraCalibrator; 
	myCameraCalibrator.setFilename(); 
	myCameraCalibrator.setBorderSize(Size(CHECKERBOARD_WIDTH,CHECKERBOARD_HEIGHT));
	myCameraCalibrator.addChessboardPoints();
	myCameraCalibrator.calibrate();

	while(1) {
		camera >> raw_image;

		myCameraCalibrator.undistort_image(raw_image, undistorted_image);
		imshow("intrinsic calibration", undistorted_image);
		cv::waitKey(30);
	}

	destroyAllWindows();

	return 0;
}
