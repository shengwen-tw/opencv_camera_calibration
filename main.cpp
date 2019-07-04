#include "calibration.hpp"
#include <stdio.h>

#define IMAGE_WIDTH  1920
#define IMAGE_HEIGHT 1080

#define CHECKERBOARD_WIDTH  8
#define CHECKERBOARD_HEIGHT 6

#define CALIB_IMG_SAMPLE_SIZE 15

using namespace std;

int image_count = 1;

cv::Mat frame;

void on_click_callback(int event, int x, int y, int flags, void* param)
{
	char img_save_path[100] = "";

	if((event == CV_EVENT_LBUTTONDOWN) && (image_count <= CALIB_IMG_SAMPLE_SIZE)) {
		sprintf(img_save_path, "/tmp/intrinsic_%dx%d_%d.jpg", IMAGE_WIDTH,
		        IMAGE_HEIGHT, image_count);
		imwrite(img_save_path, frame);
		cout << "saving image to " << img_save_path << "\n";
		image_count++;
	}
}

int main()
{ 
	cv::VideoCapture camera(0);

	if(camera.isOpened() == false) {
		return 0;
	}

	cout << "please click the window to save 15 images for calibration.\n";

	namedWindow("intrinsic calibration");
	setMouseCallback("intrinsic calibration", on_click_callback, NULL);

	while(image_count <= 15) {
		camera >> frame;
		cv::imshow("intrinsic calibration", frame);
		waitKey(30);
	}

	cout << "start estimating extrinsicing parameters...\n";

	CameraCalibrator myCameraCalibrator; 
	myCameraCalibrator.setFilename(); 
	myCameraCalibrator.setBorderSize(Size(CHECKERBOARD_WIDTH,CHECKERBOARD_HEIGHT));
	myCameraCalibrator.addChessboardPoints();
	myCameraCalibrator.calibrate();

	Mat raw_image, undistorted_image;

	while(1) {
		camera >> raw_image;
		myCameraCalibrator.undistort_image(raw_image, undistorted_image);
		imshow("intrinsic calibration", undistorted_image);
		cv::waitKey(30);
	}

	destroyAllWindows();
	return 0;
}
