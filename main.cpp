#include <stdio.h>
#include <opencv2/opencv.hpp>

#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480

#define BOARD_WIDTH  6
#define BOARD_HEIGHT 8

#define IMG_SAMPLE_SIZE 15

using namespace std;
using namespace cv;

/* parameters of the camera calibrator */
vector<string> image_names;
//board size and image_size
Size board_size = Size(BOARD_WIDTH, BOARD_HEIGHT);
Size image_size = Size(IMAGE_WIDTH, IMAGE_HEIGHT);
vector<vector<Point2f>> image_2d_points;
vector<vector<Point3f>> object_3d_points;
//intrinsic parameters
Mat camera_matrix, dist_coeffs;
vector<Mat> rvecs, tvecs;

int image_count = 1;
bool checkerboard_detected = false;

cv::Mat raw_image;

void set_img_filenames(void)
{
	image_names.clear();

	char img_path[100] = "";

	for(int i = 1; i <= IMG_SAMPLE_SIZE; i++) {
		sprintf(img_path, "/tmp/intrinsic_%dx%d_%d.jpg", IMAGE_WIDTH,
		        IMAGE_HEIGHT, i);
		image_names.push_back(img_path); 
	}
} 

void add_board_points(vector<Point2f> &_2d_corners, vector<Point3f> &_3d_corners)
{
	image_2d_points.push_back(_2d_corners);
	object_3d_points.push_back(_3d_corners);
} 

bool visualize_checkerboard(Mat &raw_image, Mat &board_visualized_img)
{
	Mat gray_image;
	cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);

	vector<Point2f> corners;	

	bool found = findChessboardCorners(gray_image, board_size, corners);

	if(found == true) {
		raw_image.copyTo(board_visualized_img);

		TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
		cornerSubPix(gray_image, corners, Size(5,5), Size(-1,-1), param);
		cv::drawChessboardCorners(board_visualized_img, board_size, corners, found);
		return true;
	} else {
		return false;
	}
}

void estimate_intrinsic_parameters()
{
	set_img_filenames();

	vector<Point2f> _2d_corners;
	vector<Point3f> _3d_corners;

	for(int i = 0; i < board_size.height; i++) {
		for(int j = 0; j < board_size.width; j++){
			_3d_corners.push_back(Point3f(i, j, 0.0f));
		}
	}

	for(int i = 0; i < image_names.size(); i++) {
		Mat image = imread(image_names[i], CV_LOAD_IMAGE_GRAYSCALE); 

		findChessboardCorners(image, board_size, _2d_corners); 
		TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
		cornerSubPix(image, _2d_corners, Size(5, 5), Size(-1, -1), param);  
		if(_2d_corners.size() == board_size.area()) { 
			add_board_points(_2d_corners, _3d_corners); 
		} 
	} 

	calibrateCamera(object_3d_points, image_2d_points, image_size,
			camera_matrix, dist_coeffs, rvecs, tvecs);  
}

void undistort_image(const Mat &src, Mat &dst)
{
	Mat map1, map2; 

	calibrateCamera(object_3d_points, image_2d_points, image_size, camera_matrix,
			dist_coeffs, rvecs, tvecs);  
	initUndistortRectifyMap(camera_matrix, dist_coeffs, Mat(), Mat(),
				image_size, CV_32F, map1, map2); 
	remap(src, dst, map1, map2, INTER_LINEAR); 
}

void on_click_callback(int event, int x, int y, int flags, void* param)
{
	char img_save_path[100] = "";

	if((event == CV_EVENT_LBUTTONDOWN) && (image_count <= IMG_SAMPLE_SIZE)) {
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
	camera.set(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);

	if(camera.isOpened() == false) {
		return 0;
	}

	namedWindow("intrinsic calibration");
	setMouseCallback("intrinsic calibration", on_click_callback, NULL);

	cout << "please click the window to save 15 images for calibration.\n";

	Mat undistorted_image;
	vector<Point2f> corners;

	while(image_count <= 15) {
		camera >> raw_image;

		Mat board_visualized_img;

		if(visualize_checkerboard(raw_image, board_visualized_img) == true) {
			cv::imshow("intrinsic calibration", board_visualized_img);
			checkerboard_detected = true;
		} else {
			checkerboard_detected = false;
			cv::imshow("intrinsic calibration", raw_image);
			checkerboard_detected = false;
		}
		waitKey(30);
	}

	cout << "start estimating extrinsicing parameters...\n";
	estimate_intrinsic_parameters();

	while(1) {
		camera >> raw_image;

		undistort_image(raw_image, undistorted_image);
		imshow("intrinsic calibration", undistorted_image);
		cv::waitKey(30);
	}

	destroyAllWindows();
	return 0;
}
