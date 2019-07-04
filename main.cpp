#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define IMAGE_WIDTH  1920
#define IMAGE_HEIGHT 1080

#define CHECKERBOARD_WIDTH  6
#define CHECKERBOARD_HEIGHT 8

#define CALIB_IMG_SAMPLE_SIZE 15

using namespace std;

int image_count = 1;
bool checkerboard_detected = false;

cv::Mat raw_image;

        vector<string> m_filenames;
        Size m_borderSize;
        vector<vector<Point2f>> m_srcPoints;
        vector<vector<Point3f>> m_dstPoints;
        Size imageSize;
        Mat cameraMatrix, distCoeffs;
        vector<Mat> rvecs, tvecs;

void setFilename()
{
	m_filenames.clear();
	m_filenames.push_back("/tmp/intrinsic_1920x1080_1.jpg"); 
	m_filenames.push_back("/tmp/intrinsic_1920x1080_2.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_3.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_4.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_5.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_6.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_7.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_8.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_9.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_10.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_12.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_12.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_13.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_14.jpg");
	m_filenames.push_back("/tmp/intrinsic_1920x1080_15.jpg");
} 

void setBorderSize(const Size &borderSize){ 
    m_borderSize = borderSize; 
} 

void addPoints(const vector<Point2f> &srcCorners,
				 const vector<Point3f> &dstCorners)
{
	m_srcPoints.push_back(srcCorners);
	m_dstPoints.push_back(dstCorners);
} 

void addChessboardPoints()
{ 
	vector<Point2f> srcCandidateCorners; 
	vector<Point3f> dstCandidateCorners; 
	for(int i=0; i<m_borderSize.height; i++){ 
		for(int j=0; j<m_borderSize.width; j++){ 
			dstCandidateCorners.push_back(Point3f(i, j, 0.0f)); 
		} 
	} 

	for(int i=0; i<m_filenames.size(); i++){ 
		Mat image=imread(m_filenames[i],CV_LOAD_IMAGE_GRAYSCALE); 

		imageSize = image.size();

		findChessboardCorners(image, m_borderSize, srcCandidateCorners); 
		TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
		cornerSubPix(image, srcCandidateCorners, Size(5,5), Size(-1,-1), param);  
		if(srcCandidateCorners.size() == m_borderSize.area()){ 
			addPoints(srcCandidateCorners, dstCandidateCorners); 
		} 
	} 
} 

bool visualize_checkerboard(Mat &raw_image, Mat &chessboard_visualized_image, Size border_size)
{
	Mat gray_image;
	cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);

	vector<Point2f> corners;	

	bool find = findChessboardCorners(gray_image, border_size, corners);

	if(find == true) {
		raw_image.copyTo(chessboard_visualized_image);

		TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
		cornerSubPix(gray_image, corners, Size(5,5), Size(-1,-1), param);
		cv::drawChessboardCorners(chessboard_visualized_image, border_size, corners, find);
		return true;
	} else {
		return false;
	}
}

void calibrate()
{
	calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs);  
}

void undistort_image(const Mat &src, Mat &dst)
{
	Mat map1, map2; 

	calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs);  
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(),
				imageSize, CV_32F, map1, map2); 
	remap(src, dst, map1, map2, INTER_LINEAR); 
}

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

		Mat chessboard_visualized_image;

		if(visualize_checkerboard(raw_image, chessboard_visualized_image, Size(CHECKERBOARD_WIDTH, CHECKERBOARD_HEIGHT)) == true) {
			cv::imshow("intrinsic calibration", chessboard_visualized_image);
			checkerboard_detected = true;
		} else {
			checkerboard_detected = false;
			cv::imshow("intrinsic calibration", raw_image);
			checkerboard_detected = false;
		}
		waitKey(30);
	}

	cout << "start estimating extrinsicing parameters...\n";

	setFilename(); 
	setBorderSize(Size(CHECKERBOARD_WIDTH,CHECKERBOARD_HEIGHT));
	addChessboardPoints();
	calibrate();

	while(1) {
		camera >> raw_image;

		undistort_image(raw_image, undistorted_image);
		imshow("intrinsic calibration", undistorted_image);
		cv::waitKey(30);
	}

	destroyAllWindows();

	return 0;
}
