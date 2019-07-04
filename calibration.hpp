#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

class CameraCalibrator {
private:
	vector<string> m_filenames;
	Size m_borderSize;
	vector<vector<Point2f>> m_srcPoints;
	vector<vector<Point3f>> m_dstPoints;
	Size imageSize;
	Mat cameraMatrix, distCoeffs;
	vector<Mat> rvecs, tvecs;

public:
	void setFilename();
	void setBorderSize(const Size &borderSize);
	void addChessboardPoints();
	void addPoints(const vector<Point2f> &imageCorners,
		       const vector<Point3f> &objectCorners);
	void calibrate();
	void undistort_image(const Mat &src, Mat &dst);
};
