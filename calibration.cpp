#include "calibration.hpp"

void CameraCalibrator::setFilename()
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

void CameraCalibrator::setBorderSize(const Size &borderSize){ 
    m_borderSize = borderSize; 
} 

void CameraCalibrator::addChessboardPoints()
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

void CameraCalibrator::addPoints(const vector<Point2f> &srcCorners,
				 const vector<Point3f> &dstCorners)
{
	m_srcPoints.push_back(srcCorners);
	m_dstPoints.push_back(dstCorners);
} 

void CameraCalibrator::calibrate()
{
	calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs);  
}

void CameraCalibrator::undistort_image(const Mat &src, Mat &dst)
{
	Mat map1, map2; 

	calibrateCamera(m_dstPoints, m_srcPoints, imageSize, cameraMatrix,
			distCoeffs, rvecs, tvecs);  
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(),
				imageSize, CV_32F, map1, map2); 
	remap(src, dst, map1, map2, INTER_LINEAR); 
}
