#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

class CameraCalibrator
{
public:
	CameraCalibrator(int checkerboardWidth, int checkerboardHeight, double checkerboardSize);
	~CameraCalibrator() = default;
	bool captureCalibrationImages(int numImages, std::string webcamName);
	bool calibrateCamera(std::string calibImagePath);
	bool FindImageCorners(cv::Mat& currentFrame);
	void printResults();
	void saveCameraParams();
	cv::Mat getIntrinsMatrix();
	cv::Mat getDistCoeffs();
	cv::Mat getRVec();
	cv::Mat getTVec();
	std::vector<cv::Point2f> getImageCornerPoints();
	std::vector<cv::Point3f> getWorldCornerPoints();
	// Setters for when params read in from XML
	void setIntrinsMatix(cv::Mat& intrinsMat);
	void setDistCoeffs(cv::Mat& distCoeffsMat);

private:
	// Camera params
	cv::Mat intrinsMatrix;
	cv::Mat distCoeffs;
	double reprojError;

	// Calibration params
	int innerCornerWidth;
	int innerCornerHeight;
	double checkerboardSize;
	cv::Mat rVec, tVec;
	std::vector<cv::Mat> rVecs, tVecs;
	std::vector<cv::Point3f> worldCornerCoords; // Appended to world vector below everytime the calibration loops
	std::vector<cv::Point2f> imageCornerCoords;
	std::vector<std::vector<cv::Point3f>> worldCornerPoints; // These still need to be vectors of vectors to match camera calib function params
	std::vector<std::vector<cv::Point2f>> imageCornerPoints;
};

