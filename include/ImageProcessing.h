#pragma once

#include <vector>
#include <opencv2/core.hpp>

class ImageProcessing
{
public:
	ImageProcessing(int checkerboardWidth_, int checkerboardHeight_, double checkerboardSize_);
	~ImageProcessing() = default;
	void drawVillage(cv::Mat& currentFrame, cv::Mat& intrinsMat, cv::Mat& rVec, 
		cv::Mat& tVec, std::vector<cv::Point2f>& imageCornerPoints);
	void generateModels();

private:
	// Checkerboard params
	cv::Point2f topLeft;
	cv::Point2f topRight;
	cv::Point2f bottomLeft;
	cv::Point2f bottomRight;
	int innerCornerWidth;
	int innerCornerHeight;
	double checkerboardSize;

	// Cam params
	cv::Mat camIntrins;
	cv::Mat rotMat;
	cv::Mat transMat;
	cv::Mat board6DoFPose;
	cv::Mat fullCamMatrix;

	// Colours
	cv::Scalar green;
	cv::Scalar brown;
	cv::Scalar black;
	cv::Scalar white;
	cv::Scalar yellow;

	// Models

	// Tree
	std::vector<cv::Point3f> inhomog3DTreePoints;
	std::vector<cv::Point2f> tree1PixPoints;
	std::vector<cv::Point2f> tree2PixPoints;
	std::vector<cv::Point2f> tree3PixPoints;
	std::vector<cv::Point2f> tree4PixPoints;
	// House
	std::vector<cv::Point3f> inhomog3DHousePoints;
	std::vector<cv::Point2f> housePixPoints;
	std::vector<cv::Point3f> inhomog3DRoofPoints;
	std::vector<cv::Point2f> roofPixPoints;
	// Telephone poles
	std::vector<cv::Point3f> inhomog3DPhonePolePoints;
	std::vector<cv::Point2f> phonePole1PixPoints;
	std::vector<cv::Point2f> phonePole2PixPoints;
	std::vector<cv::Point2f> phonePole3PixPoints;
	std::vector<cv::Point2f> phonePole4PixPoints;
	// Actual lines between poles
	std::vector<cv::Point3f> inhomog3DHorizPhoneLinePoints;
	std::vector<cv::Point3f> inhomog3DVertPhoneLinePoints;
	std::vector<cv::Point2f> phoneLine1PixPoints;
	std::vector<cv::Point2f> phoneLine2PixPoints;
	std::vector<cv::Point2f> phoneLine3PixPoints;

	cv::Mat rotation(double thetaX, double thetaY, double thetaZ);
	cv::Mat translation(double tX, double tY, double tZ);
	std::vector<double> linspace(double start, double end, int numElems);
	std::vector<cv::Mat> getHomogeneousPoints(std::vector<cv::Point3f>& inhomog3DCylPoints);
	cv::Mat inhomogToHomog(cv::Point3f& point3D);
	cv::Point2f homogToInhomog2D(cv::Mat& homog2DMat);
	std::vector<cv::Point2f> drawModel(cv::Mat& currentFrame, cv::Mat& camMatrix, std::vector<cv::Point3f>& inhomog3DModelPoints, cv::Scalar& colour);
};
