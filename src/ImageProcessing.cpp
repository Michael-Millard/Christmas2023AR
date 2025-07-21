#include "ImageProcessing.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

ImageProcessing::ImageProcessing(int checkerboardWidth_, int checkerboardHeight_, double checkerboardSize_)
	: innerCornerWidth{ checkerboardWidth_ - 1 }
	, innerCornerHeight{ checkerboardHeight_ - 1 }
	, checkerboardSize{ checkerboardSize_ }
{
	initParams();
}

ImageProcessing::~ImageProcessing()
{}

void ImageProcessing::initParams()
{
	camIntrins = cv::Mat(3, 3, CV_64FC1);
	rotMat = cv::Mat(3, 3, CV_64FC1);
	transMat = cv::Mat(3, 1, CV_64FC1);
	board6DoFPose = cv::Mat(3, 4, CV_64FC1);
	fullCamMatrix = cv::Mat(3, 4, CV_64FC1);

	brown = cv::Scalar(25, 50, 125);
	green = cv::Scalar(0, 255, 0);
	black = cv::Scalar(0, 0, 0);
	white = cv::Scalar(255, 255, 255);
	yellow = cv::Scalar(0, 255, 255);
}

void ImageProcessing::drawVillage(cv::Mat& currentFrame, cv::Mat& intrinsMat, cv::Mat& rVec, cv::Mat& tVec,
	std::vector<cv::Point2f>& imageCornerPoints)
{
	// Corners of inner corners
	topLeft = imageCornerPoints[0]; // Top left is first corner in vector
	topRight = imageCornerPoints[innerCornerHeight]; // Top right is 'width - 1'th corner
	bottomLeft = imageCornerPoints[(innerCornerHeight) * innerCornerWidth]; // Bottom left is first element of final row
	bottomRight = imageCornerPoints[imageCornerPoints.size() - 1]; // Bottom right is final corner in vector

	// Convert vecs to Mats
	cv::Rodrigues(rVec, rotMat);

	// Draw models onto frame, one by one (starting with objects that are in background)

	/*******************
		Phone Poles
	*******************/

	// Points sequence : 0 - bottom, 1 - top, 2 - TAL, 3 - TAR, 4 - BAL, 5 - BAR
	// Back left
	transMat = (cv::Mat)tVec + rotMat * translation(-checkerboardSize, -checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	phonePole1PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DPhonePolePoints, brown);
	// Connecting points
	// TL = size - 4, TR = size - 3, BL = size - 2, BR = size - 1
	int vecSize = phonePole1PixPoints.size();
	cv::line(currentFrame, phonePole1PixPoints[vecSize - 4], phonePole1PixPoints[vecSize - 3], brown, 2, 8);
	cv::line(currentFrame, phonePole1PixPoints[vecSize - 2], phonePole1PixPoints[vecSize - 1], brown, 2, 8);
	// Back right
	transMat = (cv::Mat)tVec + rotMat * translation(innerCornerWidth * checkerboardSize, -checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	phonePole2PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DPhonePolePoints, brown);
	// Connecting points
	cv::line(currentFrame, phonePole2PixPoints[vecSize - 4], phonePole2PixPoints[vecSize - 3], brown, 2, 8);
	cv::line(currentFrame, phonePole2PixPoints[vecSize - 2], phonePole2PixPoints[vecSize - 1], brown, 2, 8);
	// Front left
	transMat = (cv::Mat)tVec + rotMat * translation(-checkerboardSize, innerCornerHeight * checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	phonePole3PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DPhonePolePoints, brown);
	// Connecting points
	cv::line(currentFrame, phonePole3PixPoints[vecSize - 4], phonePole3PixPoints[vecSize - 3], brown, 2, 8);
	cv::line(currentFrame, phonePole3PixPoints[vecSize - 2], phonePole3PixPoints[vecSize - 1], brown, 2, 8);
	// Front right
	transMat = (cv::Mat)tVec + rotMat * translation(innerCornerWidth * checkerboardSize, innerCornerHeight * checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	phonePole4PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DPhonePolePoints, brown);
	// Connecting points
	cv::line(currentFrame, phonePole4PixPoints[vecSize - 4], phonePole4PixPoints[vecSize - 3], brown, 2, 8);
	cv::line(currentFrame, phonePole4PixPoints[vecSize - 2], phonePole4PixPoints[vecSize - 1], brown, 2, 8);

	/**************************
		Fences
	**************************/

	// Back fence
	cv::line(currentFrame, phonePole1PixPoints[5], phonePole2PixPoints[5], brown, 2, 8);
	cv::line(currentFrame, phonePole1PixPoints[15], phonePole2PixPoints[15], brown, 2, 8);

	// Left fence
	cv::line(currentFrame, phonePole1PixPoints[5], phonePole3PixPoints[5], brown, 2, 8);
	cv::line(currentFrame, phonePole1PixPoints[15], phonePole3PixPoints[15], brown, 2, 8);

	// Right fence
	cv::line(currentFrame, phonePole2PixPoints[5], phonePole4PixPoints[5], brown, 2, 8);
	cv::line(currentFrame, phonePole2PixPoints[15], phonePole4PixPoints[15], brown, 2, 8);

	/**************************
		Phone lines
	**************************/

	// Back left to back right pole
	transMat = (cv::Mat)tVec + rotMat * translation(-checkerboardSize, -checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	phoneLine1PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DHorizPhoneLinePoints, green);

	// Back left to front left pole
	transMat = (cv::Mat)tVec + rotMat * translation(-checkerboardSize, -checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	phoneLine2PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DVertPhoneLinePoints, green);

	// Back right to front right pole
	transMat = (cv::Mat)tVec + rotMat * translation((innerCornerWidth + 1) * checkerboardSize, -checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	phoneLine2PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DVertPhoneLinePoints, green);

	/*************************************
		Trees
	*************************************/
	// Top left corner
	transMat = (cv::Mat)tVec + rotMat * translation(0, 0, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	tree1PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DTreePoints, green);
	// Top right corner
	transMat = (cv::Mat)tVec + rotMat * translation((innerCornerWidth - 1) * checkerboardSize, 0, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	tree2PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DTreePoints, green);
	// Bottom left corner
	transMat = (cv::Mat)tVec + rotMat * translation(0, (innerCornerHeight - 1) * checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	tree3PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DTreePoints, green);
	// Bottom right
	transMat = (cv::Mat)tVec + rotMat * translation((innerCornerWidth - 1) * checkerboardSize, (innerCornerHeight - 1) * checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	tree4PixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DTreePoints, green);

	/*************************************
		House model
	*************************************/

	// Walls
	transMat = (cv::Mat)tVec + rotMat * translation(7 * checkerboardSize, 5 * checkerboardSize, 0);
	cv::hconcat(rotMat, -transMat, board6DoFPose);
	fullCamMatrix = intrinsMat * board6DoFPose;
	housePixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DHousePoints, green);

	// Roof
	roofPixPoints = drawModel(currentFrame, fullCamMatrix, inhomog3DRoofPoints, green);
	// Connecting corners to roof center
	cv::line(currentFrame, roofPixPoints[0], roofPixPoints[roofPixPoints.size() - 1], green, 2, 8);
	cv::line(currentFrame, roofPixPoints[1], roofPixPoints[roofPixPoints.size() - 1], green, 2, 8);
	cv::line(currentFrame, roofPixPoints[2], roofPixPoints[roofPixPoints.size() - 1], green, 2, 8);
	cv::line(currentFrame, roofPixPoints[3], roofPixPoints[roofPixPoints.size() - 1], green, 2, 8);

	// Checking inner corners
	//cv::circle(currentFrame, topLeft, 0, cv::Scalar(0, 0, 255), 3, 8);
	//cv::circle(currentFrame, topRight, 0, cv::Scalar(0, 0, 255), 3, 8);
	//cv::circle(currentFrame, bottomLeft, 0, cv::Scalar(0, 0, 255), 3, 8);
	//cv::circle(currentFrame, bottomRight, 0, cv::Scalar(0, 0, 255), 3, 8);
}

void ImageProcessing::generateModels()
{
	/***************
	   Cone Model
	 **************/

	int numAngles = 60;
	int height = 2 * checkerboardSize; // mm
	int radius = 0.3 * checkerboardSize; // mm

	std::vector<double> zv = linspace(0.0, height - 1, height / 2);
	std::vector<double> thv = linspace(0.0, 360.0, numAngles);
	std::vector<double> rc = linspace(radius, 0, height / 2);

	cv::Point3f modelPoint;
	for (int i = 0; i < (int)zv.size(); i++) {
		for (int j = 0; j < (int)thv.size(); j++) {
			modelPoint.x = rc[i] * cos(thv[j] * CV_PI / 180);
			modelPoint.y = rc[i] * sin(thv[j] * CV_PI / 180);
			modelPoint.z = zv[i];
			inhomog3DTreePoints.push_back(modelPoint);
		}
	}

	/***************
		House
	***************/
	int width = 6 * checkerboardSize;
	int depth = 5 * checkerboardSize;
	height = 2 * checkerboardSize;
	int roofHeight = 2 * checkerboardSize; // Above normal height

	// Horiz lines
	for (int x = 0; x < width; x++)
	{
		// BTL to BTR
		inhomog3DHousePoints.push_back(cv::Point3f(x, 0, 0));
		// BBL to BBR
		inhomog3DHousePoints.push_back(cv::Point3f(x, depth, 0));
		// TTL to TTR
		inhomog3DHousePoints.push_back(cv::Point3f(x, 0, height));
		// TBL to TBR
		inhomog3DHousePoints.push_back(cv::Point3f(x, depth, height));
	}
	// Vert lines
	for (int y = 0; y < depth; y++)
	{
		// BTL to BBL
		inhomog3DHousePoints.push_back(cv::Point3f(0, y, 0));
		// BBL to BBR
		inhomog3DHousePoints.push_back(cv::Point3f(width, y, 0));
		// TTL to TBL
		inhomog3DHousePoints.push_back(cv::Point3f(0, y, height));
		// TBL to TBR
		inhomog3DHousePoints.push_back(cv::Point3f(width, y, height));
	}
	// Perpendicular lines
	for (int h = 0; h < height; h++)
	{
		// BTL to TTL
		inhomog3DHousePoints.push_back(cv::Point3f(0, 0, h));
		// BTR to TTR
		inhomog3DHousePoints.push_back(cv::Point3f(width, 0, h));
		// BBL to TBL
		inhomog3DHousePoints.push_back(cv::Point3f(0, depth, h));
		// BBR to TBR
		inhomog3DHousePoints.push_back(cv::Point3f(width, depth, h));
	}
	// Drawing roof using opencv lines for now
	inhomog3DRoofPoints.push_back(cv::Point3f(0, 0, height)); // TTL
	inhomog3DRoofPoints.push_back(cv::Point3f(width, 0, height)); // TTR
	inhomog3DRoofPoints.push_back(cv::Point3f(0, depth, height)); // TBL
	inhomog3DRoofPoints.push_back(cv::Point3f(width, depth, height)); // TBR
	inhomog3DRoofPoints.push_back(cv::Point3f(width / 2, depth / 2, height + roofHeight)); // Centre

	// Telephone poles
	height = 3 * checkerboardSize;
	width = checkerboardSize;
	// Need to insert vertical points manually to allow easy fence implementation
	for (int z = 0; z < height + 1; z++)
	{
		inhomog3DPhonePolePoints.push_back(cv::Point3f(0, 0, z)); // Bottom
	}
	// The rest can just be added and connecting using cv::line
	inhomog3DPhonePolePoints.push_back(cv::Point3f(-0.5 * width, 0, 0.9 * height)); // Top arm left
	inhomog3DPhonePolePoints.push_back(cv::Point3f(0.5 * width, 0, 0.9 * height)); // Top arm right
	inhomog3DPhonePolePoints.push_back(cv::Point3f(-0.5 * width, 0, 0.75 * height)); // Bottom arm left
	inhomog3DPhonePolePoints.push_back(cv::Point3f(0.5 * width, 0, 0.75 * height)); // Bottom arm right

	// Phone lines between poles
	// Horizontal: y-coord is fixed, x-z parabola (0.9*height it top arm height)
	// z = a(x - p)^2 + q, p = xDist/2, q = 0.7*height, a = (0.8*height)/(xDist^2)
	double xDist = (innerCornerWidth + 1) * checkerboardSize;
	for (double x = 0.5 * width; x < (xDist - 0.5 * width); x++)
	{
		double z = ((0.8 * height) / (std::pow(xDist, 2))) * std::pow((x - xDist / 2), 2) + 0.7 * height;
		inhomog3DHorizPhoneLinePoints.push_back(cv::Point3f(-x, 0, z));
	}
	// Vertical: x-coord is fixed, y-z parabola
	double yDist = (innerCornerHeight + 1) * checkerboardSize;
	for (double y = 0; y < yDist; y++)
	{
		double z = ((0.8 * height) / (std::pow(yDist, 2))) * std::pow((y - yDist / 2), 2) + 0.7 * height;
		inhomog3DVertPhoneLinePoints.push_back(cv::Point3f(0.5 * width, -y, z));
	}
}

std::vector<cv::Point2f> ImageProcessing::drawModel(cv::Mat& currentFrame, cv::Mat& camMatrix, std::vector<cv::Point3f>& inhomog3DModelPoints, cv::Scalar& colour)
{
	// Homogeneous model coords in world space [Xw Yw Zw 1]^T
	std::vector<cv::Mat> homog3DModelPoints = getHomogeneousPoints(inhomog3DModelPoints);
	// Homogeneous and inhomogenous pixel points
	cv::Point2f pixPoint;
	cv::Mat homogPixPoint;
	std::vector<cv::Point2f> pixPoints;
	for (int i = 0; i < (int)homog3DModelPoints.size(); i++) {
		// Convert to homogeneous pixel point
		homogPixPoint = camMatrix * homog3DModelPoints[i];
		// Convert to inhomogenous pixel point
		pixPoint = homogToInhomog2D(homogPixPoint);
		pixPoints.push_back(pixPoint);
		// Draw pixel point
		cv::circle(currentFrame, pixPoint, 0, colour, 2, 8);
	}

	return pixPoints;
}

std::vector<double> ImageProcessing::linspace(double start, double end, int numElems)
{

	std::vector<double> linspaced;

	if (numElems == 0)
	{
		return linspaced;
	}
	if (numElems == 1)
	{
		linspaced.push_back(start);
		return linspaced;
	}

	double delta = (end - start) / numElems;

	for (int i = 0; i < numElems - 1; i++)
	{
		linspaced.push_back(start + delta * i);
	}
	linspaced.push_back(end);

	return linspaced;
}

std::vector<cv::Mat> ImageProcessing::getHomogeneousPoints(std::vector<cv::Point3f>& inhomog3DModelPoints)
{
	cv::Mat homog3DModelPoint;
	std::vector<cv::Mat> homog3DModelPoints;
	for (int i = 0; i < (int)inhomog3DModelPoints.size(); i++) {
		homog3DModelPoint = inhomogToHomog(inhomog3DModelPoints[i]);
		homog3DModelPoints.push_back(homog3DModelPoint);
	}
	return homog3DModelPoints;
}

cv::Mat ImageProcessing::inhomogToHomog(cv::Point3f& point3D)
{
	return (cv::Mat_<double>(4, 1) << point3D.x, point3D.y, point3D.z, 1);
}

cv::Point2f ImageProcessing::homogToInhomog2D(cv::Mat& homog2DMat)
{
	return cv::Point2f(homog2DMat.at<double>(0, 0) / homog2DMat.at<double>(2, 0),
		homog2DMat.at<double>(1, 0) / homog2DMat.at<double>(2, 0));
}

cv::Mat ImageProcessing::rotation(double thetaX, double thetaY, double thetaZ)
{
	// Rotation matrices about x, y, and z axes are all 3x3 square matrices of 32-bit doubles
	cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0,
		0, cosf(thetaX), -sinf(thetaX),
		0, sinf(thetaX), cosf(thetaX));

	cv::Mat rotY = (cv::Mat_<double>(3, 3) << cosf(thetaY), 0, sinf(thetaY),
		0, 1, 0,
		-sinf(thetaY), 0, cosf(thetaY));

	cv::Mat rotZ = (cv::Mat_<double>(3, 3) << cosf(thetaZ), -sinf(thetaZ), 0,
		sinf(thetaZ), cosf(thetaZ), 0,
		0, 0, 1);

	// Overall rotation matrix (multiplication order: x x y x z)
	return rotZ * rotY * rotX;
}

cv::Mat ImageProcessing::translation(double tX, double tY, double tZ)
{
	return (cv::Mat_<double>(3, 1) << tX, tY, tZ);
}