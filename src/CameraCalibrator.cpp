#include "CameraCalibrator.h"

#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

CameraCalibrator::CameraCalibrator(int checkerboardWidth_, int checkerboardHeight_, double checkerboardSize_)
	: innerCornerWidth{ checkerboardWidth_ - 1 }
	, innerCornerHeight{ checkerboardHeight_ - 1 }
	, checkerboardSize{ checkerboardSize_ }
{
	intrinsMatrix = cv::Mat(3, 3, CV_32FC1);
	distCoeffs = cv::Mat(1, 5, CV_32FC1);
	reprojError = 0.0;

	// Initialize world inner corner points
	for (int i = 0; i < innerCornerHeight; i++) 
	{
		for (int j = 0; j < innerCornerWidth; j++) 
		{
			// z = 0 plane
			worldCornerCoords.push_back(cv::Point3f(
				checkerboardSize * j, 
				checkerboardSize * i, 
				0.0f));
		}
	}
}

cv::Mat CameraCalibrator::getIntrinsMatrix()
{
	return intrinsMatrix;
}

cv::Mat CameraCalibrator::getDistCoeffs()
{
	return distCoeffs;
}

cv::Mat CameraCalibrator::getRVec()
{
	return rVec;
}

cv::Mat CameraCalibrator::getTVec()
{
	return tVec;
}

std::vector<cv::Point2f> CameraCalibrator::getImageCornerPoints()
{
	return imageCornerCoords;
}

std::vector<cv::Point3f> CameraCalibrator::getWorldCornerPoints()
{
	return worldCornerCoords;
}

void CameraCalibrator::setIntrinsMatix(cv::Mat& intrinsMat)
{
	intrinsMatrix = intrinsMat;
}

void CameraCalibrator::setDistCoeffs(cv::Mat& distCoeffsMat)
{
	distCoeffs = distCoeffsMat;
}

bool CameraCalibrator::FindImageCorners(cv::Mat& currentFrame)
{
	// Creating Mat objects for images
	cv::Mat greyImage;
	// Vector to store the pixel coordinates of detected checker board corners (if found, appended to class image points)
	std::vector<cv::Point2f> cornerPts;
	// Boolean for if corners are detected in the image
	bool success = false;

	// Convert to greyscale (grey)
	cv::cvtColor(currentFrame, greyImage, cv::COLOR_BGR2GRAY);
	// Finding specified number of checkerboard inner corners 
	success = cv::findChessboardCorners(greyImage, cv::Size(innerCornerWidth, innerCornerHeight), cornerPts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |cv::CALIB_CB_NORMALIZE_IMAGE);

	if (success)
	{
		// Epsilon (accuracy) down to 0.1 of a pixel or max iteration count of 20 for termination criteria
		cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 0.1);
		// Refining pixel coordinates for given 2D points
		cv::cornerSubPix(greyImage, cornerPts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
		imageCornerCoords = cornerPts;
		// Get board pose (rVecs and tVecs)
		cv::solvePnP(worldCornerCoords, imageCornerCoords, intrinsMatrix, distCoeffs, rVec, tVec);
	}

	return success;
}

bool CameraCalibrator::captureCalibrationImages(int numImages, std::string webcamName)
{
	// Check for valid number of images
	if (numImages <= 0) {
		std::cerr << "Number of images must be greater than zero.\n";
		return false;
	}

	// Live webcam calibration
	cv::VideoCapture webCam(webcamName);
	if (!webCam.isOpened()) {
		std::cerr << "Failed to open webcam.\n";
		return false;
	}

	cv::Mat currentFrame;
	int imageTally = 0;
	std::string readPath = "./CalibImages/CalibImage";
	std::string writePath = readPath;
	std::cout << "Webcam calibration. Only use:\n\t'y' = save\n\t'r' = redo\n\t'x' = exit\n";
	while (imageTally < numImages)
	{
		webCam >> currentFrame;
		if (currentFrame.empty()) {
			std::cerr << "Failed to capture frame from webcam.\n";
			return false;
		}

		cv::imshow("Current Frame", currentFrame);
		char c = (char)cv::waitKey(30);

		switch (c)
		{
		case 'x': // Exit
			cv::destroyWindow("Current Frame");
			std::cout << "Camera calibration cancelled.\n";
			return false;

		case 'y': // Save image
			if (!cv::imwrite(writePath + std::to_string(imageTally + 1) + ".bmp", currentFrame)) {
				std::cerr << "Failed to save image.\n";
				return false;
			}
			std::cout << std::to_string(numImages - imageTally - 1) + " calibration images still to be captured.\n";
			imageTally++;
			break;

		case 'r': // Redo (delete last image)
			if (remove((writePath + std::to_string(imageTally + 1) + ".bmp").c_str()) == 0)
			{
				std::cout << "Previous calibration image deleted.\n";
			}
			else {
				std::cerr << "Failed to delete previous calibration image.\n";
				return false;
			}
			break;

		default:
			break;
		}
	}

	return calibrateCamera("./CalibImages/");
}

bool CameraCalibrator::calibrateCamera(std::string calibImagePath)
{
	// Calibration parameters
	cv::Mat greyImage;
	// Vector to store the pixel coordinates of detected checker board corners (if found, appended to class image points)
	std::vector<cv::Point2f> cornerPts;
	// Boolean for if corners are detected in the image
	bool success = false;

	std::cout << "Calibrating camera...\n";
	std::vector<std::string> calibImages = {};
	cv::glob(calibImagePath + "*.bmp", calibImages, false);
	int numImages = (int)calibImages.size();
	if (numImages <= 0) {
		std::cerr << "No calibration images found in the specified path.\n";
		return false;
	}
	else
		std::cout << "Found " << numImages << " calibration images.\n";

	cv::Mat currentFrame;
	// Epsilon (accuracy) down to 0.001 of a pixel or max iteration count of 30 for termination criteria
	cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
	for (int i = 0; i < numImages; i++)
	{
		currentFrame = cv::imread(calibImages[i], cv::IMREAD_UNCHANGED);
		cv::cvtColor(currentFrame, greyImage, cv::COLOR_BGR2GRAY);
		// Finding specified number of checkerboard inner corners 
		success = cv::findChessboardCorners(greyImage, cv::Size(innerCornerWidth, innerCornerHeight), cornerPts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

		if (success)
		{			
			// Refining pixel coordinates for given 2D points
			cv::cornerSubPix(greyImage, cornerPts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
			imageCornerPoints.push_back(cornerPts);
			worldCornerPoints.push_back(worldCornerCoords);
		}
	}
	reprojError = cv::calibrateCamera(worldCornerPoints, imageCornerPoints, cv::Size(greyImage.cols, greyImage.rows), intrinsMatrix, distCoeffs, rVecs, tVecs);

	std::cout << "Calibration complete. There were " << numImages - imageCornerPoints.size() << " failed calibration images.\n";

	return true;
}

void CameraCalibrator::printResults()
{
	// Print out the relevant camera parameters and reprojection 
	std::cout << "Camera Matrix:\n" << intrinsMatrix << std::endl;
	std::cout << "\nDistortion Coefficients:\n" << distCoeffs << std::endl;
	std::cout << "\nReprojection Error (on calibration 1 images):\n" << reprojError << std::endl;
}

void CameraCalibrator::saveCameraParams()
{
	// Write out the camera parameters and reprojection errors to an XML file
	// If file already exists, make a copy and write to new file
	// Make copy of current cam params if it exists
	std::ifstream inFile("./CameraParams/cameraParams.xml");
	std::ofstream outFile("./CameraParams/cameraParamsCopy.xml");
	outFile << inFile.rdbuf();
	inFile.close();
	outFile.close();

	// Delete old cam params file for next run
	if (remove("./CameraParams/cameraParams.xml") == 0) {
		std::cout << "\nSuccessfully deleted previous cameraParams.xml file. Writing current file...\n";
	}
	else {
		std::cout << "\nFailed to delete previous cameraParams.xml file.\n";
	}

	// Write to new file
	cv::FileStorage cameraParamsXML("./CameraParams/cameraParams.xml", cv::FileStorage::APPEND);
	cameraParamsXML << "IntrinsMatrix" << intrinsMatrix;
	cameraParamsXML << "DistCoeffs" << distCoeffs;
	cameraParamsXML << "ReprojectionError" << reprojError;

	std::cout << "\nSuccessfully saved camera parameters to XML file.\n";
}