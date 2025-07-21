#include "CameraCalibrator.h"
#include "ImageProcessing.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char* argv[])
{
	// Parse command line arguments
	if (argc < 7) 
	{
		std::cerr << "Usage: " << argv[0] << " <checkerboard_width> <checkerboard_height> <checkerboard_size_mm> <calibrate_camera:0|1> <num_calib_images> <webcam_name>\n";
		return EXIT_FAILURE;
	}

	int checkerboardWidth = std::stoi(argv[1]);
	int checkerboardHeight = std::stoi(argv[2]);
	double checkerboardSize = std::stod(argv[3]);
	bool doCalibration = std::stoi(argv[4]) != 0;
	int numCalibImages = std::stoi(argv[5]);
	std::string webcamName = argv[6];
	
	// Get needed objects
	std::unique_ptr<CameraCalibrator> calibObj = std::make_unique<CameraCalibrator>(checkerboardWidth, checkerboardHeight, checkerboardSize);
	std::vector<cv::Point3f> worldCornerPoints = calibObj->getWorldCornerPoints();
	cv::Mat intrinsMatrix, distCoeffs;
	if (doCalibration)
	{
		// Calibrate camera, print the results and write them to an XML file
		calibObj->captureCalibrationImages(numCalibImages, webcamName);
		calibObj->printResults();
		calibObj->saveCameraParams();
		intrinsMatrix = calibObj->getIntrinsMatrix();
		distCoeffs = calibObj->getDistCoeffs();
	}
	else
	{
		// Read in camera parameters from XML file
		std::cout << "Reading in camera params from XML file...\n";
		cv::FileStorage camParams("./CameraParams/cameraParams.xml", cv::FileStorage::READ);
		camParams["IntrinsMatrix"] >> intrinsMatrix;
		camParams["DistCoeffs"] >> distCoeffs;
		// Need to set these params in the calib obj
		calibObj->setIntrinsMatix(intrinsMatrix);
		calibObj->setDistCoeffs(distCoeffs);
		std::cout << intrinsMatrix << "\n";
		std::cout << distCoeffs << "\n";
	}

	// Generate models to be projected before looping
	std::unique_ptr<ImageProcessing> imgProcObj = std::make_unique<ImageProcessing>(checkerboardWidth, checkerboardHeight, checkerboardSize);
	imgProcObj->generateModels();

	cv::VideoCapture webCam(webcamName);
	cv::Mat currentFrame;
	
	// Constantly updating params
	cv::Mat rVec, tVec; 
	std::vector<cv::Point2f> imageCornerPoints;

	// Corners found bool
	bool cornersFound = false;
	
	// Window
	int screenWidth = 1920, screenHeight = 1080;
	webCam.read(currentFrame); // Giving initial frame to set window position
	cv::namedWindow("Christmas Village");
	cv::moveWindow("Christmas Village", (screenWidth - currentFrame.cols)/2, (screenHeight - currentFrame.rows)/2);

	// Check if webcam opened successfully
	if (!webCam.isOpened())
	{
		std::cout << "Could not open webcam.\n";
		return EXIT_FAILURE;
	}

	else
	{
		bool exit = false;
		while (!exit) // Loop until 'x' hit on keyboard
		{
			webCam >> currentFrame;

			cornersFound = calibObj->FindImageCorners(currentFrame);
			if (cornersFound) // Draw the village
			{
				// Get calib params
				rVec = calibObj->getRVec();
				tVec = calibObj->getTVec();
				imageCornerPoints = calibObj->getImageCornerPoints();
				
				// Draw village
				imgProcObj->drawVillage(currentFrame, intrinsMatrix, rVec, tVec, imageCornerPoints);
			}

			// Display image, with or without village
			cv::imshow("Christmas Village", currentFrame);

			// Wait 30 milliseconds, then loop
			char c = (char)cv::waitKey(30);
			if (c == 'x')
			{
				exit = true;
			}
		}
	}

	return EXIT_SUCCESS;
}