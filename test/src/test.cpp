// aruco_code_detect.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//


#include <iostream>  
#include <opencv4/opencv2/core/core.hpp>  
#include<opencv4/opencv2/highgui/highgui.hpp>  
#include <opencv4/opencv2/aruco.hpp>
#include "opencv4/opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

void maker_test(Mat image)
{

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	cv::Ptr<cv::aruco::DetectorParameters> params = aruco::DetectorParameters::create();
	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

	cv::Mat imageCopy;

	image.copyTo(imageCopy);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners, rejected;
	cv::aruco::detectMarkers(image, dictionary, corners, ids, params);
	

	// if at least one marker detected
	if (ids.size() > 0) 
	{
		cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
		cv::imshow("test", imageCopy);
		cv::waitKey(0);
	}
	else{
		cout << "No marker" <<endl;
	}

}
int main()
{
	Mat img = cv::imread("../test.jpg", 1);
	if(!img.empty())
	{
	maker_test(img);	
	}
	else
	{
		cout << "could not open img\n"; 
	}
}

