#include <iostream>

#include <ctime>
#include <cstdlib>
#include <cstdio>

#include "arm.hpp"

#include "ChamferMatchingData.h"

/*	OpenCV headers */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "extern.h"

void test_findplate(const char * filename)
{
	cv::Mat img = cv::imread(filename);
	CvMat cimg = img;
	
	CMResult_t result;
	findplate(img, &result);	

	std::cout << result << std::endl;

	cv::circle(img, cv::Point2i(result.x, result.y), 5, cv::Scalar(0, 0, 255), 2);
	cv::rectangle(img, 
		cv::Point2i(result.x - result.width / 2, result.y - result.height / 2),
		cv::Point2i(result.x + result.width / 2, result.y + result.height / 2), 
		cv::Scalar(0, 0, 255), 3);
	cv::imshow("img", img);
	cv::waitKey(0);
}
