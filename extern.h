#ifndef _EXTERN_H_
#define _EXTERN_H_

#include "ChamferMatchingData.h"

/*	OpenCV headers */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


extern "C" void read_in_image(char* filename, CMResult_t * cm_result);
extern "C" void read_cvmat(CvMat * cvmat_in, CMResult_t * cm_result);
extern void findplate(cv::Mat & image, CMResult_t * cm_result);

std::ostream & operator << (std::ostream & os, CMResult_t & result)
{
	return os << "X = " << result.x << ", Y = " << result.y << std::endl
			<< "Width = " << result.width << ", Height = " << result.height << std::endl
			<< "Angle = " << result.angle << ", Scale = " << result.scale << ", Cost = " << result.cost << std::endl; 
}

#endif
