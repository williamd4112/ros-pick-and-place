#include <iostream>

#include <ctime>
#include <cstdlib>
#include <cstdio>

#include <vector>

/*	OpenCV headers */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char * argv[])
{
	cv::Mat _img = cv::imread(argv[1]);
	cv::Mat img;
	cv::cvtColor(_img, img, CV_BGR2GRAY);
	cv::threshold(img, img, 0, 255, CV_THRESH_BINARY);	

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
  	std::vector<cv::RotatedRect> minRect( contours.size() );

  	for(int i = 0; i < contours.size(); i++ ) { 
		minRect[i] = cv::minAreaRect( cv::Mat(contours[i]) );
	}

  	for( int i = 0; i < contours.size(); i++ ) {
       	cv::Point2f rect_points[4]; minRect[i].points( rect_points );
   	   	for( int j = 0; j < 4; j++ ) {
			cv::line(_img, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 0, 255), 1, 8 );
		}
	}

	cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	for(;;) {
  		cv::imshow( "Contours", _img);	
		if(cv::waitKey(1) == 27)
			break;
	}

	return 0;
}
