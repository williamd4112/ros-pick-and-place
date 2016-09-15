#include <iostream>

#include <ctime>
#include <cstdlib>
#include <cstdio>

/*	Arm headers */
#include "arm.hpp"

/*	Chamfer headers */
#include "ChamferMatchingData.h"

/*	Realsense */
#ifndef REALSENSE
#define REALSENSE
#include <librealsense/rs.hpp>
#endif

/*	OpenCV headers */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*	Extern definition */
#include "extern.h"

/*	Global variables */
static Arm * g_arm;
static float g_scale;
static cv::Mat g_depth_image;
static rs::intrinsics g_color_intrin;
static rs::intrinsics g_depth_intrin;
static rs::extrinsics g_depth_to_color;

static void rs_color_to_cvmat(rs::device & dev, cv::Mat & img)
{
	const rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);

	const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color));
	
	img = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC3, (void*) color_frame);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

}

static void rs_depth_aligned_to_color_to_cvmat(rs::device & dev, cv::Mat & img)
{
	const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth_aligned_to_color);
	const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth_aligned_to_color));
	img = cv::Mat(depth_intrin.height, depth_intrin.width, CV_16UC1, (void *) depth_frame);	
}

static void depth_to_scaled_depth(cv::Mat & in, cv::Mat & out, uint16_t one_meter)
{
	in.convertTo(out, CV_8U, 255.0 / one_meter);
}

static CMResult_t locate_alphabet(cv::Mat & img)
{
	CvMat cvmat = img;
	CMResult_t result;
	read_cvmat(&cvmat, &result);

	return result;
}

static CMResult_t locate_plate(cv::Mat & img)
{
	CMResult_t cm_result;
	findplate(img, &cm_result);
	
	return cm_result;
}

static void key_handler(int key, cv::Mat & color_image)
{
	switch(key) {
		case 27:
			exit(0);
			break;
		case '1':
			locate_alphabet(color_image);
			break;
		case '2':
			locate_plate(color_image);
			break;
		default:
			break;
	}
}

static void mouse_handler(int event, int x, int y, int flag, void* param)
{
	switch(event) {
		case CV_EVENT_LBUTTONDOWN:
		{
			float depth_value = g_depth_image.at<uint16_t>(y, x);
			float depth_in_meter = depth_value * g_scale;

	        rs::float2 depth_pixel = {(float)x, (float)y};
           	rs::float3 depth_point = g_depth_intrin.deproject(depth_pixel, depth_in_meter);
            rs::float3 color_point = g_depth_to_color.transform(depth_point);
            rs::float2 color_pixel = g_color_intrin.project(color_point);

#ifdef DEBUG
			printf("Point at (%d, %d) = (%f, %f, %f)\n", y, x, color_point.x, color_point.y, color_point.z);
#endif
		}					
			break;
		default:
			break;
	}
}

int main(int argc, char * argv[])
{
	/* Initialize arm */
	g_arm = new Arm(argc, argv, "chatter", "pick_and_place");
	
	/* Initialize realsense */
    rs::log_to_console(rs::log_severity::warn);
    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

    rs::device & dev = *ctx.get_device(0);
    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.start();

	g_color_intrin = dev.get_stream_intrinsics(rs::stream::color);
	g_depth_intrin = dev.get_stream_intrinsics(rs::stream::depth_aligned_to_color);
	g_depth_to_color =  dev.get_extrinsics(rs::stream::depth_aligned_to_color, rs::stream::color);

	/* Main loop */
	g_scale = dev.get_depth_scale();
    const uint16_t one_meter = static_cast<uint16_t>(1.0f / g_scale);

    cv::namedWindow("Color Image",  CV_WINDOW_NORMAL);
	cvSetMouseCallback("Color Image", mouse_handler, NULL);

	for (;;) {
		dev.wait_for_frames();

		cv::Mat color_image;
		rs_color_to_cvmat(dev, color_image);

		rs_depth_aligned_to_color_to_cvmat(dev, g_depth_image);

		cv::Mat scale_depth_image;
		depth_to_scaled_depth(g_depth_image, scale_depth_image, one_meter);
		
#ifdef DEBUG
		cv::imshow("Color Image", color_image);
		cv::imshow("Depth Image", scale_depth_image);

		int key = cv::waitKey(1);
		key_handler(key, color_image);
#endif
	}

	return 0;
}
