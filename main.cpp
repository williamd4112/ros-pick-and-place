#include <iostream>
#include <fstream>

#include <ctime>
#include <cstdlib>
#include <cstdio>

/*	Arm headers */
#define ARM_HOVER_Z 0.015
#include "arm.hpp"

/*	Arm action headers */
#include "action.hpp"

/*	Chamfer headers */
#include "Color.h"
#include "Contour.h"
#include "Detector.h"
#include "ChamferMatchingData.h"
#include "ChamferMatcher.h"

#include "MatchingTest.h"

/*	Realsense */
#ifndef REALSENSE
#define REALSENSE
#include <librealsense/rs.hpp>
#endif

/*	OpenCV headers */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define deg(rad) ((rad) / CV_PI * 180.0)

/*	Extern definition */
#include "extern.h"

#include <mutex>

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

/* Workaround external declaration */
std::ostream & operator << (std::ostream & os, ending::Matcher::MatchPoint p)
{
	return os << "x = " << p.getBoundingBoxCenter().x << ", "
			<< "y = " << p.getBoundingBoxCenter().y << ", "
			<< "w = " << p.getBoundingBoxSize().width << ", "
			<< "h = " << p.getBoundingBoxSize().height << ", "
			<< "angle = " << deg(p.getAngle());
}

std::vector<ending::Matcher::MatchPoints> match(cv::Mat &image, std::vector<cv::Mat> & templs, std::vector<cv::Rect> boundingBoxs, float step=1.0f)
{
	cv::Mat iedge;
	colorEdgeDetection(image, iedge, true);

	ending::RChamferMatcher cmatcher(1, 20, 1.0, 3, 3, 3, 0.5, 0.6, 0.5, 20, step);

	ending::DEBUG_img = image.clone();  //if no defined __CHAMFER_DEBUG_MODE___  then remove this

	std::vector<ending::Matcher::MatchPoints> matchpoints;
	
	for (auto templ : templs) {
		cmatcher.addMatcher(templ);
	}
	cmatcher.multimatching(iedge, boundingBoxs, matchpoints);
	
	cv::imshow("debug", ending::DEBUG_img);   //if no defined __CHAMFER_DEBUG_MODE___  then remove this

	return matchpoints;
}

ending::Matcher::MatchPoints match(cv::Mat &image, cv::Mat &templ, cv::Rect boundingBox, float step=1.0f)
{

	cv::Mat iedge;
	colorEdgeDetection(image, iedge, true);

	ending::RChamferMatcher cmatcher(1, 20, 1.0, 3, 3, 3, 0.5, 0.6, 0.5, 20, step);

	ending::DEBUG_img = image.clone();  //if no defined __CHAMFER_DEBUG_MODE___  then remove this

	ending::Matcher::MatchPoints matchpoints;

	cmatcher.addMatcher(templ);
	cmatcher.matching(iedge, boundingBox, matchpoints);
	
	cv::imshow("debug", ending::DEBUG_img);   //if no defined __CHAMFER_DEBUG_MODE___  then remove this

/*
	std::vector<cv::Mat> templs;
	templs.push_back(templ);
	
	std::vector<cv::Rect> boundingBoxs;
	boundingBoxs.push_back(boundingBox);
*/	
	return matchpoints;
}

/*	Global variables */
Arm * g_arm;
static ending::MatchingTest * g_matching_test_ptr;
static std::vector<cv::Rect> g_scale_depth_box;
static float g_scale;
static float g_hover_height = ARM_HOVER_Z;
static float g_pick_height = ARM_HOVER_Z;
static float g_place_height = ARM_HOVER_Z;
static cv::Mat g_depth_image;
static rs::intrinsics g_color_intrin;
static rs::intrinsics g_depth_intrin;
static rs::extrinsics g_depth_to_color;
static std::mutex g_depth_mutex;
static rs::float3 g_pointcloud[FRAME_WIDTH * FRAME_HEIGHT];
static rs::float3 g_origin;
static float g_plate_angle = 0.0f;
static float g_target_height_camera = 0.46f;
static float g_correct_scale_x = 1.08f;
static float g_correct_scale_y = 1.08f;

static struct camera_calibration_t {
	/* In meters */
	rs::float3 ref;
	
	rs::float3 calibrate(const rs::float3 p)
	{
		return rs::float3 {-(p.y - ref.y), -(p.x - ref.x), -(p.z - ref.z)};
	}
	
	void load_config()
	{
		ref.x = -0.135416f;
		ref.y = 0.143554f;
		ref.z = 0.4513f + g_hover_height;
	}

} g_camera_calibration;

struct color_def_t
{
	std::string name;
	cv::Scalar lower;
	cv::Scalar upper;
};

static struct pick_and_place_config_t {
	Arm::target_t src;
	Arm::target_t dst;
	std::string plate_template_path;
	
	/* Retry parameter */
	float retry_offset;
	float retry_inteval;
	
	/* Matching input */
	std::vector<color_def_t> color_defs;
	std::vector<char> alphabets;
	std::vector<cv::Rect> alphabets_bounding_boxs;

	/* Matching result */
	ending::Matcher::MatchPoints plate_match;
	std::vector<ending::Matcher::MatchPoints> alphabet_matches;
	ending::Matcher::MatchPoint slot_match;	

} g_pick_and_place_config;

static bool g_workspace_enable = true;
static int32_t g_screenshot_cnt = 0;

static std::vector<cv::Mat> g_templates;
#ifdef DEBUG
static std::vector<cv::Rect> g_template_box;
#endif
//static cv::Rect g_cm_roi(cv::Point2i(102, 64), cv::Point2i(252, 176));
static cv::Rect g_cm_roi(cv::Point2i(190, 110), cv::Point2i(252, 176));
static cv::Rect g_cm_search_roi(cv::Point2i(0, 0), cv::Point2i(300, 220));
static cv::Rect g_plate_roi(cv::Point2i(300, 0), cv::Point2i(640, 480));

static void mouse_handler(int event, int x, int y, int flag, void* param);

static void draw_scale_depth_box(cv::Mat & img)
{
	for (auto box : g_scale_depth_box) {
		cv::rectangle(img, box, cv::Scalar(255, 255, 0), 1);
	}
}

static void draw_workspace(cv::Mat & img)
{
	if (g_workspace_enable) {
		cv::rectangle(img, cv::Point2i(262, 306), cv::Point2i(96, 476), cv::Scalar(0, 0, 255), 3);	  
		//cv::rectangle(img, g_cm_roi, cv::Scalar(0, 255, 0), 3);
		cv::rectangle(img, g_plate_roi, cv::Scalar(255, 255, 0), 3);
		cv::rectangle(img, g_cm_search_roi, cv::Scalar(255, 0, 255), 3);
		
		for (auto rect : g_template_box) {
			cv::rectangle(img, rect, cv::Scalar(255, 0, 0), 3);
			cv::circle(img, cv::Point( (rect.tl().x + rect.br().x)/2, (rect.tl().y + rect.br().y)/2 ), 
				3, cv::Scalar(0, 0, 255));
		}
		
		cv::circle(img, g_pick_and_place_config.slot_match.getBoundingBoxCenter(), 3, cv::Scalar(0, 0, 255));
		
		if (g_pick_and_place_config.alphabet_matches.size()) {
			if (g_pick_and_place_config.alphabet_matches.front().size()) {
				auto pick = g_pick_and_place_config.alphabet_matches.front().front();
				cv::circle(img, pick.getBoundingBoxCenter(), 3, cv::Scalar(0, 0, 255));
			}
		}
	}
}

static rs::float3 pointcloud_get_point(int x, int y)
{
	static int kSize = 17;

	rs::float3 point;
	g_depth_mutex.lock();
	point = g_pointcloud[FRAME_WIDTH * y + x];
	if (point.x == 0 && point.y == 0 && point.z == 0) {
		rs::float3 avg = rs::float3{0, 0, 0};
		bool found = false;
		for (int k = 1; k < kSize && !found; k++) {
			for (int i = y - k; i <= y + k && !found; i++) {
				for (int j = x - k; j <= x + k && !found; j++) {
					if (i < 0 || i >= FRAME_HEIGHT || j < 0 || j >= FRAME_WIDTH)
						continue;
					/// TODO : for now, use first found point
					avg = g_pointcloud[FRAME_WIDTH * i + j];
					if (avg.z < g_target_height_camera) {
						continue;
					} 
					found = true;
					std::cout << "Found nearest" << std::endl;
				}
			}
		}
		point = avg;
	}
	g_depth_mutex.unlock();

	return point;
}

static void set_pick_target(int x, int y, float angle)
{
	rs::float3 point = pointcloud_get_point(x, y);
	rs::float3 calibrate_point = g_camera_calibration.calibrate(point);	

	g_pick_and_place_config.src.x = calibrate_point.x * 1000.0f;
	g_pick_and_place_config.src.y = calibrate_point.y * 1000.0f;
	g_pick_and_place_config.src.z = calibrate_point.z * 1000.0f;
	g_pick_and_place_config.src.angle = angle;
#ifdef DEBUG
	printf("Point (%f, %f, %f)\n",
		calibrate_point.x,
		calibrate_point.y,
		calibrate_point.z);
	printf("(Src) Target (%f, %f, %f)\n", 
		g_pick_and_place_config.src.x,
		g_pick_and_place_config.src.y,
		g_pick_and_place_config.src.z);
#endif
}

static void set_place_target(int x, int y, float angle)
{
	rs::float3 point = pointcloud_get_point(x, y);
	rs::float3 calibrate_point = g_camera_calibration.calibrate(point);	

	g_pick_and_place_config.dst.x = calibrate_point.x * 1000.0f;
	g_pick_and_place_config.dst.y = calibrate_point.y * 1000.0f;
	g_pick_and_place_config.dst.z = calibrate_point.z * 1000.0f;
	g_pick_and_place_config.dst.angle = angle;
#ifdef DEBUG
	printf("Point (%f, %f, %f)\n",
		calibrate_point.x,
		calibrate_point.y,
		calibrate_point.z);
	printf("(Dst) Target (%f, %f, %f)\n", 
		g_pick_and_place_config.dst.x,
		g_pick_and_place_config.dst.y,
		g_pick_and_place_config.dst.z);
#endif		
}

static void rs_color_to_cvmat(rs::intrinsics & color_intrin, cv::Mat & img, const uint8_t * color_frame)
{	
	img = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC3, (void*) color_frame);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
}

static void rs_depth_aligned_to_color_to_cvmat(rs::intrinsics & depth_intrin, cv::Mat & img, const uint16_t * depth_frame)
{
	img = cv::Mat(depth_intrin.height, depth_intrin.width, CV_16UC1, (void *) depth_frame);	
}

static void depth_to_scaled_depth(cv::Mat & in, cv::Mat & out, uint16_t one_meter)
{
	in.convertTo(out, CV_8U, 255.0 / one_meter);
}

static void locate_scaled_depth_box(cv::Mat & img, int area_ts=50)
{
	g_scale_depth_box.clear();
	cv::Mat img_thresh;
	cv::threshold(img(g_cm_search_roi), img_thresh, 0, 255, CV_THRESH_BINARY);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hier;

	cv::findContours(img_thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	static float scale = 0.4f;
	std::vector<cv::Moments> mu;
	for(auto contour : contours) {
		if (cv::contourArea(contour) > area_ts) {
			cv::Rect box = cv::boundingRect(contour);
			box.x -= (box.width / 2) * scale;
			box.y -= (box.height / 2) * scale;
			box.width = box.width + (box.width) * scale;;
			box.height =  box.height + (box.height) * scale;
			g_scale_depth_box.push_back(box);
			mu.push_back(cv::moments(contour, false));
		}
	}
}

static void inRangeHSV(cv::Mat src, cv::Rect box, cv::Scalar lower, cv::Scalar upper, cv::Mat & dst)
{
	if (lower.val[0] > upper.val[0]) {
		cv::Mat mask[2];
		cv::inRange(src(box), upper, cv::Scalar(179, 255, 255), mask[0]);
		cv::inRange(src(box), cv::Scalar(0, 100, 100), lower, mask[1]);
		cv::add(mask[0], mask[1], dst);
	}
	else {
		cv::inRange(src(box), lower, upper, dst);
	}
}

static int detect_color(cv::Mat & hsv, cv::Rect box)
{
	static int area_ts = 100;
	int id = -1;

	for (int i = 0; i == -1 && i < g_pick_and_place_config.color_defs.size(); i++) {
		color_def_t & def = g_pick_and_place_config.color_defs[i];
		cv::Mat mask;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hier;

		inRangeHSV(hsv, box, def.lower, def.upper, mask);
#ifdef DEBUG_COLOR
		cv::imshow(def.name, mask);
#endif	
		cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		for(auto contour : contours) {
			int area = cv::contourArea(contour);
			std::cout << "Detect " << def.name <<  " = " << area << std::endl;
			if (area > area_ts) {
				id = i;
				break;
			}
		}
	}
	
	return id;	
}

static void locate_alphabet_with_scale_depth(cv::Mat & img)
{
	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_BGR2HSV);

	g_pick_and_place_config.alphabets.clear();
	g_pick_and_place_config.alphabets_bounding_boxs.clear();	

	for (auto box : g_scale_depth_box) {
		int color_id = detect_color(hsv, box);
		std::vector<char> alphabet_candidates = g_matching_test_ptr->getColorList(color_id);
		for (char ch : alphabet_candidates) {
			g_pick_and_place_config.alphabets.push_back(ch);
			g_pick_and_place_config.alphabets_bounding_boxs.push_back(box);
		}
	}
}

static void key_handler(int key, cv::Mat & color_image, cv::Mat & scale_depth_image)
{
	switch(key) {
		case 27:
			exit(0);
			break;
		case 'W': case 'w':
			g_workspace_enable = !g_workspace_enable;
			break;
		case 'R': case 'r':
			g_arm->touch(150, 0, 0);
			break;
		case 'D': case 'd':
		{
			std::cout << "Locate scaled depth box begin" << std::endl;
			locate_scaled_depth_box(scale_depth_image);
			std::cout << "Locate scaled depth box end" << std::endl;
		}
			break;
		case 'F': case 'f':
		{
			std::cout << "Test with scaled depth box begin" << std::endl;
			g_pick_and_place_config.alphabets_bounding_boxs = g_scale_depth_box;
			std::cout << "Test with scaled depth box end" << std::endl;
		}
			break;
		case '0':
		{
			std::cout << "Set image begin" << std::endl;
			g_matching_test_ptr->setImage(color_image);
			std::cout << "Set image end" << std::endl;
		}
			break;
		case '1':
		{
			std::cout << "Locate alphabets begin" << std::endl;
			g_pick_and_place_config.alphabet_matches = g_matching_test_ptr->matchPuzzle(g_pick_and_place_config.alphabets, g_pick_and_place_config.alphabets_bounding_boxs);
			std::cout << "Locate alphabets end" << std::endl;
			
		}
			break;
		case '2':
			std::cout << "Locate plate begin" << std::endl;
			g_pick_and_place_config.plate_match = g_matching_test_ptr->matchPlate(g_plate_roi);
			std::cout << "Locate plate end" << std::endl;
			break;
		case '3':
		{
			char buff[100];
			sprintf(buff, "screenshot%d.jpg", g_screenshot_cnt);
			cv::imwrite(buff, color_image);
			std::cout << "Save screenshot" << (g_screenshot_cnt) << std::endl;
			g_screenshot_cnt++;
		}
			break;
		case '4':
		{
			std::cout << "Pick and place" << std::endl;
			auto pick = g_pick_and_place_config.src;
			auto place = g_pick_and_place_config.dst;

			pick.z = g_pick_height;
			place.z = g_place_height;
			
			printf("(%f, %f, %f, %f, %f, %f, %f, %f)\n",
				pick.x, pick.y, pick.y, pick.angle,
				place.x, place.y, place.z, place.angle);		
			
			g_arm->grab_and_place(pick, place);
		}
			break;
		case '5':
		{
			std::cout << "Touch" << std::endl;
			g_arm->touch(
				g_pick_and_place_config.src.x, 
				g_pick_and_place_config.src.y, 
				g_pick_and_place_config.src.z);
		}
			break;
		case '6':
		{
			std::cout << "Calibrate" << std::endl;
			g_camera_calibration.ref.x = g_origin.x + 0.03;
			g_camera_calibration.ref.y = g_origin.y + 0.03;
			g_camera_calibration.ref.z = g_origin.z + 0.03;
		}
			break;
		case '7':
		{
			std::cout << "Retry" << std::endl;
			action_retry(1, 15.0f, 10.0f, 100000);
		}
			break;
		case '8':
		{
			std::cout << "Find hole begin" << std::endl;
			if (g_pick_and_place_config.alphabets.size()) {
				char top_alphabet = g_pick_and_place_config.alphabets.front();
				g_pick_and_place_config.slot_match =  g_matching_test_ptr->matchHole(top_alphabet);
					
				std::cout << g_pick_and_place_config.slot_match << std::endl;
			}
			else {
				std::cout << "No candidate alphabets." << std::endl;
			}
			std::cout << "Find hole end" << std::endl;
		}
			break;
		case '9':
		{
			/* Setup pick target */
			if (g_pick_and_place_config.alphabet_matches.size()) {
				auto pick = g_pick_and_place_config.alphabet_matches.front().front();
				set_pick_target(pick.getBoundingBoxCenter().x,
								pick.getBoundingBoxCenter().y,
								deg(pick.getAngle()));
			}
			else {
				std::cout << "No pick target." << std::endl;
			}

			/* Setup place target */
			auto place = g_pick_and_place_config.slot_match;
			set_place_target(place.getBoundingBoxCenter().x, 
							place.getBoundingBoxCenter().y, 
							deg(place.getAngle()));
		
			printf("Pick(%f, %f, %f, %f)\tPlace(%f, %f, %f, %f)\n",
				g_pick_and_place_config.src.x, 
				g_pick_and_place_config.src.y,
				g_pick_and_place_config.src.z,
				(g_pick_and_place_config.src.angle),
				
				g_pick_and_place_config.dst.x,
				g_pick_and_place_config.dst.y,
				g_pick_and_place_config.dst.z,
				(g_pick_and_place_config.dst.angle));
		}	
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
			rs::float3 point = pointcloud_get_point(x, y);
			printf("Click at (%d, %d) = (%f, %f, %f)\n", x, y,
					point.x, point.y, point.z);
			set_pick_target(x, y, 0);
		}					
			break;
		case CV_EVENT_RBUTTONDOWN:
		{
			set_place_target(x, y, g_plate_angle);
		}		
			break;
		default:
			break;
	}
}

static void load_global_config(const char * filename)
{
	std::ifstream fin(filename);
	
	rs::float3 ref;
	fin >> ref.x >> ref.y >> ref.z;

	std::cout << "Load Camera offset (" << ref.x << ", " << ref.y << ", " << ref.z << ")" << std::endl;
	
	fin >> g_hover_height;
	fin >> g_correct_scale_x >> g_correct_scale_y;
	std::cout << "Scale (" << g_correct_scale_x << ", " << g_correct_scale_y << ")" <<std::endl;

	ref.z += g_hover_height;
	g_camera_calibration.ref = ref;

	fin >> g_pick_height >> g_place_height;
	std::cout << "Pick height : " << g_pick_height << std::endl;
	std::cout << "Place height : " << g_place_height << std::endl;

	fin >> g_pick_and_place_config.plate_template_path;
	std::cout << "Plate template : " << g_pick_and_place_config.plate_template_path << std::endl;

	fin >> g_plate_angle;
	std::cout << "Plate angle : " << g_plate_angle << std::endl;

	fin >> g_target_height_camera;
	std::cout << "Target height (camera space) : " << g_target_height_camera << std::endl;

	std::cout << "Color defs" << std::endl;
	for (int i = 0; i < 7; i++) {
		color_def_t def;
		fin >> def.name 
			>> def.lower.val[0] >> def.lower.val[1] >> def.lower.val[2]
			>> def.upper.val[0] >> def.upper.val[1] >> def.upper.val[2];
		g_pick_and_place_config.color_defs.push_back(def);
	} 
}

void load_default_config()
{
	g_pick_and_place_config.plate_template_path = "edge_x04/platex01.png";
}

void load_cm_template(const char * path)
{
#ifdef TEST_TEMPLATE
		char ch = path[0];
		char buff[100];
		sprintf(buff, "edge_x04/%c.png", ch);
		std::cout << "Load template " << buff << std::endl;	
		cv::Mat img = cv::imread(buff);
		cv::cvtColor(img, img, CV_BGR2GRAY);
		g_templates.push_back(img);
#ifdef DEBUG
		g_template_box.push_back(cv::Rect());
#endif
		g_pick_and_place_config.alphabets.push_back(path[0]);
		g_pick_and_place_config.alphabets_bounding_boxs.push_back(g_cm_roi);
#else

	static int num = 1;
	for (int i = 0; i < num; i++) {
		char ch = 'E' + (uint8_t)(i & 0xff);	
		char buff[100];
		sprintf(buff, "%s/%c.png", path, ch);
		
		cv::Mat img = cv::imread(buff);
		cv::cvtColor(img, img, CV_BGR2GRAY);
		g_templates.push_back(img);
#ifdef DEBUG
		g_template_box.push_back(cv::Rect());
#endif
		std::cout << "Load template " << ch << ".png" << std::endl;
	}
#endif
}

int main(int argc, char * argv[])
{
	/* Load default config */
	load_default_config();

	/* Load CM template */
#ifdef TEST_TEMPLATE
	if (argc >= 3) {
		std::cout << "Load " << argv[2] << std::endl;
		load_cm_template((const char *)argv[2]);
	}
	else {
		load_cm_template("A");
	}
#else
	load_cm_template("edge_x04");
#endif

	/* Initialize camera */
	g_camera_calibration.load_config();
	
	if (argc >= 2) {
		load_global_config(argv[1]);
	}
	
	/* Initialize matching test */
	std::ifstream matching_test_file("config.ini");
	ending::MatchingTest matching_test(matching_test_file);
	matching_test.init();
	g_matching_test_ptr = &matching_test;

	/* Initialize arm */
	g_arm = new Arm(argc, argv, "pick_and_place", "chatter");
	
	/* Initialize realsense */
    rs::log_to_console(rs::log_severity::warn);
    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

    rs::device & dev = *ctx.get_device(0);
    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.start();

	g_color_intrin = dev.get_stream_intrinsics(rs::stream::rectified_color);
	g_depth_intrin = dev.get_stream_intrinsics(rs::stream::depth_aligned_to_rectified_color);
	g_depth_to_color = dev.get_extrinsics(rs::stream::depth_aligned_to_rectified_color, rs::stream::rectified_color);
	g_scale = dev.get_depth_scale();

	/* Main loop */
    const uint16_t one_meter = static_cast<uint16_t>(1.0f / g_scale);

    cv::namedWindow("Color Image",  CV_WINDOW_NORMAL);
	cvSetMouseCallback("Color Image", mouse_handler, NULL);

    cv::namedWindow("Depth Image",  CV_WINDOW_NORMAL);
	cvSetMouseCallback("Depth Image", mouse_handler, NULL);

#ifdef VIDEO_PLAYBACK
    int frame_width = (FRAME_WIDTH);
    int frame_height = (FRAME_HEIGHT);
	cv::VideoWriter playback("playback.avi",CV_FOURCC('M','J','P','G'), 10, cv::Size(frame_width, frame_height),true);
	std::cout << "Video writter starting ... " << std::endl;
#endif

	for (;;) {
		dev.wait_for_frames();

		const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth_aligned_to_rectified_color));
		const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::rectified_color));

		cv::Mat color_image;
		rs_color_to_cvmat(g_color_intrin, color_image, color_frame);
	
		rs_depth_aligned_to_color_to_cvmat(g_depth_intrin, g_depth_image, depth_frame);
        memset(g_pointcloud, 0, sizeof(g_pointcloud));
        
        g_depth_mutex.lock();  
        for(int dy=0; dy<g_depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<g_depth_intrin.width; ++dx)
            {
                uint16_t depth_value = depth_frame[dy * g_depth_intrin.width + dx];
                float depth_in_meters = depth_value * g_scale;

				if(depth_value == 0) continue;

                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = g_depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = g_depth_to_color.transform(depth_point);
                rs::float2 color_pixel = g_color_intrin.project(color_point);
				
				depth_point.x *= g_correct_scale_x;
				depth_point.y *= g_correct_scale_y;
			
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y); 
                g_pointcloud[cy * FRAME_WIDTH + cx] = depth_point;
            }
        }
        g_depth_mutex.unlock();

		cv::Mat scale_depth_image;
		depth_to_scaled_depth(g_depth_image, scale_depth_image, one_meter);
		
				
#ifdef VIDEO_PLAYBACK
		if (!color_image.empty()) {
			playback.write(color_image);
		}
#endif
		cv::Mat _color_image = color_image.clone();
		draw_workspace(_color_image);
		draw_scale_depth_box(_color_image);
#ifdef DEBUG
		cv::imshow("Color Image", _color_image);
		cv::imshow("Depth Image", scale_depth_image);

		int key = cv::waitKey(1) & 0xff;
		key_handler(key, color_image, scale_depth_image);
		if (key == 10) {
#ifdef VIDEO_PLAYBACK
			playback.release();
			std::cout << "Video writter release" << std::endl;
#endif
		}
#endif
	}

	return 0;
}
