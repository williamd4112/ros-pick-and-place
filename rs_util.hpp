#ifndef _RS_UTIL_HPP_
#define _RS_UTIL_HPP_

#include <opencv2/core/core.hpp>

#ifndef REALSENSE
#define REALSENSE
#include <librealsense/rs.hpp>
#endif

void rs_color_to_cvmat(rs::device & dev, cv::Mat & img);

#endif
